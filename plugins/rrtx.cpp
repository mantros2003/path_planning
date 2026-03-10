#include <custom_nav/rrtx.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>

#include <random>
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTxPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

//  Helpers

RRTxNode& RRTxPlanner::node(unsigned int idx) {
    auto it = nodes_.find(idx);
    if (it == nodes_.end()) {
        nodes_.emplace(idx, RRTxNode(idx));
        return nodes_.at(idx);
    }
    return it->second;
}

double RRTxPlanner::distance(int x1, int y1, int x2, int y2) const {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double RRTxPlanner::edgeCost(unsigned int from, unsigned int to) const {
    int fx = from % width_, fy = from / width_;
    int tx = to   % width_, ty = to   / width_;
    return distance(fx, fy, tx, ty);
}

bool RRTxPlanner::hasObstacle(unsigned int from, unsigned int to) const {
    int sx = from % width_, sy = from / width_;
    int gx = to   % width_, gy = to   / width_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy);
         line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >=
            costmap_2d::LETHAL_OBSTACLE)
            return true;
    }
    return false;
}

unsigned int RRTxPlanner::steer(unsigned int from, unsigned int to,
                                double step) const {
    int sx = from % width_, sy = from / width_;
    int gx = to   % width_, gy = to   / width_;
    double dx = gx - sx, dy = gy - sy;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len <= step) return to;

    int nx = sx + static_cast<int>(std::round(step * dx / len));
    int ny = sy + static_cast<int>(std::round(step * dy / len));
    nx = std::max(0, std::min((int)width_  - 1, nx));
    ny = std::max(0, std::min((int)height_ - 1, ny));
    return costmap_->getIndex(nx, ny);
}

// shrinking-ball radius:  r(n) = min(δ, γ · (ln n / n)^(1/d))
// d=2 for 2-D grids; γ is set so that r never exceeds step_size_
double RRTxPlanner::shrinkingBallRadius() const {
    unsigned int n = static_cast<unsigned int>(nodes_.size());
    if (n < 2) return step_size_;
    const double gamma = 2.0 * step_size_;          // tunable
    double r = gamma * std::sqrt(std::log((double)n) / (double)n);
    return std::min(r, step_size_);
}

//  Priority queue

QKey RRTxPlanner::makeKey(unsigned int v) const {
    const RRTxNode& nd = nodes_.at(v);
    double k1 = std::min(nd.g, nd.lmc);
    double k2 = nd.lmc;
    return { k1, k2, v };
}

bool RRTxPlanner::keyLess(const QKey& a, const QKey& b) const {
    if (a.k1 != b.k1) return a.k1 < b.k1;
    return a.k2 < b.k2;
}

void RRTxPlanner::verifyQueue(unsigned int v) {
    removeFromQueue(v);
    QKey k = makeKey(v);
    queue_.insert(k);
    inQueue_[v] = k;
}

void RRTxPlanner::removeFromQueue(unsigned int v) {
    auto it = inQueue_.find(v);
    if (it != inQueue_.end()) {
        queue_.erase(it->second);
        inQueue_.erase(it);
    }
}

//  Algorithm 6 – findParent(v, U)

void RRTxPlanner::findParent(unsigned int v,
                              const std::vector<unsigned int>& U) {
    RRTxNode& vn = node(v);
    for (unsigned int u : U) {
        if (!nodes_.count(u)) continue;          // u not yet in tree
        if (hasObstacle(v, u))  continue;

        double d = edgeCost(v, u);
        RRTxNode& un = node(u);

        if (d <= shrinkingBallRadius() &&
            vn.lmc > d + un.lmc) {
            vn.lmc    = d + un.lmc;
            vn.parent = u;
            vn.hasParent = true;
        }
    }
}

//  Algorithm 2 – extend(v, r)

void RRTxPlanner::extend(unsigned int v, double r) {
    // 1. Collect nodes within radius r
    int vx = v % width_, vy = v / width_;
    // Use kd-tree to find neighbours within r cells
    // (kdTree::nearest gives one point; for RRTx we need all within r)
    // We approximate with brute-force over tree nodes — for large maps
    // replace with a proper range-search on the kd-tree.
    std::vector<unsigned int> Vnear;
    for (auto& kv : nodes_) {
        unsigned int u = kv.first;
        int ux = u % width_, uy = u / width_;
        if (distance(vx, vy, ux, uy) <= r)
            Vnear.push_back(u);
    }

    // 2. findParent (Algorithm 6)
    findParent(v, Vnear);

    // 3. If no valid parent found, do not add node
    RRTxNode& vn = node(v);
    if (!vn.hasParent) {
        nodes_.erase(v);
        return;
    }

    // 4. Add v to vertex set (already in nodes_) and update parent's children
    node(vn.parent).children.insert(v);

    // 5. Build neighbourhood sets for v and its neighbours
    for (unsigned int u : Vnear) {
        if (u == v) continue;
        RRTxNode& un = node(u);

        // π(v, u): edge from v to u (running-in  for u)
        if (!hasObstacle(v, u)) {
            vn.N0_plus.insert(u);       // v can reach u
            un.Nr_minus.insert(v);      // u is reachable from v
        }

        // π(u, v): edge from u to v (running-in  for v)
        if (!hasObstacle(u, v)) {
            un.N0_plus.insert(v);       // u can reach v ... wait—
            // Algorithm 2, lines 11-13:
            //   Nr+(u) ← Nr+(u) ∪ {v}  (u can run into v)
            //   N0-(v) ← N0-(v) ∪ {u}
            un.Nr_plus.insert(v);
            vn.N0_minus.insert(u);
        }
    }
}

//  Algorithm 3 – cullNeighbors(v, r)

void RRTxPlanner::cullNeighbors(unsigned int v, double r) {
    RRTxNode& vn = node(v);
    std::vector<unsigned int> toRemove;

    for (unsigned int u : vn.Nr_plus) {
        double d = edgeCost(v, u);
        bool isParent = vn.hasParent && (vn.parent == u);
        if (r < d && !isParent) {
            toRemove.push_back(u);
        }
    }

    for (unsigned int u : toRemove) {
        vn.Nr_plus.erase(u);
        if (nodes_.count(u))
            node(u).Nr_minus.erase(v);
    }
}

//  Algorithm 4 – rewireNeighbors(v)

void RRTxPlanner::rewireNeighbors(unsigned int v) {
    RRTxNode& vn = node(v);
    double r = shrinkingBallRadius();

    if (vn.g - vn.lmc > epsilon_)
        cullNeighbors(v, r);

    // Build N-(v) \ {p+(v)}
    std::unordered_set<unsigned int> Nminus;
    for (unsigned int u : vn.Nr_minus)  Nminus.insert(u);
    for (unsigned int u : vn.N0_minus)  Nminus.insert(u);
    if (vn.hasParent) Nminus.erase(vn.parent);

    for (unsigned int u : Nminus) {
        if (!nodes_.count(u)) continue;
        RRTxNode& un = node(u);

        double d = edgeCost(u, v);
        if (un.lmc > d + vn.lmc) {
            un.lmc = d + vn.lmc;

            // Remove u from old parent's children, assign new parent
            if (un.hasParent)
                node(un.parent).children.erase(u);
            un.parent    = v;
            un.hasParent = true;
            vn.children.insert(u);

            if (un.g - un.lmc > epsilon_)
                verifyQueue(u);
        }
    }
}

//  updateLMC helper – pick best parent from neighbours

void RRTxPlanner::updateLMC(unsigned int v) {
    RRTxNode& vn = node(v);

    // Merge all potential parents: Nr+(v) ∪ N0+(v)
    std::unordered_set<unsigned int> candidates;
    for (unsigned int u : vn.Nr_plus)  candidates.insert(u);
    for (unsigned int u : vn.N0_plus)  candidates.insert(u);
    if (vn.hasParent) candidates.erase(vn.parent);

    for (unsigned int u : candidates) {
        if (!nodes_.count(u)) continue;
        RRTxNode& un = node(u);
        if (hasObstacle(v, u)) continue;

        double d = edgeCost(v, u);
        if (d + un.lmc < vn.lmc) {
            vn.lmc = d + un.lmc;
            if (vn.hasParent) node(vn.parent).children.erase(v);
            vn.parent    = u;
            vn.hasParent = true;
            un.children.insert(v);
        }
    }
}

//  Algorithm 5 – reduceInconsistency()

void RRTxPlanner::reduceInconsistency() {
    const RRTxNode& bot = nodes_.at(start_index_);

    auto topKey = [&]() -> QKey { return *queue_.begin(); };
    QKey botKey = makeKey(start_index_);

    while (!queue_.empty() &&
           (keyLess(topKey(), botKey) ||
            std::abs(bot.lmc - bot.g) > epsilon_ ||
            bot.g == std::numeric_limits<double>::infinity() ||
            inQueue_.count(start_index_))) {

        unsigned int v = queue_.begin()->index;
        queue_.erase(queue_.begin());
        inQueue_.erase(v);

        RRTxNode& vn = node(v);

        if (vn.g - vn.lmc > epsilon_) {
            updateLMC(v);
            rewireNeighbors(v);
        }
        vn.g = vn.lmc;

        // Re-evaluate bot key each iteration
        botKey = makeKey(start_index_);
    }
}

//  nav_core interface

void RRTxPlanner::initialize(std::string name,
                              costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_    = costmap_ros->getCostmap();
        initialized_ = true;
        height_     = costmap_->getSizeInCellsY();
        width_      = costmap_->getSizeInCellsX();
        ROS_INFO("[RRTx] Initialized: map %u × %u", width_, height_);
    } else {
        ROS_WARN("[RRTx] Already initialized");
    }
}

bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("[RRTx] Planning…");
    ros::Time t0 = ros::Time::now();

    // Map coordinates 
    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                               sx, sy) ||
        !costmap_->worldToMap(goal.pose.position.x,  goal.pose.position.y,
                               gx, gy)) {
        ROS_ERROR("[RRTx] Start or goal outside map");
        return false;
    }

    goal_index_  = costmap_->getIndex(gx, gy);
    start_index_ = costmap_->getIndex(sx, sy);

    // Reset state
    nodes_.clear();
    queue_.clear();
    inQueue_.clear();
    tree_ = kdTree();

    // Initialise goal node (RRTx grows from goal toward robot)
    RRTxNode& goalNode = node(goal_index_);
    goalNode.lmc = 0.0;
    goalNode.g   = 0.0;

    point gpt = { (int)gx, (int)gy };
    tree_.insert(gpt);

    // Robot starts at v_start
    unsigned int v_bot = start_index_;
    node(v_bot);    // ensure entry exists

    // Random-sampling loop
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> udist(0,
                                                       width_ * height_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    bool goal_reached = false;

    for (unsigned int iter = 0; iter < max_iters_; ++iter) {
        // r = shrinkingBallRadius
        double r = shrinkingBallRadius();

        // v = randomNode(S) with goal-bias toward v_bot
        unsigned int rand_idx;
        if (rand01(rng) < goal_bias_) {
            rand_idx = v_bot;           // bias toward robot position
        } else {
            rand_idx = udist(rng);
        }

        // v_nearest = nearest(v)
        point rand_pt = { (int)(rand_idx % width_),
                          (int)(rand_idx / width_) };
        point near_pt = tree_.nearest(rand_pt);
        unsigned int near_idx = costmap_->getIndex(near_pt.first,
                                                    near_pt.second);

        // steer: new candidate node
        unsigned int new_idx = steer(near_idx, rand_idx, step_size_);

        // Skip if already in tree or if it is an obstacle
        if (nodes_.count(new_idx)) continue;
        if (costmap_->getCost(new_idx % width_, new_idx / width_) >=
            costmap_2d::LETHAL_OBSTACLE) continue;

        // if v ∉ X_obs then extend(v, r)
        // Create node entry and attempt extend
        node(new_idx);      // initialise with inf costs

        extend(new_idx, r);

        // If extend failed (no parent found), node was erased — skip
        if (!nodes_.count(new_idx)) continue;

        // Insert into kd-tree
        point new_pt = { (int)(new_idx % width_),
                         (int)(new_idx / width_) };
        tree_.insert(new_pt);

        // if v ∈ V then rewireNeighbors + reduceInconsistency
        rewireNeighbors(new_idx);
        reduceInconsistency();

        // Check whether robot node is now connected
        if (nodes_.count(v_bot)) {
            const RRTxNode& botNode = nodes_.at(v_bot);
            if (botNode.lmc < std::numeric_limits<double>::infinity()) {
                goal_reached = true;
                break;
            }
        }
    }

    if (!goal_reached) {
        ROS_WARN("[RRTx] Failed to find a path");
        return false;
    }

    // Reconstruct path from v_bot → goal 
    plan.clear();
    unsigned int curr = v_bot;
    unsigned int safety = 0;

    while (curr != goal_index_ && safety++ < width_ * height_) {
        unsigned int cx = curr % width_, cy = curr / width_;
        double wx, wy;
        costmap_->mapToWorld(cx, cy, wx, wy);

        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx;
        p.pose.position.y = wy;
        plan.push_back(p);

        if (!nodes_.count(curr) || !nodes_.at(curr).hasParent) {
            ROS_WARN("[RRTx] Broken tree during reconstruction");
            return false;
        }
        curr = nodes_.at(curr).parent;
    }

    // Push goal
    {
        double wx, wy;
        costmap_->mapToWorld(gx, gy, wx, wy);
        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx;
        p.pose.position.y = wy;
        plan.push_back(p);
    }

    ROS_INFO("[RRTx] Path found in %.4f s, %zu poses",
             (ros::Time::now() - t0).toSec(), plan.size());
    return true;
}

} // namespace custom_planner