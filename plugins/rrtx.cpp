/*
 * RRTx Global Planner – ROS nav_core plugin
 *
 * Implements the full RRTx algorithm including:
 *   • Algorithms 1-6 from the paper
 *   • Dynamic obstacle handling via costmap snapshot diffing
 *   • Subtree orphaning (propagateOrphan) when edges become blocked
 *   • Orphan re-integration (reintegrateOrphans) when obstacles clear
 *   • Persistent tree reuse across makePlan() calls
 */

#include <custom_nav/rrtx.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>

#include <random>
#include <algorithm>
#include <queue>
#include <cmath>
#include <cstring>

PLUGINLIB_EXPORT_CLASS(custom_planner::RRTxPlanner,
                       nav_core::BaseGlobalPlanner)

namespace custom_planner {

static constexpr double INF = std::numeric_limits<double>::infinity();

// =============================================================================
//  Node map accessor
// =============================================================================

RRTxNode& RRTxPlanner::node(unsigned int idx) {
    auto [it, inserted] = nodes_.emplace(idx, RRTxNode(idx));
    return it->second;
}

// =============================================================================
//  Geometry / costmap helpers
// =============================================================================

double RRTxPlanner::dist(int x1, int y1, int x2, int y2) const {
    double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double RRTxPlanner::edgeCost(unsigned int from, unsigned int to) const {
    return dist((int)(from % width_), (int)(from / width_),
                (int)(to   % width_), (int)(to   / width_));
}

bool RRTxPlanner::hasObstacle(unsigned int from, unsigned int to) const {
    int sx = (int)(from % width_), sy = (int)(from / width_);
    int gx = (int)(to   % width_), gy = (int)(to   / width_);
    for (base_local_planner::LineIterator ln(sx, sy, gx, gy);
         ln.isValid(); ln.advance()) {
        if (costmap_->getCost(ln.getX(), ln.getY()) >= costmap_2d::LETHAL_OBSTACLE)
            return true;
    }
    return false;
}

bool RRTxPlanner::edgePassesThrough(unsigned int from,
                                     unsigned int to,
                                     unsigned int cell) const {
    int sx = (int)(from % width_), sy = (int)(from / width_);
    int gx = (int)(to   % width_), gy = (int)(to   / width_);
    int cx = (int)(cell % width_), cy = (int)(cell / width_);
    for (base_local_planner::LineIterator ln(sx, sy, gx, gy);
         ln.isValid(); ln.advance()) {
        if (ln.getX() == cx && ln.getY() == cy) return true;
    }
    return false;
}

unsigned int RRTxPlanner::steer(unsigned int from, unsigned int to,
                                double step) const {
    int sx = (int)(from % width_), sy = (int)(from / width_);
    int gx = (int)(to   % width_), gy = (int)(to   / width_);
    double dx = gx - sx, dy = gy - sy;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len <= step) return to;
    int nx = sx + (int)std::round(step * dx / len);
    int ny = sy + (int)std::round(step * dy / len);
    nx = std::max(0, std::min((int)width_  - 1, nx));
    ny = std::max(0, std::min((int)height_ - 1, ny));
    return costmap_->getIndex((unsigned int)nx, (unsigned int)ny);
}

double RRTxPlanner::shrinkingBallRadius() const {
    unsigned int n = (unsigned int)nodes_.size();
    if (n < 2) return step_size_;
    double g = (gamma_ > 0.0) ? gamma_ : 2.0 * step_size_;
    return std::min(step_size_, g * std::sqrt(std::log((double)n) / (double)n));
}

// =============================================================================
//  Priority-queue helpers
// =============================================================================

QKey RRTxPlanner::makeKey(unsigned int v) const {
    const RRTxNode& nd = nodes_.at(v);
    return { std::min(nd.g, nd.lmc), nd.lmc, v };
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

// =============================================================================
//  Algorithm 6 – findParent(v, U)
// =============================================================================

void RRTxPlanner::findParent(unsigned int v,
                              const std::vector<unsigned int>& U) {
    RRTxNode& vn = node(v);
    double r = shrinkingBallRadius();

    for (unsigned int u : U) {
        if (u == v || !nodes_.count(u)) continue;
        if (hasObstacle(v, u)) continue;
        double d = edgeCost(v, u);
        const RRTxNode& un = nodes_.at(u);
        if (d <= r && vn.lmc > d + un.lmc) {
            vn.lmc       = d + un.lmc;
            vn.parent    = u;
            vn.hasParent = true;
        }
    }
}

// =============================================================================
//  Algorithm 2 – extend(v, r)
// =============================================================================

void RRTxPlanner::extend(unsigned int v, double r) {
    int vx = (int)(v % width_), vy = (int)(v / width_);

    // Collect nodes within radius r (brute-force; replace with range-search)
    std::vector<unsigned int> Vnear;
    Vnear.reserve(64);
    for (auto& [idx, nd] : nodes_) {
        if (idx == v) continue;
        if (dist(vx, vy, (int)(idx % width_), (int)(idx / width_)) <= r)
            Vnear.push_back(idx);
    }

    findParent(v, Vnear);

    RRTxNode& vn = node(v);
    if (!vn.hasParent) { nodes_.erase(v); return; }

    node(vn.parent).children.insert(v);

    // Build neighbour sets (Algorithm 2, lines 7-13)
    for (unsigned int u : Vnear) {
        RRTxNode& un = node(u);
        if (!hasObstacle(v, u)) { vn.Nr_plus.insert(u);  un.Nr_minus.insert(v); }
        if (!hasObstacle(u, v)) { un.Nr_plus.insert(v);  vn.Nr_minus.insert(u); }
    }
}

// =============================================================================
//  Algorithm 3 – cullNeighbors(v, r)
// =============================================================================

void RRTxPlanner::cullNeighbors(unsigned int v, double r) {
    RRTxNode& vn = node(v);
    std::vector<unsigned int> prune;

    for (unsigned int u : vn.Nr_plus) {
        bool isParent = vn.hasParent && (vn.parent == u);
        if (!isParent && edgeCost(v, u) > r) prune.push_back(u);
    }
    for (unsigned int u : prune) {
        vn.Nr_plus.erase(u);
        if (nodes_.count(u)) nodes_.at(u).Nr_minus.erase(v);
    }
}

// =============================================================================
//  Algorithm 4 – rewireNeighbors(v)
// =============================================================================

void RRTxPlanner::rewireNeighbors(unsigned int v) {
    RRTxNode& vn = node(v);
    double r = shrinkingBallRadius();

    if (vn.g - vn.lmc > epsilon_) cullNeighbors(v, r);

    // N-(v) \ {p+(v)}
    std::unordered_set<unsigned int> Nminus;
    for (unsigned int u : vn.Nr_minus) Nminus.insert(u);
    for (unsigned int u : vn.N0_minus) Nminus.insert(u);
    if (vn.hasParent) Nminus.erase(vn.parent);

    for (unsigned int u : Nminus) {
        if (!nodes_.count(u)) continue;
        RRTxNode& un = nodes_.at(u);
        double d = edgeCost(u, v);
        if (un.lmc > d + vn.lmc) {
            un.lmc = d + vn.lmc;
            if (un.hasParent) nodes_.at(un.parent).children.erase(u);
            un.parent    = v;
            un.hasParent = true;
            vn.children.insert(u);
            if (un.g - un.lmc > epsilon_) verifyQueue(u);
        }
    }
}

// =============================================================================
//  updateLMC – scan neighbours for cheapest parent
// =============================================================================

void RRTxPlanner::updateLMC(unsigned int v) {
    RRTxNode& vn = node(v);

    std::unordered_set<unsigned int> cands;
    for (unsigned int u : vn.Nr_plus) cands.insert(u);
    for (unsigned int u : vn.N0_plus) cands.insert(u);
    if (vn.hasParent) cands.erase(vn.parent);

    for (unsigned int u : cands) {
        if (!nodes_.count(u)) continue;
        if (hasObstacle(v, u)) continue;
        RRTxNode& un = nodes_.at(u);
        double d = edgeCost(v, u);
        if (d + un.lmc < vn.lmc) {
            if (vn.hasParent) nodes_.at(vn.parent).children.erase(v);
            vn.lmc       = d + un.lmc;
            vn.parent    = u;
            vn.hasParent = true;
            un.children.insert(v);
        }
    }
}

// =============================================================================
//  Algorithm 5 – reduceInconsistency()
// =============================================================================

void RRTxPlanner::reduceInconsistency() {
    if (!nodes_.count(start_index_)) return;

    auto topKey  = [&]() { return *queue_.begin(); };
    QKey botKey  = makeKey(start_index_);
    const RRTxNode* bot = &nodes_.at(start_index_);

    while (!queue_.empty() &&
           (keyLess(topKey(), botKey)               ||
            std::abs(bot->lmc - bot->g) > epsilon_  ||
            bot->g == INF                           ||
            inQueue_.count(start_index_)))
    {
        unsigned int v = queue_.begin()->index;
        queue_.erase(queue_.begin());
        inQueue_.erase(v);

        RRTxNode& vn = node(v);
        if (vn.g - vn.lmc > epsilon_) {
            updateLMC(v);
            rewireNeighbors(v);
        }
        vn.g = vn.lmc;

        bot    = &nodes_.at(start_index_);
        botKey = makeKey(start_index_);
    }
}

// =============================================================================
//  Costmap snapshot & change detection
//
//  In ROS a BaseGlobalPlanner plugin has no callback hook into the costmap
//  layer stack.  We therefore keep a full copy (costmap_snapshot_) of the
//  costmap char-map and diff it against the live map at the top of each
//  makePlan() call.  getCharMap() is O(1) – it returns a pointer to the
//  internal array – so the memcpy is the only cost.
// =============================================================================

bool RRTxPlanner::detectObstacleChanges(
        std::vector<unsigned int>& newly_blocked,
        std::vector<unsigned int>& newly_cleared)
{
    const uint8_t*   live = costmap_->getCharMap();
    const unsigned int N  = width_ * height_;

    newly_blocked.clear();
    newly_cleared.clear();

    if (costmap_snapshot_.empty()) {
        costmap_snapshot_.assign(live, live + N);
        return false;   // first call – no previous state to diff against
    }

    for (unsigned int i = 0; i < N; ++i) {
        bool was_obs = (costmap_snapshot_[i] >= costmap_2d::LETHAL_OBSTACLE);
        bool  is_obs = (live[i]              >= costmap_2d::LETHAL_OBSTACLE);
        if (!was_obs &&  is_obs) newly_blocked.push_back(i);
        if ( was_obs && !is_obs) newly_cleared.push_back(i);
    }

    std::memcpy(costmap_snapshot_.data(), live, N);
    return !newly_blocked.empty() || !newly_cleared.empty();
}

// =============================================================================
//  Edge-set helpers
// =============================================================================

void RRTxPlanner::removeEdgeFromSets(unsigned int u, unsigned int v) {
    if (nodes_.count(u)) {
        auto& un = nodes_.at(u);
        un.Nr_plus.erase(v); un.Nr_minus.erase(v);
        un.N0_plus.erase(v); un.N0_minus.erase(v);
    }
    if (nodes_.count(v)) {
        auto& vn = nodes_.at(v);
        vn.Nr_plus.erase(u); vn.Nr_minus.erase(u);
        vn.N0_plus.erase(u); vn.N0_minus.erase(u);
    }
}

void RRTxPlanner::restoreEdgeInSets(unsigned int u, unsigned int v) {
    if (!nodes_.count(u) || !nodes_.count(v)) return;
    if (hasObstacle(u, v)) return;   // still blocked by another cell

    double d = edgeCost(u, v);
    double r = shrinkingBallRadius();
    auto& un = nodes_.at(u);
    auto& vn = nodes_.at(v);

    if (d <= r) {
        if (!hasObstacle(u, v)) { un.Nr_plus.insert(v); vn.Nr_minus.insert(u); }
        if (!hasObstacle(v, u)) { vn.Nr_plus.insert(u); un.Nr_minus.insert(v); }
    } else {
        if (!hasObstacle(u, v)) { un.N0_plus.insert(v); vn.N0_minus.insert(u); }
        if (!hasObstacle(v, u)) { vn.N0_plus.insert(u); un.N0_minus.insert(v); }
    }
}

// =============================================================================
//  propagateOrphan(root)
//
//  Iterative BFS disconnects root and every tree descendant from the tree.
//  Each disconnected node:
//    • loses its parent pointer
//    • gets lmc = g = INF
//    • is removed from the priority queue (stale key)
//    • is added to orphan_set_
//    • is re-inserted into the queue with its new INF key so that
//      reduceInconsistency will call updateLMC on it and try re-routing
//
//  Neighbour sets are intentionally LEFT INTACT so that reintegrateOrphans
//  can find surviving paths through them.
// =============================================================================

void RRTxPlanner::propagateOrphan(unsigned int root) {
    std::queue<unsigned int> bfs;
    bfs.push(root);

    while (!bfs.empty()) {
        unsigned int v = bfs.front(); bfs.pop();
        if (!nodes_.count(v)) continue;

        RRTxNode& vn = nodes_.at(v);

        // Disconnect from parent
        if (vn.hasParent && nodes_.count(vn.parent))
            nodes_.at(vn.parent).children.erase(v);
        vn.hasParent = false;

        // Mark cost as unknown
        vn.lmc = INF;
        vn.g   = INF;

        removeFromQueue(v);     // remove stale key
        orphan_set_.insert(v);
        verifyQueue(v);         // re-insert with new (INF, INF) key

        for (unsigned int c : vn.children) bfs.push(c);
        vn.children.clear();
    }
}

// =============================================================================
//  reintegrateOrphans()
//
//  Repeatedly scans orphan_set_, trying to re-parent each orphan via its
//  surviving neighbour sets.  Iterates until no further progress is made
//  (handles chains A→B→C where B must be re-parented before A can be).
// =============================================================================

void RRTxPlanner::reintegrateOrphans() {
    bool progress = true;

    while (progress) {
        progress = false;
        std::vector<unsigned int> remain;
        remain.reserve(orphan_set_.size());

        for (unsigned int v : orphan_set_) {
            if (!nodes_.count(v)) continue;
            RRTxNode& vn = nodes_.at(v);

            double       best_cost   = INF;
            unsigned int best_parent = 0;
            bool         found       = false;

            // Scan all sets that hold potential parents
            auto tryParent = [&](unsigned int u) {
                if (!nodes_.count(u))    return;
                if (orphan_set_.count(u)) return;   // also orphaned
                const RRTxNode& un = nodes_.at(u);
                if (un.lmc >= INF)       return;
                if (hasObstacle(v, u))   return;
                double d = edgeCost(v, u);
                if (d + un.lmc < best_cost) {
                    best_cost   = d + un.lmc;
                    best_parent = u;
                    found       = true;
                }
            };

            for (unsigned int u : vn.Nr_plus) tryParent(u);
            for (unsigned int u : vn.N0_plus) tryParent(u);

            if (found) {
                // Re-parent
                vn.lmc       = best_cost;
                vn.parent    = best_parent;
                vn.hasParent = true;
                nodes_.at(best_parent).children.insert(v);

                // g stays INF → inconsistent → reduceInconsistency will fix it
                verifyQueue(v);
                progress = true;
            } else {
                remain.push_back(v);
            }
        }

        orphan_set_ = { remain.begin(), remain.end() };
    }
    // Nodes still in orphan_set_ have no surviving path; they stay in the
    // tree (as disconnected nodes) and may be re-parented by future samples.
}

// =============================================================================
//  Algorithm 1, lines 5-6:  updateObstacles()
//
//  How obstacle detection works in a ROS global-planner plugin
//  -----------------------------------------------------------
//  A BaseGlobalPlanner has no direct callback into the costmap layer stack.
//  The recommended pattern is:
//
//    1.  Store a snapshot of costmap_->getCharMap() at the end of makePlan().
//    2.  At the START of the next makePlan() call, diff the live char-map
//        against the snapshot to get newly_blocked / newly_cleared cell lists.
//    3.  Call updateObstacles() with those lists.
//
//  This is safe because makePlan() is called from the move_base thread that
//  also locks the costmap, so the char-map pointer remains valid for the
//  duration of the call.
//
//  Newly-blocked cells
//  -------------------
//  For each blocked cell C we walk every tree node v and:
//    (a) If the tree-edge (parent(v) → v) passes through C:
//          propagateOrphan(v)  – disconnects v's whole subtree
//    (b) For each neighbour u of v whose segment (v,u) passes through C:
//          removeEdgeFromSets(v, u)  – edge is no longer a valid connection
//
//  Newly-cleared cells
//  -------------------
//  For each cleared cell we look for node-pairs within step_size_ that are
//  NOT already connected and whose segment is now fully free, then restore
//  the edge and mark the nodes inconsistent so reduceInconsistency can
//  propagate any cost improvements.
// =============================================================================

void RRTxPlanner::updateObstacles(
        const std::vector<unsigned int>& newly_blocked,
        const std::vector<unsigned int>& newly_cleared)
{
    // ── 1. Handle newly blocked cells ────────────────────────────────────────
    for (unsigned int cell : newly_blocked) {
        // Snapshot node indices to avoid iterator invalidation
        std::vector<unsigned int> snap;
        snap.reserve(nodes_.size());
        for (auto& [idx, _] : nodes_) snap.push_back(idx);

        for (unsigned int v : snap) {
            if (!nodes_.count(v)) continue;
            RRTxNode& vn = nodes_.at(v);

            // (a) Check the tree-parent edge
            if (vn.hasParent && nodes_.count(vn.parent)) {
                if (edgePassesThrough(vn.parent, v, cell)) {
                    propagateOrphan(v);
                    continue;
                }
            }

            // (b) Remove any neighbour edges that cross this cell
            std::vector<unsigned int> to_remove;
            auto scan = [&](const std::unordered_set<unsigned int>& S) {
                for (unsigned int u : S)
                    if (edgePassesThrough(v, u, cell))
                        to_remove.push_back(u);
            };
            scan(vn.Nr_plus); scan(vn.Nr_minus);
            scan(vn.N0_plus); scan(vn.N0_minus);

            for (unsigned int u : to_remove) removeEdgeFromSets(v, u);

            // If the stored parent is no longer reachable, orphan this node
            if (vn.hasParent && hasObstacle(v, vn.parent)) {
                nodes_.at(vn.parent).children.erase(v);
                vn.hasParent = false;
                vn.lmc = INF;
                vn.g   = INF;
                orphan_set_.insert(v);
                removeFromQueue(v);
                verifyQueue(v);
            }
        }
    }

    // ── 2. Handle newly cleared cells ────────────────────────────────────────
    if (!newly_cleared.empty()) {
        std::vector<unsigned int> all;
        all.reserve(nodes_.size());
        for (auto& [idx, _] : nodes_) all.push_back(idx);

        for (size_t i = 0; i < all.size(); ++i) {
            unsigned int u = all[i];
            if (!nodes_.count(u)) continue;
            int ux = (int)(u % width_), uy = (int)(u / width_);

            for (size_t j = i + 1; j < all.size(); ++j) {
                unsigned int v = all[j];
                if (!nodes_.count(v)) continue;

                if (dist(ux, uy, (int)(v % width_), (int)(v / width_)) > step_size_)
                    continue;

                // Only try to restore edges that are NOT already present
                const auto& un = nodes_.at(u);
                bool already = un.Nr_plus.count(v) || un.Nr_minus.count(v) ||
                               un.N0_plus.count(v) || un.N0_minus.count(v);
                if (already) continue;

                restoreEdgeInSets(u, v);

                // If an edge was restored, mark both nodes for re-processing
                if (nodes_.at(u).Nr_plus.count(v) || nodes_.at(u).N0_plus.count(v)) {
                    verifyQueue(u);
                    verifyQueue(v);
                }
            }
        }
    }

    // ── 3. Re-integrate orphans then let the main loop call reduceInconsistency
    reintegrateOrphans();

    ROS_INFO("[RRTx] updateObstacles done: %zu blocked cells, %zu cleared cells, "
             "%zu orphans remain", newly_blocked.size(), newly_cleared.size(),
             orphan_set_.size());
}

// =============================================================================
//  nav_core interface – initialize()
// =============================================================================

void RRTxPlanner::initialize(std::string name,
                              costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) { ROS_WARN("[RRTx] Already initialized"); return; }
    costmap_ros_ = costmap_ros;
    costmap_     = costmap_ros->getCostmap();
    width_       = costmap_->getSizeInCellsX();
    height_      = costmap_->getSizeInCellsY();
    initialized_ = true;
    ROS_INFO("[RRTx] Initialized: map %u x %u", width_, height_);
}

// =============================================================================
//  nav_core interface – makePlan()
//
//  Call flow
//  ---------
//   1. Map start/goal to cell indices.
//   2. If goal changed → full reset.
//   3. Update robot position (v_bot = start_index_).
//   4. Diff costmap snapshot → updateObstacles() + reduceInconsistency().
//   5. Continue RRTx sampling loop until robot node has finite lmc.
//   6. Reconstruct path via parent pointers.
//   7. Take costmap snapshot for the next call.
// =============================================================================

bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("[RRTx] makePlan called");
    ros::Time t0 = ros::Time::now();

    // ── 1. Map coordinates ───────────────────────────────────────────────────
    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !costmap_->worldToMap(goal.pose.position.x,  goal.pose.position.y,  gx, gy)) {
        ROS_ERROR("[RRTx] Start or goal outside map"); return false;
    }
    unsigned int new_start = costmap_->getIndex(sx, sy);
    unsigned int new_goal  = costmap_->getIndex(gx, gy);

    // ── 2. Reset if goal changed ─────────────────────────────────────────────
    bool goal_changed = (std::abs(goal.pose.position.x - cached_goal_x_) > 1e-3 ||
                         std::abs(goal.pose.position.y - cached_goal_y_) > 1e-3);

    if (goal_changed || !tree_initialized_) {
        ROS_INFO("[RRTx] %s – resetting tree",
                 goal_changed ? "Goal changed" : "First run");
        nodes_.clear(); queue_.clear(); inQueue_.clear();
        orphan_set_.clear(); costmap_snapshot_.clear();
        tree_ = kdTree(); tree_initialized_ = false;

        cached_goal_x_ = goal.pose.position.x;
        cached_goal_y_ = goal.pose.position.y;
        goal_index_    = new_goal;

        RRTxNode& gn = node(goal_index_);
        gn.lmc = 0.0; gn.g = 0.0;
        tree_.insert({ (int)gx, (int)gy });
    }

    // ── 3. Update robot position ─────────────────────────────────────────────
    start_index_ = new_start;
    if (!nodes_.count(start_index_)) node(start_index_);

    // ── 4. Obstacle change detection and tree repair ─────────────────────────
    std::vector<unsigned int> newly_blocked, newly_cleared;
    if (detectObstacleChanges(newly_blocked, newly_cleared)) {
        ROS_INFO("[RRTx] Costmap changed: %zu blocked, %zu cleared",
                 newly_blocked.size(), newly_cleared.size());
        updateObstacles(newly_blocked, newly_cleared);
        reduceInconsistency();
    }

    // ── 5. Sampling loop ─────────────────────────────────────────────────────
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> udist(0, width_ * height_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    bool path_found = (nodes_.count(start_index_) &&
                       nodes_.at(start_index_).lmc < INF);

    for (unsigned int iter = 0; iter < max_iters_ && !path_found; ++iter) {
        double r = shrinkingBallRadius();

        unsigned int rand_idx = (rand01(rng) < goal_bias_) ? start_index_ : udist(rng);

        point rp = { (int)(rand_idx % width_), (int)(rand_idx / width_) };
        point np = tree_.nearest(rp);
        unsigned int near_idx = costmap_->getIndex((unsigned int)np.first,
                                                    (unsigned int)np.second);

        unsigned int new_idx = steer(near_idx, rand_idx, step_size_);
        if (nodes_.count(new_idx)) continue;
        if (costmap_->getCost(new_idx % width_, new_idx / width_) >=
            costmap_2d::LETHAL_OBSTACLE) continue;

        node(new_idx);
        extend(new_idx, r);
        if (!nodes_.count(new_idx)) continue;

        tree_.insert({ (int)(new_idx % width_), (int)(new_idx / width_) });
        rewireNeighbors(new_idx);
        reduceInconsistency();

        if (nodes_.count(start_index_) && nodes_.at(start_index_).lmc < INF)
            path_found = true;
    }

    tree_initialized_ = true;

    // ── 7. Snapshot for next call ────────────────────────────────────────────
    {
        const uint8_t* live = costmap_->getCharMap();
        unsigned int N = width_ * height_;
        if (costmap_snapshot_.size() != N) costmap_snapshot_.resize(N);
        std::memcpy(costmap_snapshot_.data(), live, N);
    }

    if (!path_found) { ROS_WARN("[RRTx] No path found"); return false; }

    // ── 6. Path reconstruction ───────────────────────────────────────────────
    plan.clear();
    unsigned int curr = start_index_, safety = 0;

    while (curr != goal_index_ && safety++ < width_ * height_) {
        double wx, wy;
        costmap_->mapToWorld(curr % width_, curr / width_, wx, wy);
        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx; p.pose.position.y = wy;
        plan.push_back(p);

        if (!nodes_.count(curr) || !nodes_.at(curr).hasParent) {
            ROS_WARN("[RRTx] Broken tree at idx %u", curr); return false;
        }
        curr = nodes_.at(curr).parent;
    }
    {
        double wx, wy;
        costmap_->mapToWorld(gx, gy, wx, wy);
        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx; p.pose.position.y = wy;
        plan.push_back(p);
    }

    ROS_INFO("[RRTx] Path: %zu poses, %.4fs, %zu nodes in tree",
             plan.size(), (ros::Time::now() - t0).toSec(), nodes_.size());
    return true;
}

} // namespace custom_planner