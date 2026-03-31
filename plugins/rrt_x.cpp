#include <custom_nav/rrt_x.h>
#include <custom_nav/kd_tree_x.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>
#include <algorithm>

#define INF 1e9

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTXPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

// Function to initialize and store the costmap info once.
void RRTXPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_m_ = costmap_->getSizeInMetersY();
        width_m_ = costmap_->getSizeInMetersX();
        height_c_ = costmap_->getSizeInCellsY();
        width_c_ = costmap_->getSizeInCellsX();
        origin_x = costmap_->getOriginX();
        origin_y = costmap_->getOriginY();
        resolution_ = costmap_->getResolution();

        ROS_INFO("Initialized RRTX planner with map of size %f * %f", height_m_, width_m_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

// Main function that plans the trajectory
bool RRTXPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    if (!initialized_) {
        ROS_ERROR("RRTXPlanner has not been initialized");
        return false;
    }

    ROS_INFO("Making path using RRTX");

    ros::Time start_time = ros::Time::now();

    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    
    ROS_INFO("Start position: (%f, %f), Goal position: (%f, %f)", sx, sy, gx, gy);
    
    // Check if the goal has changed and we need a new tree
    if (isNewGoal(gx, gy)) {
        resetTree();
        
        Node root_node(goal.pose.position.x, goal.pose.position.y);
        root_node.g = 0.0;     // Goal cost is 0
        root_node.lmc = 0.0;
        nodes_.push_back(root_node);
        kd_tree.insert(Point<double, 2>({root_node.x, root_node.y}), 0);
        // Add to kd tree
    }
    
    // Modify the global variables
    start_ = Point<double, 2>({sx, sy});
    goal_ = Point<double, 2>({gx, gy});

    // Check if obstacles have changed and update the queues
    if (nodes_.size() > 1) {
        start_proxy = findStartProxy();
        updateObstacles();
    }

    // Grow or refine the tree
    // Sample, find nearest, grow tree
    int iters = 0;
    while (!isConnected(sx, sy) && iters < max_iters_) {
        double x = origin_x + (rand01(rng) * width_m_);
        double y = origin_y + (rand01(rng) * height_m_);

        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my)) continue;
        if (costmap_->getCost(mx, my) >= 254) continue;

        Point<double, 2> random_pt({x, y});

        // Get the point nearest to the ranomly selected point
        std::size_t nearest_index = kd_tree.nearest(random_pt);
        Node& near_node = nodes_[nearest_index];
        Point<double, 2> near_pt({near_node.x, near_node.y});

        // Move in the direction of the random point from the near point
        Point<double, 2> new_pt = steer(near_pt[0], near_pt[1], random_pt[0], random_pt[1]);

        if (!costmap_->worldToMap(new_pt[0], new_pt[1], mx, my)) continue;
        if (costmap_->getCost(mx, my) >= 254) continue;
        
        // Check if the path is free of obstacles
        if (hasObstacle(near_pt, new_pt)) continue;

        // If not an obstacle, then add the point to the tree
        Node new_node(new_pt[0], new_pt[1]);
        std::size_t new_idx = nodes_.size(); // It will be inserted at this index


        // Find neighbors within radius
        std::vector<std::size_t> neighbors = kd_tree.radius_search(new_pt, radius_);

        // Find best parent
        double min_cost = near_node.g + new_pt.distance(near_pt);
        std::size_t best_parent = nearest_index;

        // In this loop, we iterate through all the neighbours
        // and populate the neighbours lists, and find the best parent
        for (std::size_t nbr_idx : neighbors) {
            Node& nbr = nodes_[nbr_idx];
            Point<double, 2> nbr_pt({nbr.x, nbr.y});
            
            // Check edge validity
            if (!hasObstacle(nbr_pt, new_pt)) {
                // Populate new node's neighbor lists
                new_node.nbr_init.push_back(nbr_idx);
                new_node.nbr_running.push_back(nbr_idx);
                
                nbr.nbr_init.push_back(new_idx);
                nbr.nbr_running.push_back(new_idx);

                double cost = nbr.g + new_pt.distance(nbr_pt);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_parent = nbr_idx;
                }
            }
        }

        new_node.par_idx = best_parent;
        new_node.lmc = min_cost;
        new_node.g = min_cost; // It is initially consistent
        
        // Add to tree
        // Update the tree insert function to support adding indices
        nodes_.push_back(new_node);
        kd_tree.insert(new_pt, new_idx);
        nodes_[best_parent].children.push_back(new_idx);

        radius_ = getRadius();

        rewireNeighbors(new_idx);
        reduceInconsistency();

        iters++;
    }

    if (!isConnected(sx, sy)) {
        ROS_WARN("RRTXPlanner: Failed to find a path to the goal within max iterations.");
        return false;
    }

    // Clear any old path data
    plan.clear();

    // Find the node closest to our current start position to begin tracing
    std::size_t current_idx = findStartProxy();
    
    if (current_idx == Node::INVALID_IDX) {
        ROS_ERROR("RRTXPlanner: Start proxy is invalid despite tree being connected.");
        return false;
    }

    // Add the exact starting pose so the local planner has a smooth beginning
    plan.push_back(start);

    // Trace the tree from the start proxy up to the goal root
    std::size_t loop_safety_counter = 0;
    
    while (current_idx != Node::INVALID_IDX) {
        const Node& curr_node = nodes_[current_idx];

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = start_time;
        pose.header.frame_id = start.header.frame_id; // Keep consistent with move_base frames
        pose.pose.position.x = curr_node.x;
        pose.pose.position.y = curr_node.y;
        pose.pose.position.z = 0.0;
        
        pose.pose.orientation.w = 1.0; 
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        plan.push_back(pose);

        // Break if we reached the goal node (where cost is 0)
        if (curr_node.g == 0.0) {
            break;
        }

        // Move to the parent node
        current_idx = curr_node.par_idx;

        // For safety
        loop_safety_counter++;
        if (loop_safety_counter > nodes_.size()) {
            ROS_ERROR("RRTXPlanner: Infinite loop detected in tree topology during path extraction!");
            plan.clear();
            return false;
        }
    }

    // Add the exact goal pose for precise final positioning
    plan.push_back(goal);

    planned_ = true;
    ROS_INFO("RRTXPlanner: Successfully extracted path with %zu waypoints.", plan.size());
    
    return true;
}

void RRTXPlanner::reduceInconsistency() {
    // Anonymous function that returns the minimum in the set
    auto topKey = [&]() { return *queue_.begin(); };

    // Make key and extract the closest node to the current bot position
    auto botKey = makeKey(start_proxy);
    Node& bot = nodes_[start_proxy];

    // Stop when the robot's proxy is consistent 
    // AND all cheaper nodes have been processed.
    while (!queue_.empty() &&
           (keyLess(topKey(), botKey)               ||
            std::abs(bot.lmc - bot.g) > epsilon_    ||
            bot.g == std::numeric_limits<double>::infinity() ||
            bot.in_queue)) 
    {
        // Remove the minimum from the queue
        auto top = queue_.begin();
        std::size_t v_idx = top->index;
        queue_.erase(top);

        Node& v = nodes_[v_idx];
        v.in_queue = false;

        if (v.g - v.lmc > epsilon_) {
            updateLMC(v_idx);
            rewireNeighbors(v_idx);
        }

        v.g = v.lmc;
        
        // Update botKey just in case the proxy's cost changed during the loop
        botKey = makeKey(start_proxy); 
    }
}

void RRTXPlanner::rewireNeighbors(std::size_t v_index) {
    Node& v = nodes_[v_index];

    if (v.g - v.lmc <= epsilon_) return;

    cullNeighbors(v_index);

    // Check all the neighbors
    for (std::size_t nbr_index: v.nbr_init) {
        if (nbr_index == v.par_idx) continue;

        Node& nbr = nodes_[nbr_index];
        double nbr_dist_via_v = v.lmc + distance(v_index, nbr_index);

        // If going through v is better
        if (nbr_dist_via_v < nbr.lmc) {
            // Remove from parent's children list
            if (nbr.par_idx != Node::INVALID_IDX) {
                Node& old_parent = nodes_[nbr.par_idx];
                old_parent.children.erase(
                    std::remove(old_parent.children.begin(), old_parent.children.end(), nbr_index),
                    old_parent.children.end()
                );
            }

            // Link to new parent
            nbr.par_idx = v_index;
            v.children.push_back(nbr_index);

            // Update lmc
            nbr.lmc = nbr_dist_via_v;

            if (nbr.g - nbr.lmc > epsilon_) verifyQueue(nbr_index);
        }
    }
}

// TODO: Modify the implementations so that the neighbor lists are sorted
// when created, and we only check from back when removing elements
// Can be made more optimised
void RRTXPlanner::cullNeighbors(std::size_t node_idx) {
    Node& node = nodes_[node_idx];
    
    // We use remove_if to cleanly filter the vector
    node.nbr_running.erase(
        std::remove_if(node.nbr_running.begin(), node.nbr_running.end(),
            [&](std::size_t nbr_idx) {
                Node& nbr = nodes_[nbr_idx];

                // If the neighbor is now outside the shrinking radius
                if (distance(node_idx, nbr_idx) > radius_) {
                    // Remove 'node_idx' from the neighbor's running list as well
                    nbr.nbr_running.erase(
                        std::remove(nbr.nbr_running.begin(), nbr.nbr_running.end(), node_idx),
                        nbr.nbr_running.end()
                    );
                    return true; // Tells remove_if to erase this nbr_idx from node's list
                }
                return false; // Keep it
            }
        ),
        node.nbr_running.end()
    );
}

void RRTXPlanner::updateLMC(std::size_t v_idx) {
    cullNeighbors(v_idx); 
    
    Node& v = nodes_[v_idx];
    
    double min_cost = std::numeric_limits<double>::infinity();
    std::size_t best_parent = Node::INVALID_IDX;

    Point<double, 2> v_pt({v.x, v.y});

    // Line 2: forall the u in N+(v)
    for (std::size_t u_idx : v.nbr_running) {
        Node& u = nodes_[u_idx];
        
        // Line 2 (Constraint): p(u) != v 
        // Prevent infinite routing loops by ensuring we don't pick our own child
        if (u_idx == v_idx || u.par_idx == v_idx) continue;

        Point<double, 2> u_pt({u.x, u.y});

        // Implicitly part of d_pi(v, u): Check if edge is valid
        if (!hasObstacle(v_pt, u_pt)) {
            
            // Line 3: d_pi(v, u) + lmc(u)
            // CRITICAL FIX: Using u.lmc instead of u.g
            double cost = v_pt.distance(u_pt) + u.lmc;

            // Lines 3 & 4 logic: Track the minimum cost and best parent (p')
            if (cost < min_cost) {
                min_cost = cost;
                best_parent = u_idx;
            }
        }
    }

    // Line 5: makeParentOf(p', v)
    // Inline implementation of breaking old ties and making the new connection
    if (v.par_idx != best_parent) {
        
        if (v.par_idx != Node::INVALID_IDX) {
            Node& old_parent = nodes_[v.par_idx];
            old_parent.children.erase(
                std::remove(old_parent.children.begin(), old_parent.children.end(), v_idx),
                old_parent.children.end()
            );
        }

        if (best_parent != Node::INVALID_IDX) {
            nodes_[best_parent].children.push_back(v_idx);
        }

        v.par_idx = best_parent;
    }

    // Update the Lookahead cost explicitly (done inside makeParentOf in the paper)
    v.lmc = min_cost;
}

void RRTXPlanner::updateObstacles() {
    // Make two vectors to store the cells that have changed
    std::vector<unsigned int> newly_blocked;
    std::vector<unsigned int> newly_cleared;

    // Get the new costmap data
    const uint8_t* live = costmap_->getCharMap();
    const unsigned int N = width_c_ * height_c_;

    // First invocation of the function
    if (costmap_snapshot_.empty()) {
        costmap_snapshot_.assign(live, live + N);
        return;
    }

    // Diff with the old snapshot
    for (unsigned int i = 0; i < N; i++) {
        bool was_obs = (costmap_snapshot_[i] >= 254);
        bool is_obs = (live[i] >= 254);

        if (!was_obs && is_obs) newly_blocked.push_back(i);
        if (was_obs && !is_obs) newly_cleared.push_back(i);
    }

    // Store the new data
    std::memcpy(costmap_snapshot_.data(), live, N);

    // Handle removed obstacles
    if (!newly_cleared.empty()) {
        for (unsigned int i: newly_cleared) removeObstacle(i);
        reduceInconsistency();
    }

    // Handle added obstacles
    if (!newly_blocked.empty()) {
        for (unsigned int i: newly_blocked) addObstacle(i);
        propogateDescendents();
        verifyQueue(start_proxy);
        reduceInconsistency();
    }
}

void RRTXPlanner::removeObstacle(unsigned int i) {
    // Remove from global tracking
    auto it = std::find(obstacles_.begin(), obstacles_.end(), i);
    if (it != obstacles_.end()) {
        *it = obstacles_.back();
        obstacles_.pop_back();
    }

    // Map back to world coordinates
    unsigned int obstacle_x = i % width_c_;
    unsigned int obstacle_y = i / width_c_;
    double center_x = origin_x + (obstacle_x + 0.5) * resolution_;
    double center_y = origin_y + (obstacle_y + 0.5) * resolution_;

    // KD Tree query
    std::vector<std::size_t> affected_nodes = kd_tree.radius_search(
        Point<double, 2>({center_x, center_y}), step_length_ + resolution_);

    // Restoration cascade
    for (std::size_t v_idx : affected_nodes) {
        Node& v = nodes_[v_idx];

        // Copy set to allow safe erasure during iteration
        std::vector<std::size_t> neighbors_to_check(v.blocked_nbrs.begin(), v.blocked_nbrs.end());

        for (std::size_t u_idx : neighbors_to_check) {
            if (v_idx > u_idx) continue;

            Node& u = nodes_[u_idx];

            // Verify the edge is clear against ALL remaining obstacles
            // Note: Replace hasObstacle with your global collision function if named differently
            if (!hasObstacle(Point<double, 2>({v.x, v.y}), Point<double, 2>({u.x, u.y}))) {
                v.blocked_nbrs.erase(u_idx);
                u.blocked_nbrs.erase(v_idx);

                updateLMC(v_idx);
                updateLMC(u_idx);

                if (std::abs(v.lmc - v.g) > epsilon_) {
                    verifyQueue(v_idx);
                }
                if (std::abs(u.lmc - u.g) > epsilon_) {
                    verifyQueue(u_idx);
                }
            }
        }
    }
}

void RRTXPlanner::addObstacle(unsigned int i) {
    obstacles_.push_back(i);

    unsigned int obstacle_x = i % width_c_;
    unsigned int obstacle_y = i / width_c_;

    double xmin = origin_x + obstacle_x * resolution_;
    double ymin = origin_y + obstacle_y * resolution_;
    double xmax = xmin + resolution_;
    double ymax = ymin + resolution_;

    // Make the set of edges that intersect this obstacle
    // Get the points that are within step_length_ of the obstacle
    std::vector<std::size_t> possible_invalidated_points = kd_tree.radius_search(Point<double, 2>({xmin, ymin}), step_length_ + resolution_);
    for (std::size_t v_idx: possible_invalidated_points) {
        Node& v = nodes_[v_idx];

        for (std::size_t u_idx: v.nbr_running) {
            if (v.blocked_nbrs.count(u_idx) > 0) continue;

            Node& u = nodes_[u_idx];
            if (isEdgeInCollision(u.x, u.y, v.x, v.y, xmin, ymin, xmax, ymax)) {
                v.blocked_nbrs.insert(u_idx);
                u.blocked_nbrs.insert(v_idx);

                if (v.par_idx == u_idx) verifyOrphan(v_idx);
                else if (u.par_idx == v_idx) verifyOrphan(u_idx);

                // Can also add condition to stop the robot if the obstacle is in current path
            }
        }
    }
}

Point<double, 2> RRTXPlanner::steer(double near_x, double near_y, double rand_x, double rand_y) {
    double new_x, new_y;

    double dx = rand_x - near_x;
    double dy = rand_y - near_y;
    double len = distance(near_x, near_y, rand_x, rand_y);

    if (len <= step_length_) return Point<double, 2>({rand_x, rand_y});

    new_x = near_x + (step_length_ * dx / len);
    new_y = near_y + (step_length_ * dy / len);

    return Point<double, 2>({new_x, new_y});
}

// Checks if the goal is changed
bool RRTXPlanner::isNewGoal(double gx, double gy) {
    return (gx != goal_[0]) || (gy != goal_[1]);
}

/**
 * Resets the tree when we encounter a new goal
 */
void RRTXPlanner::resetTree() {
    nodes_.clear();
    kd_tree.clear();
    orphan_set_.clear();
    queue_.clear();
    queueMap_.clear();
}

/**
 * Checks if point `p` is connected to the tree
 * Searches for points within `step_length_` of the start and chooses the best possible option
 */
bool RRTXPlanner::isConnected(double sx, double sy) {
    if (nodes_.empty()) return false;

    Point<double, 2> start_pt({sx, sy});

    std::vector<std::size_t> neighbors = kd_tree.radius_search(start_pt, step_length_);

    std::size_t best_index = Node::INVALID_IDX;
    double min_distance = std::numeric_limits<double>::infinity();

    for (std::size_t idx: neighbors) {
        Node n = nodes_[idx];

        if (n.g >= std::numeric_limits<double>::infinity()) continue;

        Point<double, 2> n_pt({nodes_[idx].x, nodes_[idx].y});
        double distance_to_goal = n.g + n_pt.distanceSquared(start_pt);
        if (distance_to_goal <= min_distance) {
            min_distance = distance_to_goal;
            best_index = idx;
        }
    }

    if (best_index != Node::INVALID_IDX) return true;

    return false;
}

bool RRTXPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_c_, sy = start / width_c_, gx = end % width_c_, gy = end / width_c_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) return true;
    }
    return false;
}

bool RRTXPlanner::hasObstacle(Point<double, 2> p1, Point<double, 2> p2) {
    unsigned int start, end;
    unsigned int mx, my;

    if (!costmap_->worldToMap(p1[0], p1[1], mx, my)) return true;
    start = costmap_->getIndex(mx, my);

    if (!costmap_->worldToMap(p2[0], p2[1], mx, my)) return true;
    end = costmap_->getIndex(mx, my);

    return hasObstacle(start, end);
}

// NOTE: Make this function more modular and
// remove the hardcoded 0.5 and add 1/dim logic
double RRTXPlanner::getRadius() const {
    double n = static_cast<double>(nodes_.size());
    if (n <= 1) return step_length_;

    return std::min(rad_const_ * std::pow(std::log(n) / n, 0.5), step_length_);
}

/**
 * We don't usually have start node in our tree
 * So we use the next nearest unblocked node as a proxy for the start node
 */
std::size_t RRTXPlanner::findStartProxy() {
    // Get the nearest node
    std::size_t proxy_idx = kd_tree.nearest(start_);
    
    // If the tree is empty or invalid, return the invalid index
    if (proxy_idx == Node::INVALID_IDX || proxy_idx >= nodes_.size()) {
        return Node::INVALID_IDX;
    }

    Node& nearest_node = nodes_[proxy_idx];
    Point<double, 2> proxy_pt({nearest_node.x, nearest_node.y});

    // If there's no obstacle between the robot and this node
    if (!hasObstacle(start_, proxy_pt)) {
        return proxy_idx;
    }

    // We must search a local radius for the closest node that we can actually see
    // We can change search radius to a multiple of step length
    double search_radius = step_length_;
    std::vector<std::size_t> local_neighbors = kd_tree.radius_search(start_, search_radius);

    double min_dist = std::numeric_limits<double>::infinity();
    std::size_t best_visible_proxy = proxy_idx;

    for (std::size_t nbr_idx : local_neighbors) {
        Node& nbr = nodes_[nbr_idx];
        Point<double, 2> nbr_pt({nbr.x, nbr.y});

        // Check if we have a clear path to this neighbor
        if (!hasObstacle(start_, nbr_pt)) {
            double dist = start_.distance(nbr_pt);
            if (dist < min_dist) {
                min_dist = dist;
                best_visible_proxy = nbr_idx;
            }
        }
    }

    return best_visible_proxy;
}

void RRTXPlanner::propogateDescendents() {
    std::vector<std::size_t> processing_queue(orphan_set_.begin(), orphan_set_.end());
    std::size_t head = 0;
    
    // Do a BFS and add all the children
    while (head < processing_queue.size()) {
        std::size_t v_idx = processing_queue[head++];
        for (std::size_t child_idx : nodes_[v_idx].children) {
            // std::unordered_set::insert returns a pair. .second is true if insertion took place
            if (orphan_set_.insert(child_idx).second) processing_queue.push_back(child_idx);
        }
    }

    // Invalidate neighboring nodes that rely on these orphans
    for (std::size_t v_idx : orphan_set_) {
        const Node& v = nodes_[v_idx];
        
        auto invalidate_neighbor = [&](std::size_t u_idx) {
            if (u_idx != Node::INVALID_IDX && orphan_set_.count(u_idx) == 0) {
                nodes_[u_idx].g = std::numeric_limits<double>::infinity();
                verifyQueue(u_idx);
            }
        };

        for (std::size_t u_idx : v.nbr_running) {
            invalidate_neighbor(u_idx);
        }
        invalidate_neighbor(v.par_idx);
    }

    // Sever all tree connections of the nodes in orphan set
    for (std::size_t v_idx: orphan_set_) {
        Node& v = nodes_[v_idx];
        v.g = std::numeric_limits<double>::infinity();
        v.lmc = std::numeric_limits<double>::infinity();

        if (v.par_idx != Node::INVALID_IDX) {
            Node& parent = nodes_[v.par_idx];
            auto it = std::find(parent.children.begin(), parent.children.end(), v_idx);
            if (it != parent.children.end()) {
                *it = parent.children.back();
                parent.children.pop_back();
            }
            v.par_idx = Node::INVALID_IDX;
        }
    }

    orphan_set_.clear();
}

void RRTXPlanner::verifyOrphan(std::size_t node_idx) {
    // Check if the node is in the queue or not
    auto it = queueMap_.find(node_idx);
    if (it != queueMap_.end()) {
        queue_.erase(it->second);               // Remove from the queue
        nodes_[node_idx].in_queue = false;
        queueMap_.erase(it);                    // Remove from map
    }
    orphan_set_.insert(node_idx);
}

void RRTXPlanner::verifyQueue(std::size_t node_idx) {
    QKey key = makeKey(node_idx);
    auto it = queueMap_.find(node_idx);
    if (it == queueMap_.end()) {
        queueMap_.insert({node_idx, key});
        queue_.insert(key);
        nodes_[node_idx].in_queue = true;
    } else {
        queue_.erase(it->second);
        queue_.insert(key);
        queueMap_[node_idx] = key;
    }
}

QKey RRTXPlanner::makeKey(std::size_t v_idx) {
    Node& v = nodes_[v_idx];
    
    QKey key;
    key.k1 = std::min(v.g, v.lmc);
    key.k2 = v.g;
    key.index = v_idx;
    
    return key;
}

/**
 * Returns true if a is less than b
 * We don't just return a < b as that compares index if both keys are equal
 */
bool RRTXPlanner::keyLess(const QKey& a, const QKey& b) const {
    if (a.k1 != b.k1) return a.k1 < b.k1;
    return a.k2 < b.k2;
}

/**
 *  Returns true if the line segment from (x0, y0) to (x1, y1) intersects the bounding box
 */
bool RRTXPlanner::isEdgeInCollision(double x0, double y0, double x1, double y1, 
                       double xmin, double ymin, double xmax, double ymax) {
    double t0 = 0.0;
    double t1 = 1.0;
    double dx = x1 - x0;
    double dy = y1 - y0;

    // p contains the direction vectors, q contains the distances to the boundaries
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};

    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0) {
            // Line is parallel to the boundary. If it's outside, it misses completely.
            if (q[i] < 0) return false;
        } else {
            double t = q[i] / p[i];
            if (p[i] < 0) {
                if (t > t1) return false; // Exits before it enters
                if (t > t0) t0 = t;       // Update entry point
            } else {
                if (t < t0) return false; // Exits before it enters
                if (t < t1) t1 = t;       // Update exit point
            }
        }
    }
    // If we get here, a valid intersection interval [t0, t1] exists within [0, 1]
    return true; 
}

/**
 * Computes square of distance between two nodes of the tree
 * Returns infinity if the graph edge is blocked
 */
double RRTXPlanner::squaredDistance(const std::size_t node_idx1, const std::size_t node_idx2) const {
    const Node& node1 = nodes_[node_idx1];
    if (node1.blocked_nbrs.find(node_idx2) != node1.blocked_nbrs.end()) return std::numeric_limits<double>::infinity();

    double dx = node1.x - nodes_[node_idx2].x;
    double dy = node1.y - nodes_[node_idx2].y;

    return (dx * dx) + (dy * dy);
}

/**
 * Computes distance between two nodes of the tree
 * Returns infinity if the graph edge is blocked
 */
double RRTXPlanner::distance(const std::size_t node_idx1, const std::size_t node_idx2) const {
    const Node& node1 = nodes_[node_idx1];
    if (node1.blocked_nbrs.find(node_idx2) != node1.blocked_nbrs.end()) return std::numeric_limits<double>::infinity();

    double dx = node1.x - nodes_[node_idx2].x;
    double dy = node1.y - nodes_[node_idx2].y;

    return std::hypot(dx, dy);
}

}
