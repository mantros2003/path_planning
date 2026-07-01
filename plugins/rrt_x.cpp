#include <custom_nav/rrt_x.h>
#include <custom_nav/utils.h>
#include <custom_nav/benchmark.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <cmath>
#include <algorithm>
#include <stack>

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTXPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

// Function to initialize and store the costmap info once.
void RRTXPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_    = costmap_ros->getCostmap();
        height_m_   = costmap_->getSizeInMetersY();
        width_m_    = costmap_->getSizeInMetersX();
        height_c_   = costmap_->getSizeInCellsY();
        width_c_    = costmap_->getSizeInCellsX();
        origin_x    = costmap_->getOriginX();
        origin_y    = costmap_->getOriginY();
        resolution_ = costmap_->getResolution();

        goal_ = Point<double, 2>{origin_x, origin_y};

        // Initialize the costmap snapshot
        // const unsigned int N = width_c_ * height_c_;
        // const uint8_t *charMap = costmap_->getCharMap();
        // costmap_snapshot_.assign(charMap, charMap + N);

        _buildFreeCellList();

        ros::NodeHandle private_nh("~/" + name);

        int min_x, min_y, max_x, max_y;

        private_nh.param<int>("sampling_min_x", min_x, 0);
        private_nh.param<int>("sampling_max_x", max_x, width_c_);
        private_nh.param<int>("sampling_min_y", min_y, 0);
        private_nh.param<int>("sampling_max_y", max_y, height_c_);

        sampling_min_x_ = min_x;
        sampling_max_x_ = max_x;
        sampling_min_y_ = min_y;
        sampling_max_y_ = max_y;

        ROS_INFO("[RRTXPlanner] has_param_1: %d\t has_param_2: %d\t has_param_3: %d\t has_param_4: %d\t", private_nh.hasParam("sampling_min_x"), private_nh.hasParam("sampling_max_x"), private_nh.hasParam("sampling_min_y"), private_nh.hasParam("sampling_max_y"));

        ROS_INFO("[RRTXPlanner] Initialized RRTX planner with map of size %fm * %fm", height_m_, width_m_);
        ROS_INFO("[RRTXPlanner] Origin: (%f, %f), Resolution: %f", origin_x, origin_y, resolution_);
        
        tree_pub_ = private_nh.advertise<visualization_msgs::Marker>(
            "search_tree", 1, true
        );

        initialized_ = true;
    } else {
        ROS_WARN("[RRTXPlanner] This node has already been initialized...");
    }
}

// Main function that plans the trajectory
bool RRTXPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    if (!initialized_) {
        ROS_ERROR("[RRTXPlanner] RRTXPlanner has not been initialized");
        return false;
    }

    ROS_INFO("[RRTXPlanner] Making path using RRTX");

    ros::Time start_time = ros::Time::now();

    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    
    ROS_INFO("[RRTXPlanner] Start position: (%f, %f), Goal position: (%f, %f)", sx, sy, gx, gy);
    
    // Check if the goal has changed and we need a new tree
    if (isNewGoal(gx, gy)) {
        ROS_INFO("[RRTXPlanner] Got a new goal, cleaning old containers");
        resetTree();
        costmap_snapshot_.clear();
        
        Node root_node(goal.pose.position.x, goal.pose.position.y);
        root_node.g = 0.0;
        root_node.lmc = 0.0;
        nodes_.push_back(root_node);
        kd_tree.insert(Point<double, 2>({root_node.x, root_node.y}), 0);
        planned_ = false;
        // Add to kd tree
        _buildFreeCellList();
        ROS_INFO("Number of cells in free cell list: %zu", free_cells.size());
    }
    
    // Modify the global variables
    goal_ = Point<double, 2>({gx, gy});
    start_ = Point<double, 2>({sx, sy});

    // Check if obstacles have changed and update the queues
    if (nodes_.size() > 1) {
        start_proxy = findStartProxy();
        updateObstacles();
    }

    // Grow or refine the tree
    // But if the initial tree is densed enough we need not sample more nodes
    // Sample, find nearest, grow tree
    int iters = 0;
    /*
     * Because of how we were sampling, the sample efficency was very less
     * The costmap has much more area than only the mapped region
     * To solve this issue, we can precompute the free cells and sample from them
     * We can then update this on runtime
     * UPDATE: The cells that are outside the main area have cost 0, so the above method does not work
     * We have to manually input the bounds of the area
     * Introduced sampling bounds that are input by the programmer
     */
    if (planned_) ROS_INFO("[RRTXPlanner] Replanning");
    int total_samples = 0;
    int out[5] = {0, 0, 0, 0, 0};

    unsigned int min_x = 1000, min_y = 1000;
    unsigned int max_x = 0, max_y = 0;
    // Modify it so that it only samples node initially, remove the second loop condition
    // while ((!planned_ && iters < max_iters_) || (isConnected(sx, sy) && iters < 5))
    // while ((!isConnected(sx, sy) && iters < max_iters_) || iters < 1)
    while (!planned_ && (iters < max_iters_))
    {
        // double x = origin_x + (rand01(rng) * width_m_);
        // double y = origin_y + (rand01(rng) * height_m_);

        // Sample a random point from the map
        double x, y;
        std::pair<double, double> sampled_pt = samplePoint();
        x = sampled_pt.first;
        y = sampled_pt.second;

        total_samples++;

        // First we convert to map/grid coordinates, then check if it is occupied
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my)) {
            out[0]++;
            continue;
        }
        if (costmap_->getCost(mx, my) >= obstacle_cost_threshold_) {
            out[1]++;
            continue;
        }

        Point<double, 2> random_pt({x, y});

        // Get the point nearest to the ranomly selected point
        std::size_t nearest_index = kd_tree.nearest(random_pt);
        Node& near_node = nodes_[nearest_index];
        Point<double, 2> near_pt{near_node.x, near_node.y};

        // Move in the direction of the random point from the near point
        Point<double, 2> new_pt = steer(near_pt[0], near_pt[1], random_pt[0], random_pt[1]);

        // Check if this point is in bounds
        if (!costmap_->worldToMap(new_pt[0], new_pt[1], mx, my)) {
            out[2]++;
            continue;
        }
        // Check if the new point is near an obstacle
        if (costmap_->getCost(mx, my) >= obstacle_cost_threshold_) {
            out[3]++;
            continue;
        }
        // Check if the path is free of obstacles
        if (hasObstacle(near_pt, new_pt)) {
            out[4]++;
            continue;
        }

        min_x = std::min(min_x, mx);
        min_y = std::min(min_y, my);
        max_x = std::max(max_x, mx);
        max_y = std::max(max_y, my);

        // If not an obstacle, then add the point to the tree
        Node new_node(new_pt[0], new_pt[1]);

        bool is_inserted = addPointToTree(new_pt, nearest_index);

        if (is_inserted) {
            radius_ = getRadius();

            rewireNeighbors(this->nodes_.size() - 1);
            // In the Julia implementation by Otte, new node was explicitly added to the heap
            verifyQueue(this->nodes_.size() - 1);
            reduceInconsistency();

            iters++;
        }
    }

    ROS_INFO("[RRTXPlanner] Our graph has %zu nodes", nodes_.size());
    ROS_INFO("[RRTXPlanner] Total samples taken: %d", total_samples);
    ROS_INFO("[RRTXPlanner] Failed samples: %d, %d, %d, %d, %d", out[0], out[1], out[2], out[3], out[4]);
    ROS_INFO("[RRTXPlanner] Bounding boxes of all the valid samples: (%u, %u) - (%u, %u)", min_x, min_y, max_x, max_y);

    #ifdef _VIS_RRTX_TREE
    buildTreeMarker();
    #endif

    if (!isConnected(sx, sy)) {
        ROS_WARN("[RRTXPlanner] Failed to find a path to the goal within max iterations.");
        return false;
    }

    if (planned_) {
        ROS_INFO("[RRTXPlanner] redInc stats: %zu calls, %ld us", profiling::redInc_stats.calls, profiling::redInc_stats.duration.count());
        ROS_INFO("[RRTXPlanner] addObs stats: %zu calls, %ld us", profiling::addObs_stats.calls, profiling::addObs_stats.duration.count());
        ROS_INFO("[RRTXPlanner] remObs stats: %zu calls, %ld us", profiling::remObs_stats.calls, profiling::remObs_stats.duration.count());


    }

    return extractPath(start, goal, plan, start_time);
}

/* Adds a new point to the tree */
bool RRTXPlanner::addPointToTree(Point<double, 2> p, std::size_t near_idx) {
    this->nodes_.emplace_back(p[0], p[1]);
    Node& new_node = this->nodes_.back();

    Node& near_node = this->nodes_[near_idx];

    // Get all points close to the node
    std::vector<std::size_t> neighbors = this->kd_tree.radius_search(p, radius_);

    // Find parent, node that has the least cost
    for (std::size_t nbr_idx: neighbors) {
        Node& nbr = this->nodes_[nbr_idx];

        if (!hasObstacle(p[0], p[1], nbr.x, nbr.y)) {
            new_node.nbr_init.push_back(nbr_idx);

            nbr.nbr_running.push_back(this->nodes_.size() - 1);

            double cost = nbr.lmc + utils::distance<double>(nbr.x, nbr.y, p[0], p[1]);
            if (cost < new_node.lmc) {
                new_node.lmc = cost;
                new_node.par_idx = nbr_idx;
            }
        }
    }

    // We have iterated over all the nodes that were within radius_ of the new node
    // If we have not been able to find a parent, we do not add the node
    if (new_node.par_idx == Node::INVALID_IDX) {
        this->nodes_.pop_back();
        return false;
    }

    // Add to the parents children list and to the kd tree
    this->nodes_[new_node.par_idx].children.push_back(this->nodes_.size() - 1);
    this->kd_tree.insert(p, this->nodes_.size() - 1);

    return true;
}

bool RRTXPlanner::extractPath(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan,
    ros::Time start_time
) {
    // Clear any old path data
    plan.clear();

    // Find the node closest to our current start position to begin tracing
    // Or should we use the start proxy calculated initially
    std::size_t current_idx = findStartProxy();

    if (current_idx == Node::INVALID_IDX) {
        ROS_ERROR("[RRTXPlanner] Start proxy is invalid despite tree being connected.");
        return false;
    }

    // Add the exact starting pose so the local planner has a smooth beginning
    plan.push_back(start);

    std::size_t loop_safety_counter = 0;
    
    // Trace the tree from the start proxy up to the goal root
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
            ROS_ERROR("[RRTXPlanner] Infinite loop detected during path extraction");
            plan.clear();
            for (int i = 0; i < 5; i++) {
                const Node& curr_node = nodes_[current_idx];
                ROS_ERROR("Node index: %lu\tparent: %lu\tg: %f\tlmc: %f\tdist_to_par: %f", current_idx, curr_node.par_idx, curr_node.g, curr_node.lmc, distance(current_idx, curr_node.par_idx));
                current_idx = curr_node.par_idx;
            }
            return false;
        }
    }

    // Add the exact goal pose for precise final positioning
    plan.push_back(goal);

    this->planned_ = true;
    double path_len = utils::computePathLength(plan);
    ROS_INFO("[RRTXPlanner] Successfully extracted path with %zu waypoints,\tPath length: %.3f m,\tPlanning time: %.3f ms", plan.size(), path_len, (ros::Time::now() - start_time).toSec() * 1000);
    
    return true;
}

void RRTXPlanner::reduceInconsistency() {
    // Anonymous function that returns the minimum in the set
    auto topKey = [&]() { return *queue_.begin(); };

    // Make key and extract the closest node to the current bot position
    QKey botKey;
    Node* bot = nullptr;
    if (start_proxy == Node::INVALID_IDX) {
        Node _bot;

        double dist = 1.3 * utils::distance<double>(start_[0], start_[1], goal_[0], goal_[1]);
        botKey.k1 = dist;
        botKey.k2 = dist;
        botKey.index = Node::INVALID_IDX;

        _bot.x = start_[0]; _bot.y = start_[1];
        _bot.g = dist;
        _bot.lmc = dist;
        _bot.in_queue = false;

        bot = &_bot;
    } else {
        botKey = makeKey(start_proxy);
        bot = &nodes_[start_proxy];
    }

    // Stop when the robot's proxy is consistent 
    // And all cheaper nodes have been processed.
    while (!queue_.empty() &&
           (keyLess(topKey(), botKey)               ||
            std::abs(bot->lmc - bot->g) > epsilon_    ||
            bot->g == std::numeric_limits<double>::infinity() ||
            bot->in_queue))
    {
        // Remove the minimum from the queue
        auto top = queue_.begin();
        std::size_t v_idx = top->index;
        queue_.erase(top);
        queueMap_.erase(v_idx);

        if (v_idx == 0) ROS_WARN("[RRTXPlanner] Root is in the queue");

        Node& v = nodes_[v_idx];
        v.in_queue = false;

        if (v.g - v.lmc > epsilon_) {
            updateLMC(v_idx);
            rewireNeighbors(v_idx);
        }

        v.g = v.lmc;
        
        // Update botKey just in case the proxy's cost changed during the loop
        if (start_proxy != Node::INVALID_IDX) botKey = makeKey(start_proxy);
    }
}

/**
 * Checks if the node's path cost can be reduced by rewiring to a new parent
 */
void RRTXPlanner::rewireNeighbors(std::size_t v_index) {
    Node& v = nodes_[v_index];

    if (v.g - v.lmc <= epsilon_) return;

    cullNeighbors(v_index);

    auto rewire = [&](const auto& neighbors) {
        for (std::size_t nbr_index: neighbors) {
            if (nbr_index == v.par_idx || nbr_index == 0) continue;

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
    };

    rewire(v.nbr_init);
    rewire(v.nbr_running);
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
                if (utils::distance<double>(node.x, node.y, nbr.x, nbr.y) > radius_) {
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
    if (v_idx == 0) {
        ROS_WARN("[RRTXPlanner] Trying to update root's lmc... stopped");
        return;
    }
    cullNeighbors(v_idx);
    
    Node& v = nodes_[v_idx];
    
    double min_cost = std::numeric_limits<double>::infinity();
    std::size_t best_parent = Node::INVALID_IDX;

    Point<double, 2> v_pt({v.x, v.y});

    auto findBetterParent = [&] (std::size_t u_idx) {
        Node& u = nodes_[u_idx];

        // Prevent infinite routing loops by ensuring we dont pick our own child
        if (u_idx == v_idx || u.par_idx == v_idx) return;

        Point<double, 2> u_pt({u.x, u.y});

        if (orphan_set_.count(u_idx) > 0) return;

        double cost = distance(v_idx, u_idx) + u.lmc;

        if (min_cost > cost) {
            min_cost = cost;
            best_parent = u_idx;
        }
    };

    for (std::size_t u_idx: v.nbr_running) findBetterParent(u_idx);
    for (std::size_t u_idx: v.nbr_init) findBetterParent(u_idx);

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
        if (v_idx == 0) ROS_WARN("[RRTXPlanner] Setting root's parent to %lu in updateLMC", best_parent);
    }
    v.lmc = min_cost;
}

/**
 * Handles the change of obstacles
 * Updates the set of obstacles, and handles them accordingly
 */
void RRTXPlanner::updateObstacles() {
    // Make two vectors to store the cells that have changed
    std::vector<unsigned int> newly_blocked;
    std::vector<unsigned int> newly_cleared;

    // Get the new costmap data
    const uint8_t* live = costmap_->getCharMap();
    const unsigned int N = width_c_ * height_c_;

    // First invocation of the function
    if (costmap_snapshot_.empty()) {
        TIME("cmap_snapshot", costmap_snapshot_.assign(live, live + N));
        return;
    }

    // Diff with the old snapshot
    // TIME("cmap_diff",
    // for (unsigned int i = 0; i < N; i++) {
    //     bool was_obs = (costmap_snapshot_[i] >= obstacle_cost_threshold_);
    //     bool is_obs = (live[i] >= obstacle_cost_threshold_);

    //     if (!was_obs && is_obs) newly_blocked.push_back(i);
    //     if (was_obs && !is_obs) newly_cleared.push_back(i);
    // }
    // );

    // Optimization: We only process the subset of the costmap that contains our map
    TIME("cmap_diff",
    for (unsigned int y = sampling_min_y_; y <= sampling_max_y_; y++) {
        for (unsigned int x = sampling_min_x_; x <= sampling_max_x_; x++) {
            unsigned int i = y * width_c_ + x;

            bool was_obs = (costmap_snapshot_[i] >= obstacle_cost_threshold_);
            bool is_obs = (live[i] >= obstacle_cost_threshold_);

            if (!was_obs && is_obs) newly_blocked.push_back(i);
            if (was_obs && !is_obs) newly_cleared.push_back(i);

            costmap_snapshot_[i] = live[i];
        }
    }
    );

    ROS_INFO("[RRTXPlanner] Obstacles added: %zu", newly_blocked.size());
    ROS_INFO("[RRTXPlanner] Obstacles removed: %zu", newly_cleared.size());

    // Store the new data
    // std::memcpy(costmap_snapshot_.data(), live, N);

    // Handle removed obstacles
    ROS_INFO("Removing obstacles...");
    if (!newly_cleared.empty()) {
        for (unsigned int i: newly_cleared) BENCHMARK(remObs, removeObstacle(i));
        BENCHMARK(redInc, reduceInconsistency());
    }
    ROS_INFO("Completed.");

    // Handle added obstacles
    ROS_INFO("Adding obstacles...");
    if (!newly_blocked.empty()) {
        for (unsigned int i: newly_blocked) BENCHMARK(addObs, addObstacle(i));
        propogateDescendents();
        verifyQueue(start_proxy);
        BENCHMARK(redInc, reduceInconsistency());
    }
    ROS_INFO("Completed.");
}

/**
 * Removes obstacles
 * Makes necessary changes to ensure the newly freed path is used
 */
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

        for (std::size_t u_idx: neighbors_to_check) {
            if (v_idx > u_idx) continue;

            Node& u = nodes_[u_idx];

            // Verify the edge is clear against all remaining obstacles
            if (!hasObstacle(Point<double, 2>({v.x, v.y}), Point<double, 2>({u.x, u.y}))) {
                if (u_idx == 0) ROS_INFO("[RRTXPlanner] Root is other end of edge");

                v.blocked_nbrs.erase(u_idx);
                u.blocked_nbrs.erase(v_idx);

                updateLMC(v_idx);
                updateLMC(u_idx);

                if (std::abs(v.lmc - v.g) > epsilon_) verifyQueue(v_idx);
                if (std::abs(u.lmc - u.g) > epsilon_) verifyQueue(u_idx);
            }
        }
    }
}

/**
 * Adds cell with index i to the set of obstacles
 * Further processes and orphans the nodes affected
 */
void RRTXPlanner::addObstacle(unsigned int i) {
    obstacles_.push_back(i);

    unsigned int obstacle_x = i % width_c_;
    unsigned int obstacle_y = i / width_c_;

    // Get the points that make the grid cell
    double xmin = origin_x + obstacle_x * resolution_;
    double ymin = origin_y + obstacle_y * resolution_;
    double xmax = xmin + resolution_;
    double ymax = ymin + resolution_;

    // auto process = [&] (auto& store) {
    //     for (std::size_t u_idx: store) {
    //         if (v.blocked_nbrs.count(u_idx) > 0) continue;

    //         Node& u = nodes_[u_idx];
    //         if (isEdgeInCollision(u.x, u.y, v.x, v.y, xmin, ymin, xmax, ymax)) {
    //             v.blocked_nbrs.insert(u_idx);
    //             u.blocked_nbrs.insert(v_idx);

    //             if (v.par_idx == u_idx) verifyOrphan(v_idx);
    //             else if (u.par_idx == v_idx) verifyOrphan(u_idx);

    //             // Can also add condition to stop the robot if the obstacle is in current path
    //         }
    //     }
    // };

    auto process_neighbors = [&](const auto& neighbors, Node& v, std::size_t v_idx) {
        for (std::size_t u_idx : neighbors) {
            if (v.blocked_nbrs.count(u_idx) > 0) continue;

            Node& u = nodes_[u_idx];

            if (isEdgeInCollision(
                    u.x, u.y, v.x, v.y,
                    xmin, ymin, xmax, ymax
                )) {

                v.blocked_nbrs.insert(u_idx);
                u.blocked_nbrs.insert(v_idx);

                if (v.par_idx == u_idx) verifyOrphan(v_idx);
                else if (u.par_idx == v_idx) verifyOrphan(u_idx);

                // Can also add condition to stop the robot if the obstacle is in current path
            }
        }
    };

    // Make the set of edges that intersect this obstacle
    // Get the points that are within step_length_ of the obstacle
    std::vector<std::size_t> possible_invalidated_points = kd_tree.radius_search(Point<double, 2>({xmin, ymin}), step_length_ + resolution_);
    for (std::size_t v_idx: possible_invalidated_points) {
        Node& v = nodes_[v_idx];

        process_neighbors(v.nbr_running, v, v_idx);
        process_neighbors(v.nbr_init, v, v_idx);
    }
}

/**
 * Returns a point that is between the random point and the near point
 */
Point<double, 2> RRTXPlanner::steer(double near_x, double near_y, double rand_x, double rand_y) {
    double new_x, new_y;

    double dx = rand_x - near_x;
    double dy = rand_y - near_y;
    double len = utils::distance<double>(near_x, near_y, rand_x, rand_y);

    if (len <= step_length_) return Point<double, 2>({rand_x, rand_y});

    new_x = near_x + (step_length_ * dx / len);
    new_y = near_y + (step_length_ * dy / len);

    return Point<double, 2>({new_x, new_y});
}

/* Checks if the goal is changed */
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
        Node& n = nodes_[idx];

        if (std::isinf(n.g)) continue;

        Point<double, 2> n_pt({n.x, n.y});
        double distance_to_goal = n.lmc + n_pt.distance(start_pt);

        if (distance_to_goal <= min_distance) {
            min_distance = distance_to_goal;
            best_index = idx;
        }
    }

    return best_index != Node::INVALID_IDX;
}

/**
 * Scans the line from start to finish
 * Checks every cell, and returns if there is an obstacle on any of them
 */
bool RRTXPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_c_, sy = start / width_c_, gx = end % width_c_, gy = end / width_c_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= this->obstacle_cost_threshold_) return true;
    }
    return false;
}

/**
 * Checks if there is an obstacle between p1 and p2
 */
bool RRTXPlanner::hasObstacle(Point<double, 2> p1, Point<double, 2> p2) {
    return hasObstacle(p1[0], p1[1], p2[0], p2[1]);
}

/* Checks if there is an obstacle between (x1, y1) and (x2, y2) */
bool RRTXPlanner::hasObstacle(double x1, double y1, double x2, double y2) {
    unsigned int start, end;
    unsigned int mx, my;

    if (!costmap_->worldToMap(x1, y1, mx, my)) return true;
    start = costmap_->getIndex(mx, my);

    if (!costmap_->worldToMap(x2, y2, mx, my)) return true;
    end = costmap_->getIndex(mx, my);

    return hasObstacle(start, end);
}

// NOTE: Make this function more modular and
// remove the hardcoded 0.5 and add 1/dim logic
double RRTXPlanner::getRadius() const {
    double n = static_cast<double>(nodes_.size());
    if (n <= 1) return step_length_;

    return std::min(rad_const_ * std::pow(std::log(1 + n) / n, 0.5), step_length_);
}

/**
 * We don't usually have start node in our tree
 * So we use the next nearest unblocked node as a proxy for the start node
 */
std::size_t RRTXPlanner::findStartProxy() {
    // Get the nearest node
    // std::size_t proxy_idx = kd_tree.nearest(start_);
    
    // If the tree is empty or invalid, return the invalid index
    // if (proxy_idx == Node::INVALID_IDX || proxy_idx >= nodes_.size()) {
    //     return Node::INVALID_IDX;
    // }

    // Node& nearest_node = nodes_[proxy_idx];
    // Point<double, 2> proxy_pt({nearest_node.x, nearest_node.y});

    // If there's no obstacle between the robot and this node
    // if (!hasObstacle(start_, proxy_pt)) {
    //     if (proxy_idx == 0) ROS_WARN("[RRTXPlanner] Start proxy is the root");
    //     return proxy_idx;
    // }

    // We must search a local radius for the closest node that we can actually see
    // We can change search radius to a multiple of step length

    // Can also use some other value, eg 2 * step_length_
    double search_radius = radius_;
    std::vector<std::size_t> local_neighbors = kd_tree.radius_search(start_, search_radius);

    double min_dist = std::numeric_limits<double>::infinity();
    std::size_t best_visible_proxy = Node::INVALID_IDX;

    for (std::size_t nbr_idx : local_neighbors) {
        Node& nbr = nodes_[nbr_idx];

        if (std::isinf(nbr.lmc)) continue;

        Point<double, 2> nbr_pt({nbr.x, nbr.y});
        // Check if we have a clear path to this neighbor
        // And if we improve the previous cost
        double dist = nbr.lmc + start_.distance(nbr_pt);

        if (dist <= min_dist && !hasObstacle(start_, nbr_pt)) {
            min_dist = dist;
            best_visible_proxy = nbr_idx;
        }
    }

    return best_visible_proxy;
}

/**
 * Orphan all the nodes blocked by the obstacle
 * And all their descendents
 */
void RRTXPlanner::propogateDescendents() {
    std::vector<std::size_t> processing_queue(orphan_set_.begin(), orphan_set_.end());
    std::size_t head = 0;
    
    // Do a BFS and add all the children
    while (head < processing_queue.size()) {
        std::size_t v_idx = processing_queue[head++];
        for (std::size_t child_idx : nodes_[v_idx].children) {
            // std::unordered_set::insert returns a pair, .second is true if insertion took place
            if (orphan_set_.insert(child_idx).second) processing_queue.push_back(child_idx);
        }
    }

    // Invalidate neighboring nodes that rely on these orphans
    for (std::size_t v_idx : orphan_set_) {
        const Node& v = nodes_[v_idx];
        
        auto invalidate_neighbor = [&](std::size_t u_idx) {
            if (u_idx != 0 && u_idx != Node::INVALID_IDX && orphan_set_.count(u_idx) == 0) {
                if (u_idx == 0) ROS_WARN("[RRTXPlanner] Root is being added to the queue");
                nodes_[u_idx].g = std::numeric_limits<double>::infinity();
                verifyQueue(u_idx);
            }
        };

        for (std::size_t u_idx: v.nbr_running) invalidate_neighbor(u_idx);
        for (std::size_t u_idx: v.nbr_init) invalidate_neighbor(u_idx);
        invalidate_neighbor(v.par_idx);
    }

    // Sever all tree connections of the nodes in orphan set
    for (std::size_t v_idx: orphan_set_) {
        if (v_idx == 0) ROS_WARN("[RRTXPlanner] Root is in the orphan set");
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

/**
 * Add the node to orphan set
 * If in queue, remove from queue
 */
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

/**
 * Update the key of the node in the queue
 */
void RRTXPlanner::verifyQueue(std::size_t node_idx) {
    if (node_idx == 0) {
        ROS_WARN("[RRTXPlanner] Root is being added to queue");
        return;
    }
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
    const double EPS = 1e-9;

    if (xmin > xmax) std::swap(xmin, xmax);
    if (ymin > ymax) std::swap(ymin, ymax);

    double t0 = 0.0;
    double t1 = 1.0;
    double dx = x1 - x0;
    double dy = y1 - y0;

    // Case of point
    if (std::abs(dx) < EPS && std::abs(dy) < EPS) return (x0 >= xmin && x0 <= xmax && y0 >= ymin && y0 <= ymax);

    // p contains the direction vectors, q contains the distances to the boundaries
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};

    for (int i = 0; i < 4; ++i) {
        if (std::abs(p[i]) < EPS) {
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

    return t0 <= t1 && t1 >= 0.0 && t0 <= 1.0;
}

/**
 * Randomly samples a cell from the free cell list
 * Then adds random noise so that the final point is a random point from inside the cell
 */
std::pair<double, double> RRTXPlanner::samplePoint() {
    std::size_t num_free_cells = free_cells.size();
    std::uniform_int_distribution<> distr(0, num_free_cells - 1);

    std::size_t random_cell = distr(rng);
    std::pair<unsigned int, unsigned int> point = free_cells[random_cell];

    unsigned int cell_x, cell_y;
    cell_x = point.first;
    cell_y = point.second;

    double wx, wy;
    costmap_->mapToWorld(cell_x, cell_y, wx, wy);

    wx += (rand01(rng) - 0.5) * resolution_;
    wy += (rand01(rng) - 0.5) * resolution_;

    return std::make_pair(wx, wy);
}

/**
 * Populate the initial free cell list
 */
void RRTXPlanner::_buildFreeCellList() {
    free_cells.clear();

    for (unsigned int y = sampling_min_y_; y <= sampling_max_y_; y++) {
        for (unsigned int x = sampling_min_x_; x < sampling_max_x_; x++) {
            if (costmap_->getCost(x, y) < obstacle_cost_threshold_) free_cells.emplace_back(x, y);
        }
    }
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

/**
 * Sends a visualization message to visualize the tree in Rviz
 */
void RRTXPlanner::buildTreeMarker(const std::string& frame_id) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "search_tree";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    // line width
    marker.scale.x = 0.01;

    // green
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    if (nodes_.empty()) tree_pub_.publish(marker);

    std::stack<Node*> st;
    st.push(&nodes_[0]);

    while (!st.empty())
    {
        Node* parent = st.top();
        st.pop();

        for (std::size_t child_idx : parent->children)
        {
            Node* child = &nodes_[child_idx];

            geometry_msgs::Point p1;
            p1.x = parent->x;
            p1.y = parent->y;
            p1.z = 0.05;

            geometry_msgs::Point p2;
            p2.x = child->x;
            p2.y = child->y;
            p2.z = 0.05;

            marker.points.push_back(p1);
            marker.points.push_back(p2);

            st.push(child);
        }
    }

    ROS_INFO("[RRTXPlanner] Number of points reachable by goal node: %zu", marker.points.size());

    tree_pub_.publish(marker);
}

}
