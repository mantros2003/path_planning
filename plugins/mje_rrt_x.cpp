#include <custom_nav/mje_rrt_x.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <cmath>
#include <algorithm>
#include <stack>

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::MJERRTXPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

void MJERRTXPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!this->initialized_) {
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_id_ = costmap_ros->getGlobalFrameID();
        this->updateCostmapParams();

        goal_ = Point<double, 2>{origin_x, origin_y};

        ros::NodeHandle private_nh("~/" + name);

        int min_x, min_y, max_x, max_y;

        private_nh.param<int>("sampling_min_x", min_x, 0);
        private_nh.param<int>("sampling_max_x", max_x, width_c_ - 1);
        private_nh.param<int>("sampling_min_y", min_y, 0);
        private_nh.param<int>("sampling_max_y", max_y, height_c_ - 1);

        this->sampling_min_x_ = min_x;
        this->sampling_max_x_ = max_x;
        this->sampling_min_y_ = min_y;
        this->sampling_max_y_ = max_y;

        std::string theta_file, kappa_file, singular_file, arclength_file;
        std::vector<std::string> alpha_files;

        // Load filenames and build the table
        if (utils::loadParamString(private_nh, "theta_file", theta_file) &&
            utils::loadParamString(private_nh, "kappa_file", kappa_file) &&
            utils::loadParamString(private_nh, "singular_file", singular_file) &&
            utils::loadParamString(private_nh, "arclength_file", arclength_file)) {}
        else { ROS_WARN("[MJERRTXPlanner] All necessary files do not exist"); }

        private_nh.param<std::vector<std::string>>(
                "alpha_files", alpha_files, std::vector<std::string>());
        if (alpha_files.size() < 4) { ROS_WARN("[MJERRTXPlanner] Need atleast 4 alpha LUTs"); }

        this->constraint_table_.buildTable(
                theta_file, kappa_file, singular_file, arclength_file, alpha_files);

        tree_pub_ = private_nh.advertise<visualization_msgs::Marker>(
            "search_tree", 1, true
        );
        stats_pub_ = private_nh.advertise<custom_nav::RRTXStats>("stats", 10);

        buildFreeCellList();

        this->initialized_ = true;
    } else {
        ROS_WARN("[MJERRTXPlanner] Already initialized.");
    }
}

bool MJERRTXPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    if (!this->initialized_) {
        ROS_ERROR("[MJERRTXPlanner] MJERRTXPlanner has not been initialized");
        return false;
    }

    ros::Time start_time = ros::Time::now();
    ros::WallTime wall_start_time = ros::WallTime::now();

    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    if (isNewGoal(gx, gy)) {
        ROS_INFO("[MJERRTXPlanner] Got a new goal, cleaning old containers");

        resetTree();
        this->costmap_snapshot_.clear();
        
        // Add new goal node
        Node root_node(goal.pose.position.x, goal.pose.position.y);
        root_node.g = 0.0;
        root_node.lmc = 0.0;
        this->nodes_.push_back(root_node);
        this->kd_tree.insert(Point<double, 2>({root_node.state.x, root_node.state.y}));

        this->planned_ = false;

        buildFreeCellList();

        ROS_INFO("Number of cells in free cell list: %zu", free_cells.size());
    }

    this->goal_ = Point<double, 2>({gx, gy});
    this->start_ = Point<double, 2>({sx, sy});

    if (this->nodes_.size() > 1) {
        this->start_proxy = findStartProxy();
        updateObstacles();
    }

    std::size_t iters = 0;

    while (!this->planned_ && (iters < this->max_iters_))
    {
        // Sample a random state from the map
        double x, y;
        std::pair<double, double> sampled_pt = samplePoint();
        x = sampled_pt.first;
        y = sampled_pt.second;

        // First we convert to map/grid coordinates, then check if it is occupied
        if (!validatePoint(x, y)) continue;
        
        Point<double, 2> random_pt({x, y});

        // Get the nearest feasible
        std::pair<std::size_t, State> res = feasibleNearAndSteer(random_pt);
        std::size_t nearest_index = res.first;

        if (nearest_index == Node::INVALID_IDX) continue;

        Node& near_node = this->nodes_[nearest_index];
        Point<double, 2> near_pt{near_node.state.x, near_node.state.y};

        // Move in the direction of the random point from the near point
        State new_state = res.second;
        Point<double, 2> new_pt({new_state.x, new_state.y});

        // Check if this point is in bounds
        if (!validatePoint(new_pt[0], new_pt[1])) continue;
        
        // Check if the path is free of obstacles
        if (hasObstacle(near_pt, new_pt)) continue;

        addPointToTree(new_state, nearest_index);

        this->radius_ = getRadius();

        rewireNeighbors(this->nodes_.size() - 1);
        // In the Julia implementation by Otte, new node was explicitly added to the heap
        verifyQueue(this->nodes_.size() - 1);
        reduceInconsistency();

        iters++;
    }

    double planning_time = (ros::WallTime::now() - wall_start_time).toSec() * 1000.0;

    // Build the stats message
    // custom_nav::RRTXStats stats_msg;
    // stats_msg.header.stamp = ros::Time::now();
    // stats_msg.header.frame_id = this->global_frame_id_;
    // stats_msg.planning_time = planning_time;

    ROS_INFO("[RRTXPlanner] Our graph has %zu nodes", this->nodes_.size());

    #ifdef _VIS_RRTX_TREE
    buildTreeMarker();
    #endif

    if (!isConnected(sx, sy)) {
        ROS_WARN("[RRTXPlanner] Failed to find a path to the goal within max iterations.");
        ROS_WARN("[RRTXPlanner] Flushing the inconsistency queue.");
        flushQueue();
        if (!isConnected(sx, sy)) {
            ROS_WARN("[RRTXPlanner] Still unable to find a path");
                // stats_msg.path_found = false;
                // stats_pub_.publish(stats_msg);
                return false;
        }
    }

    return extractPath(start, goal, plan, start_time);
}

/* Returns a random state */
// State MJERRTXPlanner::sampleState() {
//     State random_state(this->rand_state_x_(),
//                        this->rand_state_y_(),
//                        this->rand_state_theta_());
// 
//     return random_state;
// }

/* Set the values of all the costmap related constants */
void MJERRTXPlanner::updateCostmapParams() {
    this->height_m_     = this->costmap_->getSizeInMetersY();
    this->width_m_      = this->costmap_->getSizeInMetersX();
    this->height_c_     = this->costmap_->getSizeInCellsY();
    this->width_c_      = this->costmap_->getSizeInCellsX();
    this->origin_x      = this->costmap_->getOriginX();
    this->origin_y      = this->costmap_->getOriginY();
    this->resolution_   = this->costmap_->getResolution();
}

/* Extract the path from the tree by moving from start to goal */
bool MJERRTXPlanner::extractPath(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan,
    ros::Time start_time) {
    // Clear any old path data
    plan.clear();

    // Find the node closest to our current start position to begin tracing
    // Or should we use the start proxy calculated initially
    std::size_t current_idx = findStartProxy();

    if (current_idx == Node::INVALID_IDX) {
        ROS_ERROR("[MJERRTXPlanner] Start proxy is invalid despite tree being connected.");
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
        pose.pose.position.x = curr_node.state.x;
        pose.pose.position.y = curr_node.state.y;
        pose.pose.position.z = 0.0;
        
        double half_angle = curr_node.state.theta / 2;

        pose.pose.orientation.w = std::cos(half_angle);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(half_angle);

        plan.push_back(pose);

        // Break if we reached the goal node (where cost is 0)
        if (curr_node.g == 0.0) break;

        // Move to the parent node
        current_idx = curr_node.par_idx;

        // For safety
        loop_safety_counter++;
        if (loop_safety_counter > nodes_.size()) {
            ROS_ERROR("[RRTXPlanner] Infinite loop detected during path extraction");
            plan.clear();
            for (int i = 0; i < 5; i++) {
                const Node& curr_node = nodes_[current_idx];
                ROS_ERROR("Node index: %lu\tparent: %lu\t pos: (%f, %f)\tg: %f\tlmc: %f\tdist_to_par: %f", current_idx, curr_node.par_idx, curr_node.state.x, curr_node.state.y, curr_node.g, curr_node.lmc, distance(current_idx, curr_node.par_idx));
                current_idx = curr_node.par_idx;
            }
            return false;
        }
    }

    // Add the exact goal pose for precise final positioning
    plan.push_back(goal);

    this->planned_ = true;
    // double path_len = utils::computePathLength(plan);
    ROS_INFO("[RRTXPlanner] Successfully extracted path with %zu waypoints,Planning time: %.3f ms", plan.size(), (ros::Time::now() - start_time).toSec() * 1000);
    
    return true;
}

/**
 * Checks if coordinates are in map bounds
 * and if the point is on an obstacle
 */
bool MJERRTXPlanner::validatePoint(double x, double y) {
    unsigned int mx, my;
    if (!this->costmap_->worldToMap(x, y, mx, my)) return false;
    if (this->costmap_->getCost(mx, my) >= this->obstacle_cost_threshold_) return false;

    return true;
}

/* Checks if the goal is changed */
bool MJERRTXPlanner::isNewGoal(double gx, double gy) {
    return (gx != goal_[0]) || (gy != goal_[1]);
}

/**
 * Checks if point `p` is connected to the tree
 * Searches for points within `step_length_` of the start and chooses the best possible option
 */
bool MJERRTXPlanner::isConnected(double sx, double sy) {
    if (nodes_.empty()) return false;

    Point<double, 2> start_pt({sx, sy});

    std::vector<std::size_t> neighbors = kd_tree.radius_search(start_pt, step_length_);

    std::size_t best_index = Node::INVALID_IDX;
    double min_distance = std::numeric_limits<double>::infinity();

    for (std::size_t idx: neighbors) {
        Node& n = nodes_[idx];

        if (std::isinf(n.g)) continue;

        Point<double, 2> n_pt({n.state.x, n.state.y});
        double distance_to_goal = n.lmc + n_pt.distance(start_pt);

        if (distance_to_goal <= min_distance) {
            min_distance = distance_to_goal;
            best_index = idx;
        }
    }

    return best_index != Node::INVALID_IDX;
}

/* Adds a new point to the tree */
void MJERRTXPlanner::addPointToTree(State new_state, std::size_t near_idx) {
    this->nodes_.emplace_back(new_state.x, new_state.y, new_state.theta);
    Node& new_node = this->nodes_.back();
    new_node.par_idx = near_idx;

    Node& near_node = this->nodes_[near_idx];

    // Get all points close to the node
    std::vector<std::size_t> neighbors = this->kd_tree.radius_search(p, this->radius_);

    // Add neighbors
    for (std::size_t nbr_idx: neighbors) {
        Node& nbr = this->nodes_[nbr_idx];

        if (!hasObstacle(p[0], p[1], nbr.state.x, nbr.state.y)) {
            new_node.nbr_init.push_back(nbr_idx);
            nbr.nbr_running.push_back(this->nodes_.size() - 1);
        }
    }

    double arclength = getarclength(new_state, near_node.state);

    new_node.lmc = near_node.lmc + arclength;
    new_node.par_idx = near_idx;

    // Add to the parents children list and to the kd tree
    this->nodes_[new_node.par_idx].children.push_back(this->nodes_.size() - 1);
    this->kd_tree.insert(p, this->nodes_.size() - 1);
}

/**
 * Returns a pair: near node's index, and the random state
 * Index is std::size_t::max if no feasible node is found
 */
std::pair<std::size_t, State> MJERRTXPlanner::feasibleNearAndSteer(Point<double, 2> rand_pt) {
    // Get the nearest node to the random point
    // And search in a circle of c * near_dist
    std::size_t nearest = this->kd_tree.nearest(rand_pt);
    State& nearest_node_state = this->nodes_[nearest].state;
    Point<double, 2> nearest_pt({nearest_node_state.x, nearest_node_state.y});
    std::vector<std::size_t> neighbors =
        this->kd_tree.radius_search(rand_pt, 1.5 * rand_pt.distance(nearest_pt));
    utils::sortVectorByDist<double, 2>(neighbors, this->nodes_, this->goal_);

    State random_state{rand_pt[0], rand_pt[1], 0};
    
    // Iterating over neighbors to find a suitable parent
    for (std::size_t nbr_idx: neighbors) {
        Node& node = this->nodes_[nbr_idx];

        double dx = node[1] - rand_pt[1], dy = node[0] - rand_pt[0];
        random_state.theta = std::atan2(dy, dx);

        double actualKmax;
        bool singular;
        bool feasible = isFeasible(random_state, node.state, actualKmax, singular);

        if (!feasible) continue;

        double chord_length = std::hypot(dy, dx);

        if (chord_length > this->step_length_) {
            if (actualKmax >= this->kc) ROS_WARN("[MJERRTXPlanner] Should not be possible.");
            
            double L = std::max(this->step_length_, chord_length * (actualKmax / this->kc));
            random_state.x = node.state.x + (L * std::cos(random_state.theta));
            random_state.y = node.state.y + (L * std::sin(random_state.theta));
        }

        if (hasObstacle(random_state.x, random_state.y, node.state.x, node.state.y)) continue;


        return std::pair<std::size_t, State>(nbr_idx, random_state);
    }

    return {Node::INVALID_IDX, random_state};
}

// TODO: Check this function, if it is needed or not
// and change the radius search line, make a point instead of using p
void MJERRTXPlanner::rewirePointFeasible(std::size_t new_node_idx) {
    Node& new_node = this->nodes_[new_node_idx];

    double curr_cost = new_node.g;
    std::size_t curr_par = new_node.par_idx;
    double ang_org = new_node.state.theta;
    double new_cost = std::numeric_limits<double>::infinity();

    std::vector<std::size_t> neighbors = this->kd_tree.radius_search(p, this->radius_);

    State random_state{rand_pt[0], rand_pt[1], 0};
    for (std::size_t nbr_idx: neighbors) {
        if (nbr_idx == new_node.par_idx || nbr_idx == new_node_idx) continue;

        Node& nbr_node = this->nodes_[nbr_idx];

        new_node.state.theta = std::atan2(nbr_node.state.y - new_node.state.y,
                nbr_node.state.x - new_node.state.x);
        double chord_length = std::hypot(nbr_node.state.y - new_node.state.y,
                nbr_node.state.x - new_node.state.x);

        double actualKmax; bool singular;
        feasible = isFeasible(random_state, nbr_node.state, actualKmax, singular);

        if (!feasible) continue;

        double arc_length = getarclength(new_node.state, nbr_node.state);
        double new_cost = nbr_node.g + arc_length;

        if (new_cost < curr_cost) {
            if (hasObstacle(nbr_node.state.x, nbr_node.state.y, 
                        new_node.state.x, new_node.state.y)) continue;

            curr_par = nbr_idx;
            curr_cost = new_cost;
            ang_org = new_node.state.theta;
        }
    }

    remove_child(new_node.par_idx, new_node_idx);
    new_node.par_idx = curr_par;
    new_node.g = curr_cost;
    add_child(new_node.par_idx, new_node_idx);
}

/**
 * Try to re-route the the neighbours through the new node
 */
void MJERRTXPlanner::rewireNeighbors(std::size_t node_idx) {
    Node& node = this->nodes_[node_idx];

    if (node.g - node.lmc <= this->epsilon_) return;

    cullNeighbors(node_idx);

    auto rewire = [&] (const auto& neighbors) {
        for (std::size_t nbr_idx: neighbors) {
            if (nbr_idx == node.par_idx || nbr_idx == node_idx) continue;

            Node& nbr_node = this->nodes_[nbr_idx];

            double chord_length = std::hypot(nbr_node.state.y - node.state.y,
                    nbr_node.state.x - node.state.x);

            double actualKmax; bool singular;
            bool feasible = isFeasible(nbr_node.state, node.state, actualKmax, singular);

            if (!feasible) continue;

            double arc_length = getarclength(nbr_node.state, node.state);
            double new_cost = node.lmc + arc_length;

            if (new_cost < nbr_node.lmc) {
                nbr_node.lmc = new_cost;
                if (nbr_node.par_idx != Node::INVALID_IDX) {
                    this->nodes_[nbr_node.par_idx].removeChild(nbr_idx);
                }
                nbr_node.par_idx = node_idx;
                node.children.push_back(nbr_idx);

                if (nbr_node.g - nbr_node.lmc > this->epsilon_) verifyQueue(nbr_idx);
            }
        }
    };
}

void MJERRTXPlanner::cullNeighbors(std::size_t node_idx) {
    Node& node = this->nodes_[node_idx];
    
    // We use remove_if to cleanly filter the vector
    node.nbr_running.erase(
        std::remove_if(node.nbr_running.begin(), node.nbr_running.end(),
            [&](std::size_t nbr_idx) {
                Node& nbr = this->nodes_[nbr_idx];

                // If the neighbor is now outside the shrinking radius
                if (utils::distance<double>(node.state.x, node.state.y, nbr.state.x, nbr.state.y) > this->radius_) {
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

/* Process the inconsistency queue */
void MJERRTXPlanner::reduceInconsistency() {
    // Anonymous function that returns the minimum in the set
    auto topKey = [&]() { return *this->queue_.begin(); };

    // Make key and extract the closest node to the current bot position
    QKey botKey;
    Node* bot = nullptr;
    Node _bot;
    if (start_proxy == Node::INVALID_IDX) {
        double dist = 1.3 * utils::distance<double>(start_[0], start_[1], goal_[0], goal_[1]);
        botKey.k1 = dist;
        botKey.k2 = dist;
        botKey.index = Node::INVALID_IDX;

        _bot.state.x = start_[0]; _bot.state.y = start_[1];
        _bot.g = dist;
        _bot.lmc = dist;
        _bot.in_queue = false;

        bot = &_bot;
    } else {
        botKey = makeKey(start_proxy);
        bot = &this->nodes_[start_proxy];
    }

    // Stop when the robot's proxy is consistent 
    // And all cheaper nodes have been processed.
    while (!this->queue_.empty() &&
           (keyLess(topKey(), botKey)               ||
            std::abs(bot->lmc - bot->g) > epsilon_    ||
            bot->g == std::numeric_limits<double>::infinity() ||
            bot->in_queue))
    {
        // Remove the minimum from the queue
        auto top = this->queue_.begin();
        std::size_t v_idx = top->index;
        this->queue_.erase(top);
        this->queueMap_.erase(v_idx);

        if (v_idx == 0) ROS_WARN("[RRTXPlanner] Root is in the queue");

        Node& v = this->nodes_[v_idx];
        v.in_queue = false;

        if (v.g - v.lmc > this->epsilon_) {
            updateLMC(v_idx);
            rewireNeighbors(v_idx);
        }

        v.g = v.lmc;
        
        // Update botKey just in case the proxy's cost changed during the loop
        if (this->start_proxy != Node::INVALID_IDX) botKey = makeKey(start_proxy);
    }
}

/* Empty the queue */
void MJERRTXPlanner::flushQueue() {
    while (!this->queue_.empty()) {
        auto top = this->queue_.begin();
        std::size_t v_idx = top->index;
        this->queue_.erase(top);
        this->queueMap_.erase(v_idx);

        Node& v = this->nodes_[v_idx];
        v.in_queue = false;

        if (v.g - v.lmc > this->epsilon_) {
            updateLMC(v_idx);
            rewireNeighbors(v_idx);
        }

        v.g = v.lmc;
    }
}
/**
 * Finds the best neighbor to travel through
 * Updates the parent and cost if improvement can be made
 */
void MJERRTXPlanner::updateLMC(std::size_t v_idx) {
    if (v_idx == 0) {
        ROS_WARN("[MJERRTXPlanner] Trying to update root's lmc... stopped");
        return;
    }

    cullNeighbors(v_idx);
    
    Node& v = nodes_[v_idx];
    
    std::pair<std::size_t, double> res = findBestParent(v_idx, v.nbr_running);
    std::size_t best_par_idx = res.first; double best_cost = res.second;

    res = findBestParent(v_idx, v.nbr_init);
    if (res.second < best_cost) {
        best_cost = res.second;
        best_par_idx = res.first;
    }

    if (v.lmc < best_cost) return;

    if (v.par_idx != best_par_idx) {
        if (v.par_idx != Node::INVALID_IDX) this->nodes_[v.par_idx].removeChild(v_idx);
        if (best_par_idx != Node::INVALID_IDX) this->nodes_[best_par_idx].children.push_back(v_idx);
        v.par_idx = best_par_idx;
    }
    v.lmc = best_cost;
}

/**
 * Finds the parent with least cost feasible path to goal
 * If a list of points is given, then uses that
 * Else does a kd-tree search
 * Returns the best parent and the associated cost
 */
std::pair<std::size_t, double> MJERRTXPlanner::findBestParent(
        std::size_t child_idx, Vector<std::size_t>* neighbors) {
    Node& child_node = this->nodes_[child_idx];

    if (neighbors == nullptr) {
        neighbors = &this->kd_tree.radius_search(
            Point<double, 2>(child_node.state.x, child_node.state.y), this->radius_);
    }

    double best_cost = child_node.g;
    std::size_t best_par = child_node.par_idx;

    for (std::size_t nbr_idx: *neighbors) {
        if (nbr_idx == child_idx || nbr_idx == child_node.par_idx) continue;

        Node& nbr_node = this->nodes_[nbr_idx];

        double Kmax;
        bool singular;
        bool feasible = isFeasible(child_node.state, nbr_node.state, Kmax, singular);

        if (!feasible) continue;

        double new_cost = nbr_node.g + getarclength(child_node.state, nbr_node.state);

        if (new_cost < best_cost) {
            if (hasObstacle(nbr_node.state.x, nbr_node.state.y, 
                        child_node.state.x, child_node.state.y)) continue;
            best_cost = new_cost;
            best_par = nbr_idx;
        }
    }

    return std::pair<std::size_t, double>(best_par, best_cost);
}

/**
 * Handles the change of obstacles
 * Updates the set of obstacles, and handles them accordingly
 */
void MJERRTXPlanner::updateObstacles() {
    // Make two vectors to store the cells that have changed
    std::vector<unsigned int> newly_blocked;
    std::vector<unsigned int> newly_cleared;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*this->costmap_->getMutex());

    updateCostmapParams();

    // Get the new costmap data
    const uint8_t* live = this->costmap_->getCharMap();
    const unsigned int N = this->width_c_ * this->height_c_;

    // First invocation of the function
    if (this->costmap_snapshot_.empty()) {
        this->costmap_snapshot_.assign(live, live + N);
        return;
    }

    // Optimization: We only process the subset of the costmap that contains our map
    for (unsigned int y = this->sampling_min_y_; y <= this->sampling_max_y_; y++) {
        for (unsigned int x = this->sampling_min_x_; x <= this->sampling_max_x_; x++) {
            unsigned int i = y * this->width_c_ + x;

            bool was_obs = (this->costmap_snapshot_[i] >= this->obstacle_cost_threshold_);
            bool is_obs = (live[i] >= this->obstacle_cost_threshold_);

            if (!was_obs && is_obs) newly_blocked.push_back(i);
            if (was_obs && !is_obs) newly_cleared.push_back(i);

            this->costmap_snapshot_[i] = live[i];
        }
    }

    lock.unlock();

    ROS_INFO("[MJERRTXPlanner] Obstacles added: %zu", newly_blocked.size());
    ROS_INFO("[MJERRTXPlanner] Obstacles removed: %zu", newly_cleared.size());

    // Handle removed obstacles
    ROS_INFO("Removing obstacles...");
    if (!newly_cleared.empty()) {
        for (unsigned int i: newly_cleared) removeObstacle(i);
        reduceInconsistency();
    }
    ROS_INFO("Completed.");

    // Handle added obstacles
    ROS_INFO("Adding obstacles...");
    if (!newly_blocked.empty()) {
        for (unsigned int i: newly_blocked) addObstacle(i);
        propogateDescendents();
        verifyQueue(start_proxy);
        reduceInconsistency();
    }
    ROS_INFO("Completed.");
}

/**
 * Removes obstacles
 * Makes necessary changes to ensure the newly freed path is used
 */
void MJERRTXPlanner::removeObstacle(unsigned int i) {
    // Remove from global tracking
    auto it = std::find(this->obstacles_.begin(), this->obstacles_.end(), i);
    if (it != this->obstacles_.end()) {
        *it = this->obstacles_.back();
        this->obstacles_.pop_back();
    }

    // Map back to world coordinates
    unsigned int obstacle_x = i % this->width_c_;
    unsigned int obstacle_y = i / this->width_c_;
    double center_x = this->origin_x + (obstacle_x + 0.5) * this->resolution_;
    double center_y = this->origin_y + (obstacle_y + 0.5) * this->resolution_;

    // KD Tree query
    std::vector<std::size_t> affected_nodes = this->kd_tree.radius_search(
        Point<double, 2>({center_x, center_y}), this->step_length_ + this->resolution_);

    // Restoration cascade
    for (std::size_t v_idx : affected_nodes) {
        Node& v = this->nodes_[v_idx];

        // Copy set to allow safe erasure during iteration
        std::vector<std::size_t> neighbors_to_check(v.blocked_nbrs.begin(), v.blocked_nbrs.end());

        for (std::size_t u_idx: neighbors_to_check) {
            if (v_idx > u_idx) continue;

            Node& u = this->nodes_[u_idx];

            // Verify the edge is clear against all remaining obstacles
            if (!hasObstacle(Point<double, 2>({v.state.x, v.state.y}), Point<double, 2>({u.state.x, u.state.y}))) {
                if (u_idx == 0) ROS_INFO("[MJERRTXPlanner] Root is other end of edge");

                v.blocked_nbrs.erase(u_idx);
                u.blocked_nbrs.erase(v_idx);

                updateLMC(v_idx);
                updateLMC(u_idx);

                if (std::abs(v.lmc - v.g) > this->epsilon_) verifyQueue(v_idx);
                if (std::abs(u.lmc - u.g) > this->epsilon_) verifyQueue(u_idx);
            }
        }
    }
}

/**
 * Adds cell with index i to the set of obstacles
 * Further processes and orphans the nodes affected
 */
void MJERRTXPlanner::addObstacle(unsigned int i) {
    this->obstacles_.push_back(i);

    unsigned int obstacle_x = i % this->width_c_;
    unsigned int obstacle_y = i / this->width_c_;

    // Get the points that make the grid cell
    double xmin = this->origin_x + obstacle_x * this->resolution_;
    double ymin = this->origin_y + obstacle_y * this->resolution_;
    double xmax = xmin + this->resolution_;
    double ymax = ymin + this->resolution_;

    auto process_neighbors = [&] (const auto& neighbors, Node& v, std::size_t v_idx) {
        for (std::size_t u_idx : neighbors) {
            if (v.blocked_nbrs.count(u_idx) > 0) continue;

            Node& u = this->nodes_[u_idx];

            if (isEdgeInCollision(
                    u.state.x, u.state.y, v.state.x, v.state.y,
                    xmin, ymin, xmax, ymax)) {

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
    std::vector<std::size_t> possible_invalidated_points = kd_tree.radius_search(Point<double, 2>({xmin, ymin}), this->step_length_ + this->resolution_);
    for (std::size_t v_idx: possible_invalidated_points) {
        Node& v = this->nodes_[v_idx];

        process_neighbors(v.nbr_running, v, v_idx);
        process_neighbors(v.nbr_init, v, v_idx);
    }
}

/**
 * Checks if a QBC can be made between the nodes
 * The QBC must follow the curvature cosntraints
 */
bool MJERRTXPlanner::isFeasible(State child_state, State par_state,
        double& Kmax, bool& singular) {
    double dx = par_state.x - child_state.x, dy = par_state.y - child_state.y;
    double chord_length = std::hypot(dx, dy);

    Kmax = std::numeric_limits<double>::infinity();
    if (chord_length < 1e-6) {
        singular = false;
        return false;
    }

    double chord_angle = std::atan2(dy, dx);
    double theta0 = bezier::normalizeAngle(par_state.theta - chord_angle);
    double theta5 = bezier::normalizeAngle(child_state.theta - chord_angle);

    if (std::abs(theta0) < 1e-2) theta0 = 0.0;

    std::pair<double, double> res = this->constraint_table_.getcurvature(theta0, theta5);
    double kappa = res.first;
    singular = res.second;

    if (singular) return false;
    else {
        Kmax = kappa / chord_length;
        if (Kmax > this->kc) return false;
    }

    return true;
}

/* Calculates arclength of QBC using lookup table */
double MJERRTXPlanner::getarclength(State child_state, State par_state) {
    double dx = par_state.x - child_state.x;
    double dy = par_state.y - child_state.y;
    double chord_length = std::hypot(dx, dy);

    if (chord_length < 1e-2) return 0.0;

    double chord_angle = std::atan2(dy, dx);
    double theta0 = bezier::normalizeAngle(child_state.theta - chord_angle);
    double theta5 = bezier::normalizeAngle(par_state.theta - chord_angle);

    return this->constraint_table_.getarclength(theta0, theta5) * chord_length;
}


/* Resets the tree when we encounter a new goal */
void MJERRTXPlanner::resetTree() {
    this->nodes_.clear();
    this->kd_tree.clear();
    this->orphan_set_.clear();
    this->queue_.clear();
    this->queueMap_.clear();
    this->obstacles_.clear();
}

/**
 * Scans the line from start to finish
 * Checks every cell, and returns if there is an obstacle on any of them
 */
bool MJERRTXPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_c_, sy = start / width_c_, gx = end % width_c_, gy = end / width_c_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= this->obstacle_cost_threshold_) return true;
    }
    return false;
}

/**
 * Checks if there is an obstacle between p1 and p2
 */
bool MJERRTXPlanner::hasObstacle(Point<double, 2> p1, Point<double, 2> p2) {
    return hasObstacle(p1[0], p1[1], p2[0], p2[1]);
}

/* Checks if there is an obstacle between (x1, y1) and (x2, y2) */
bool MJERRTXPlanner::hasObstacle(double x1, double y1, double x2, double y2) {
    unsigned int start, end;
    unsigned int mx, my;

    if (!costmap_->worldToMap(x1, y1, mx, my)) return true;
    start = costmap_->getIndex(mx, my);

    if (!costmap_->worldToMap(x2, y2, mx, my)) return true;
    end = costmap_->getIndex(mx, my);

    return hasObstacle(start, end);
}

double MJERRTXPlanner::getRadius() const {
    double n = static_cast<double>(nodes_.size());
    if (n <= 1) return step_length_;

    return std::min(rad_const_ * std::pow(std::log(1 + n) / n, 0.5), step_length_);
}

/**
 * Orphan all the nodes blocked by the obstacle
 * And all their descendents
 */
void MJERRTXPlanner::propogateDescendents() {
    std::vector<std::size_t> processing_queue(this->orphan_set_.begin(), this->orphan_set_.end());
    std::size_t head = 0;
    
    // Do a BFS and add all the children
    while (head < processing_queue.size()) {
        std::size_t v_idx = processing_queue[head++];
        for (std::size_t child_idx : this->nodes_[v_idx].children) {
            // std::unordered_set::insert returns a pair, .second is true if insertion took place
            if (this->orphan_set_.insert(child_idx).second) processing_queue.push_back(child_idx);
        }
    }

    // Invalidate neighboring nodes that rely on these orphans
    for (std::size_t v_idx : this->orphan_set_) {
        const Node& v = this->nodes_[v_idx];
        
        auto invalidate_neighbor = [&](std::size_t u_idx) {
            if (u_idx != 0 && u_idx != Node::INVALID_IDX && this->orphan_set_.count(u_idx) == 0) {
                if (u_idx == 0) ROS_WARN("[MJERRTXPlanner] Root is being added to the queue");
                this->nodes_[u_idx].g = std::numeric_limits<double>::infinity();
                verifyQueue(u_idx);
            }
        };

        for (std::size_t u_idx: v.nbr_running) invalidate_neighbor(u_idx);
        for (std::size_t u_idx: v.nbr_init) invalidate_neighbor(u_idx);
        invalidate_neighbor(v.par_idx);
    }

    // Sever all tree connections of the nodes in orphan set
    for (std::size_t v_idx: this->orphan_set_) {
        if (v_idx == 0) ROS_WARN("[MJERRTXPlanner] Root is in the orphan set");
        Node& v = this->nodes_[v_idx];
        v.g = std::numeric_limits<double>::infinity();
        v.lmc = std::numeric_limits<double>::infinity();

        if (v.par_idx != Node::INVALID_IDX) {
            this->nodes_[v.par_idx].removeChild(v_idx);
            v.par_idx = Node::INVALID_IDX;
        }
    }

    this->orphan_set_.clear();
}

/**
 * We don't usually have start node in our tree
 * So we use the next nearest unblocked node as a proxy for the start node
 */
std::size_t MJERRTXPlanner::findStartProxy() {
    // Get the nearest node
    // Can also use some other value, eg 2 * step_length_
    double search_radius = this->radius_;
    std::vector<std::size_t> local_neighbors =
        this->kd_tree.radius_search(this->start_, this->search_radius);

    double min_dist = std::numeric_limits<double>::infinity();
    std::size_t best_visible_proxy = Node::INVALID_IDX;

    for (std::size_t nbr_idx : local_neighbors) {
        Node& nbr = this->nodes_[nbr_idx];

        if (std::isinf(nbr.lmc)) continue;

        Point<double, 2> nbr_pt({nbr.state.x, nbr.state.y});
        // Check if we have a clear path to this neighbor
        // And if we improve the previous cost
        double dist = nbr.lmc + this->start_.distance(nbr_pt);

        if (dist <= min_dist && !hasObstacle(this->start_, nbr_pt)) {
            min_dist = dist;
            best_visible_proxy = nbr_idx;
        }
    }

    return best_visible_proxy;
}

/**
 * Add the node to orphan set
 * If in queue, remove from queue
 */
void MJERRTXPlanner::verifyOrphan(std::size_t node_idx) {
    // Check if the node is in the queue or not
    auto it = this->queueMap_.find(node_idx);
    if (it != this->queueMap_.end()) {
        this->queue_.erase(it->second);               // Remove from the queue
        this->nodes_[node_idx].in_queue = false;
        this->queueMap_.erase(it);                    // Remove from map
    }
    this->orphan_set_.insert(node_idx);
}

/* Update the key of the node in the queue */
void MJERRTXPlanner::verifyQueue(std::size_t node_idx) {
    if (node_idx == 0) {
        ROS_WARN("[MJERRTXPlanner] Root is being added to queue");
        return;
    }
    if (node_idx == Node::INVALID_IDX) return;

    QKey key = makeKey(node_idx);
    auto it = this->queueMap_.find(node_idx);
    if (it == this->queueMap_.end()) {
        this->queueMap_.insert({node_idx, key});
        this->queue_.insert(key);
        this->nodes_[node_idx].in_queue = true;
    } else {
        this->queue_.erase(it->second);
        this->queue_.insert(key);
        this->queueMap_[node_idx] = key;
    }
}

QKey MJERRTXPlanner::makeKey(std::size_t v_idx) {
    Node& v = this->nodes_[v_idx];
    
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
bool MJERRTXPlanner::keyLess(const QKey& a, const QKey& b) const {
    if (a.k1 != b.k1) return a.k1 < b.k1;
    return a.k2 < b.k2;
}

/**
 *  Returns true if the line segment from (x0, y0) to (x1, y1) intersects the bounding box
 */
bool MJERRTXPlanner::isEdgeInCollision(double x0, double y0, double x1, double y1, 
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
std::pair<double, double> MJERRTXPlanner::samplePoint() {
    std::size_t num_free_cells = this->free_cells.size();
    std::uniform_int_distribution<> distr(0, num_free_cells - 1);

    std::size_t random_cell = distr(rng);
    std::pair<unsigned int, unsigned int> point = this->free_cells[random_cell];

    unsigned int cell_x, cell_y;
    cell_x = point.first;
    cell_y = point.second;

    double wx, wy;
    this->costmap_->mapToWorld(cell_x, cell_y, wx, wy);

    wx += (this->rand01(rng) - 0.5) * this->resolution_;
    wy += (this->rand01(rng) - 0.5) * this->resolution_;

    return std::make_pair(wx, wy);
}

/* Stores the cells that are unoccupied */
void MJERRTXPlanner::buildFreeCellList() {
    this->free_cells.clear();

    for (unsigned int y = this->sampling_min_y_; y <= this->sampling_max_y_; y++) {
        for (unsigned int x = this->sampling_min_x_; x < this->sampling_max_x_; x++) {
            if (this->costmap_->getCost(x, y) < this->obstacle_cost_threshold_)
                this->free_cells.emplace_back(x, y);
        }
    }
}

/* Sorts the vector by distance to goal */
void MJERRTXPlanner::sortVector(std::vector<std::size_t>& points) {
    std::sort(points.begin(), points.end(),
            [] (const std::size_t& a, const std::size_t& b) {
                State& a_state = this->nodes[a].state;
                State& b_state = this->nodes[b].state;
                return utils::squared_dist<double>(a_state.x, a_state.y, this->goal_.x, this->goal_.y) <
                utils::squared_dist<double>(b_state.x, b_state.y, this->goal_.x, this->goal_.y);
            });
}

/**
 * Removes the child by swapping it with last element and popping
 * Works only if order of nodes does not matter
 */
void Node::removeChild(std::size_t child_idx) {
    auto it = std::find(this->children.begin(), this->children.end(), child_idx);

    if (it != this->children.end()) {
        *it = std::move(this->children.back());
        this->children.pop_back();
    }
}

} // namespace custom_nav
