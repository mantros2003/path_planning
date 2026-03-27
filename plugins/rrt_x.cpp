#include <custom_nav/rrt_x.h>
#include <custom_nav/kd_tree_x.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>

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
        height_ = costmap_->getSizeInMetersY();
        width_ = costmap_->getSizeInMetersX();
        origin_x = costmap_->getOriginX();
        origin_y = costmap_->getOriginY();

        ROS_INFO("Initialized RRTX planner with map of size %d * %d", height_, width_);
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

    
    ROS_INFO("Start position: (%f, %f), Goal position: (%f, %f)", start_index, goal_index);
    
    // Check if the goal has changed and we need a new tree
    if (isNewGoal(gx, gy)) {
        resetTree();
        
        Node root_node(goal.pose.position.x, goal.pose.position.y);
        root_node.g = 0.0;     // Goal cost is 0
        root_node.lmc = 0.0;
        nodes_.push_back(root_node);
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
    while (!isConnected((sx, sy)) && iters < max_iters_) {
        double x = origin_x + (rand01(rng) * width_);
        double y = origin_y + (rand01(rng) * height_);

        int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my)) continue;
        if (costmap_->getCost(mx, my) >= 254) continue;

        Point<double, 2> random_pt({x, y});

        // Get the point nearest to the ranomly selected point
        std::size_t nearest_index = kd_tree.nearest(random_pt);
        Node& near_node = nodes_[nearest_index];

        // Move in the direction of the random point from the near point
        Point<double, 2> new_pt = steer(near_pt, random_pt);

        if (!costmap_->worldToMap(new_pt[0], new_pt[1], mx, my)) continue;
        if (costmap_->getCost(mx, my) >= 254) continue;
        
        // Check if the path is free of obstacles
        if (hasObstacle(near_pt, new_pt)) continue;

        // If not an obstacle, then add the point to the tree
        Node new_node(new_pt[0], new_pt[1]);
        std::size_t new_idx = nodes_.size(); // It will be inserted at this index


        // Find neighbors within radius
        std::vector<std::size_t> neighbors = kd_tree_.radius_search(new_pt, radius_);

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
        kd_tree_.insert(new_pt, new_idx);
        nodes_[best_parent].children.push_back(new_idx);

        radius_ = getRadius();

        rewireNeighbors(new_idx);
        reduceInconsistency();

        iters++;
    }
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
            updateLMC();
            rewireNeighbors();
        }

        v.g = v.lmc;
        
        // Update botKey just in case the proxy's cost changed during the loop
        botKey = makeKey(start_proxy); 
    }
}

void RRTXPlanner::rewireNeighbors(std::size_t v_index) {
    Node& v = nodes_[v_index];
    Point<double, 2> v_pt = steer(near_pt, random_pt);

    if (v.g - v.lmc <= epsilon_) return;

    cullNeighbors(v_index);

    // Check all the neighbors
    for (std::size_t nbr_index: v.nbr_init) {
        if (nbr_index == par_idx) continue;

        Node& nbr = nodes_[nbr_index];
        Point<double, 2> nbr_pt({nbr.x, nbr.y});
        double nbr_dist_via_v = v.lmc + v_pt.distance(nbr_pt);

        // If going through v is better
        if (nbr_dist_via_v < nbr.lmc) {
            // Remove from parent's children list
            if (nbr.par_idx != Node::INVALID_IDX) {
                Node& old_parent = nodes_[nbr.par_idx];
                old_parent.children.erase(
                    std::remove(old_parent.children.begin(), old_parent.children.end(), nbr_idx),
                    old_parent.children.end()
                );
            }

            // Link to new parent
            nbr.par_idx = node_idx;
            node.children.push_back(nbr_idx);

            // Update lmc
            nbr.lmc = nbr_dist_via_v;

            if (nbr.u - nbr.lmc > epsilon_) verifyQueue(nbr_index);
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
                
                // Calculate Euclidean distance
                double dx = node.x - nbr.x;
                double dy = node.y - nbr.y;
                double dist = std::hypot(dx, dy);

                // If the neighbor is now outside the shrinking radius
                if (dist > radius_) {
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

void RRTXPlanner::reduceInconsistency() {
    std::vector<std::size_t> start_neighbors = kd_tree_.radius_search(start_, start_dist_threshold);
    if (start_neighbors.empty()) return;

    std::size_t closest_;

    // Add more conditions
    while (queue_.size() > 0) {
        std::size_t top_index = queue_.pop();
        Node& top = nodes_[top_index];
        top.in_queue = false;

        if (top.g - top.lmc > epsilon_) {
            updateLMC();
            rewireNeighbors();
        }
    }
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

void RRTXPlanner::updateLMC(std::size_t v_index) {
    cullNeighbors(v_index); 
    
    Node& v = nodes_[v_index];
    for (std::size_t nbr_index: v.nbr_running) {
        Node& nbr = nodes_[nbr_index];
    }
}

void RRTXPlanner::updateObstacles() {
    ;
}

Point<double, 2> RRTXPlanner::steer(double near_x, double near_y, double rand_x, double rand_y) {
    double new_x, new_y;

    double dx = rand_x - near_x;
    double dy = rand_y - near_y;
    double len = distance<double>(near_x, near_y, rand_x, rand_y);

    if (len <= step_length_) return Point<double, 2>({rand_x, rand_y});

    new_x = near_x + (step_length_ * dx / len);
    new_y = near_y + (step_length_ * dy / len);

    return Point<double, 2>({new_x, new_y});
}

// Checks if the goal is changed
bool RRTXPlanner::isNewGoal(double gx, double gy) {
    return (gx == goal[0]) && (gy == goal[1]);
}

/**
 * Resets the tree when we encounter a new goal
 */
void RRTXPlanner::resetTree() {}

/**
 * Checks if point `p` is connected to the tree
 * Searches for points within `step_length_` of the start and chooses the best possible option
 */
bool RRTXPlanner::isConnected(double sx, double sy) {
    if (nodes_.empty()) return false;

    Point<double, 2> start_pt({sx, sy});

    std::vector<std::size_t> neighbors = kd_tree.radius_search(p, step_length_);

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

bool RRTKDPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_, sy = start / width_, gx = end % width_, gy = end / width_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) return true;
    }
    return false;
}

// NOTE: Make this function more modular and
// remove the hardcoded 0.5 and add 1/dim logic
double RRTXPlanner::getRadius() const {
    double n = static_cast<double>(nodes_.size());
    if (n <= 1) return step_len;

    return std::min(rad_const_ * std::pow(std::log(n) / n, 0.5), step_length)
}

std::size_t RRTXPlanner::findStartProxy() {
    // Get the nearest node
    std::size_t proxy_idx = kd_tree_.nearest(robot_pt);
    
    // If the tree is empty or invalid, return the invalid index
    if (proxy_idx == Node::INVALID_IDX || proxy_idx >= nodes_.size()) {
        return Node::INVALID_IDX;
    }

    Node& nearest_node = nodes_[proxy_idx];
    Point<double, 2> proxy_pt({nearest_node.x, nearest_node.y});

    // If there's no obstacle between the robot and this node
    if (!hasObstacle(robot_pt, proxy_pt)) {
        return proxy_idx;
    }

    // We must search a local radius for the closest node that we can actually see
    // We can change search radius to a multiple of step length
    double search_radius = step_length_;
    std::vector<std::size_t> local_neighbors = kd_tree_.radius_search(robot_pt, search_radius);

    double min_dist = std::numeric_limits<double>::infinity();
    std::size_t best_visible_proxy = proxy_idx;

    for (std::size_t nbr_idx : local_neighbors) {
        Node& nbr = nodes_[nbr_idx];
        Point<double, 2> nbr_pt({nbr.x, nbr.y});

        // Check if we have a clear path to this neighbor
        if (!hasObstacle(robot_pt, nbr_pt)) {
            double dist = robot_pt.distance(nbr_pt);
            if (dist < min_dist) {
                min_dist = dist;
                best_visible_proxy = nbr_idx;
            }
        }
    }

    return best_visible_proxy;
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

}