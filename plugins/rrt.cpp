#include <custom_nav/rrt.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>

#define INF 1e9

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTPlanner,
    nav_core::BaseGlobalPlanner
)

inline double squared_distance(int x1, int y1, int x2, int y2) {
    return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}

inline double distance(int x1, int y1, int x2, int y2) {
    return std::sqrt(squared_distance(x1, y1, x2, y2));
}

namespace custom_planner {

// Function to initialize and store the costmap info once.
void RRTPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

        ROS_INFO("Initialized RRT planner with map of size %d * %d", height_, width_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

// Main function that plans the trajectory
bool RRTPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    ROS_INFO("Making path");

    ros::Time start_time = ros::Time::now();

    // The start and goal x, y coordinates
    unsigned int sx, sy, gx, gy;

    // Get map coordinates
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        ROS_ERROR("Goal or start is outside map boundaries");
        return false;
    }

    int start_index = costmap_->getIndex(sx, sy);
    int goal_index = costmap_->getIndex(gx, gy);

    ROS_INFO("Start index: %d, Goal index: %d", start_index, goal_index);

    // Initialize hyperparams
    unsigned int max_iters = 100000;
    double goal_bias = 0.1;
    double step = 5.0;
    double goal_tolerance = 5.0;
    bool goal_reached = false;

    // Initialize random number generator
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> uniform_dist(0, height_ * width_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    // Containers
    std::vector<unsigned int> vertices;
    std::vector<int> parents(height_ * width_, -1);

    // Initialize the root
    unsigned int root = start_index;
    vertices.push_back(root);
    parents[root] = root;

    plan.clear();

    unsigned int rand_index, near_index, new_index, cnt;
    // Main loop
    for (int iters = 0; iters < max_iters; iters++) {
        // Randomly sample points from the map
        // With probability goal_bias, select goal node
        rand_index = (rand01(rng) < goal_bias) ? goal_index : uniform_dist(rng);

        // If node already explored, continue
        if (parents[rand_index] != -1 || rand_index == root) continue;

        // Find the nearest neighbour to the selected node
        // NOTE: Implement the nearest neighbour funciton using kd Tree
        near_index = findNearestNeighbour(rand_index, vertices);

        // Get the new point that is obtained by moving step distance from near_index towards rand_index
        new_index = steer(near_index, rand_index, step);

        // Check if there is an obstacle, or new_index is already explored
        if (new_index == near_index || hasObstacle(near_index, new_index) || parents[new_index] != -1 || new_index == root) continue;

        vertices.push_back(new_index);
        parents[new_index] = near_index;
        
        if (new_index == goal_index) break;

        if (distance(new_index % width_, new_index / width_, gx, gy) < goal_tolerance) {
            parents[goal_index] = new_index;
            goal_reached = true;
            break;
        }
    }

    if (!goal_reached) {
        ROS_WARN("Failed to make a path");
        return false;
    }

    // cnt = 0;

    // Path reconstruction
    unsigned int curr = goal_index;
    while (true) {
        // For debugging
        // cnt++;
        // if (cnt % 100000 == 0) ROS_INFO("Looped %u time\tcurr: %u, par: %u", cnt, curr, parents[curr]);
        unsigned int cx = curr % width_, cy = curr / width_;
        double wx, wy;
        costmap_->mapToWorld(cx, cy, wx, wy);

        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx;
        p.pose.position.y = wy;

        plan.push_back(p);
        
        if (curr == (unsigned int)start_index) break;

        curr = parents[curr];
    }
    std::reverse(plan.begin(), plan.end());

    double time = (ros::Time::now() - start_time).toSec();

    ROS_INFO("Made a path of %ld points, took %.4fs", plan.size(), time);

    return true;
}

/**
 * Function to find the nearest node already present in the tree.
 */
unsigned int RRTPlanner::findNearestNeighbour(unsigned int node, std::vector<unsigned int>& vertices) {
    unsigned int nearest = vertices[0];
    double min_dist = std::numeric_limits<double>::max(), dist;

    unsigned int x, nx, y, ny;
    x = node % width_, y = node / width_;

    // Iterate over all the vertices and slect the best
    for (unsigned int nbr: vertices) {
        nx = nbr % width_, ny = nbr / width_;
        dist = squared_distance(nx, ny, x, y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = nbr;
        }
    }

    return nearest;
}

/**
 * Utility function for checking if path from start to end has an obstacle.
 */
bool RRTPlanner::hasObstacle(unsigned int start, unsigned int end) {
    // Get coordinates from indices
    int sx = start % width_, sy = start / width_, gx = end % width_, gy = end / width_;

    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) {
            return true; // Collision detected
        }
    }
    return false; // Path is clear
}

/**
 * Utility function to compute new node of the tree
 * Moves step distance in the direction of 'to'
 */
unsigned int RRTPlanner::steer(unsigned int from, unsigned int to, double step) {
    unsigned int sx = from % width_, sy = from / width_, gx = to % width_, gy = to / width_;

    double x_new, y_new, len;
    len = distance(sx, sy, gx, gy);

    if (len <= step) return to;

    int nx = sx + static_cast<int>(std::round(step * (gx - sx) / len));
    int ny = sy + static_cast<int>(std::round(step * (gy - sy) / len));

    // Ensure we don't return an index outside the map
    return costmap_->getIndex(std::max(0, std::min((int)width_-1, nx)), 
                               std::max(0, std::min((int)height_-1, ny)));
}

} // namespace custom_planner
