#include <custom_nav/rrt_connect.h>
#include <custom_nav/kd_tree.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>

#define INF 1e9

PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTConnectPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

void RRTConnectPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

        ROS_INFO("Initialized RRT-Connect planner with map of size %d * %d", height_, width_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

bool RRTConnectPlanner::makePlan(
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

    // Get start and goal indices
    int start_index = costmap_->getIndex(sx, sy);
    int goal_index = costmap_->getIndex(gx, gy);

    ROS_INFO("Start index: %d, Goal index: %d", start_index, goal_index);

    // Hyperparameters
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

    // Initialize trees and parents
    kdTree tree_start, tree_goal;
    std::vector<int> parents_start(height_ * width_, -1), parents_goal(height_ * width_, -1);

    // Initialize the root in the two tree
    point start_pt = { (int) sx, (int) sy };
    point goal_pt = { (int) gx, (int) gy };
    tree_start.insert(start_pt);
    tree_goal.insert(goal_pt);
    parents_start[start_index] = start_index;
    parents_goal[goal_index] = goal_index;

    // Make pointers to simplify swapping
    // Swap these variables after every iteration
    kdTree *treeA = &tree_start;
    kdTree *treeB = &tree_goal;
    std::vector<int> *parentsA = &parents_start;
    std::vector<int> *parentsB = &parents_goal;

    for (int iters = 0; iters < max_iters; iters++) {
        unsigned int rand_index;

        rand_index = (rand01(rng) < goal_bias) ? goal_index : uniform_dist(rng);

        // If node already explored, continue
        if (parents[rand_index] != -1 || rand_index == start_index) continue;

        // Convert random index to point for k-d Tree search
        point rand_pt = { (int)(rand_index % width_), (int)(rand_index / width_) };

        // Find the nearest neighbour to the selected node
        // Implemented using kd Tree
        point near_pt = treeA->nearest(rand_pt);
        unsigned int near_index = costmap_->getIndex(near_pt.first, near_pt.second);

        // Steer from near towards rand
        unsigned int new_index = steer(near_index, rand_index, step);

        // Collision and Duplicate check
        if (new_index == near_index || parentsA[new_index] != -1 || hasObstacle(near_index, new_index)) continue;

        // Add to tree
        point new_pt = { (int)(new_index % width_), (int)(new_index / width_) };
        treeA->insert(new_pt);
        (*parentsA)[new_index] = near_index;

        // Try to connect Tree B to the new node in Tree A
        point target_B = {(int)(new_index % width_), (int)(new_index / width_)};
        unsigned int target_index_B = new_index;

        // 
        while (true) {
        }

        // Check Goal
        if (new_index == goal_index) {
            goal_reached = true;
            break;
        }

        double dist_to_goal = distance(new_index % width_, new_index / width_, (int) gx, (int) gy);
        
        if (dist_to_goal < goal_tolerance) {
            parents[goal_index] = new_index;
            goal_reached = true;
            break;
        }
    }

    if (!goal_reached) {
        ROS_WARN("RRT failed to find a path");
        return false;
    }

    // Path reconstruction
    plan.clear();
    unsigned int curr = goal_index;
    while (true) {
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

    ROS_INFO("Path found! Time: %.4fs, Nodes: %ld", (ros::Time::now() - start_time).toSec(), plan.size());
    return true;
}

bool RRTConnectPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_, sy = start / width_, gx = end % width_, gy = end / width_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) return true;
    }
    return false;
}

unsigned int RRTConnectPlanner::steer(unsigned int from, unsigned int to, double step) {
    int sx = from % width_, sy = from / width_, gx = to % width_, gy = to / width_;
    double dx = gx - sx;
    double dy = gy - sy;
    double len = distance(gx, gy, sx, sy);

    if (len <= step) return to;

    int nx = sx + static_cast<int>(std::round(step * dx / len));
    int ny = sy + static_cast<int>(std::round(step * dy / len));

    return costmap_->getIndex(std::max(0, std::min((int)width_-1, nx)), 
                               std::max(0, std::min((int)height_-1, ny)));
}

} // namespace custom_planner