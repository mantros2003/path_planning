#include <custom_nav/rrt_x.h>
#include <custom_nav/kd_tree.h>
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
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

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
    ROS_INFO("Making path using RRTX");

    ros::Time start_time = ros::Time::now();

    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;

    ROS_INFO("Start position: (%f, %f), Goal position: (%f, %f)", start_index, goal_index);

    // Initialize hyperparams
    // Moved hyper params to the .h file
    bool goal_reached = false;

    // Initialize random number generator
    // Moved to .h file as class members
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> uniform_dist(0, height_ * width_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    kdTree tree;
    point goal_pt = { (int) gx, (int) gy };
    tree.insert(goal);
}

// NOTE: Make this function more modular and
// remove the hardcoded 0.5 and add 1/dim logic
double RRTXPlanner::getRadius() {
    double n = static_cast<double>(nodes_.size());
    if (n <= 1) return step_len;

    return std::min(rad_const_ * std::pow(std::log(n) / n, 0.5), step_length)
}

}