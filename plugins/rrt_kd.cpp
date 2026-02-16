#include <custom_nav/rrt.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>

// Assuming your kd_tree.h defines 'point' as std::pair<int, int>
// and the kdTree class is accessible.

#define INF 1e9

PLUGINLIB_EXPORT_CLASS(custom_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace custom_planner {

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
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

bool RRTPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    ros::Time start_time = ros::Time::now();

    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        ROS_ERROR("Goal or start is outside map boundaries");
        return false;
    }

    int start_index = costmap_->getIndex(sx, sy);
    int goal_index = costmap_->getIndex(gx, gy);

    // Hyperparameters
    unsigned int max_iters = 100000;
    double goal_bias = 0.1;
    double step = 5.0;
    double goal_tolerance = 5.0;
    bool goal_reached = false;

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> uniform_dist(0, height_ * width_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    // --- KD-TREE INTEGRATION ---
    kdTree tree; 
    std::vector<int> parents(height_ * width_, -1);

    // Initialize the root in the tree
    point start_pt = { (int)sx, (int)sy };
    tree.insert(start_pt);
    parents[start_index] = start_index;

    for (int iters = 0; iters < max_iters; iters++) {
        unsigned int rand_index;
        if (rand01(rng) < goal_bias) {
            rand_index = goal_index;
        } else {
            rand_index = uniform_dist(rng);
        }

        // Convert random index to point for k-d Tree search
        point rand_pt = { (int)(rand_index % width_), (int)(rand_index / width_) };

        // 1. Find Nearest using k-d Tree (O(log n))
        point near_pt = tree.nearest(rand_pt);
        unsigned int near_index = costmap_->getIndex(near_pt.first, near_pt.second);

        // 2. Steer from near towards rand
        unsigned int new_index = steer(near_index, rand_index, step);

        // 3. Collision and Duplicate Check
        if (new_index == near_index || parents[new_index] != -1 || hasObstacle(near_index, new_index)) {
            continue;
        }

        // 4. Add to tree
        point new_pt = { (int)(new_index % width_), (int)(new_index / width_) };
        tree.insert(new_pt);
        parents[new_index] = near_index;
        
        // Check Goal
        if (new_index == goal_index) {
            goal_reached = true;
            break;
        }

        double dist_to_goal = std::sqrt(std::pow((int)(new_index % width_) - (int)gx, 2) + 
                                       std::pow((int)(new_index / width_) - (int)gy, 2));
        
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

bool RRTPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_, sy = start / width_, gx = end % width_, gy = end / width_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) return true;
    }
    return false;
}

unsigned int RRTPlanner::steer(unsigned int from, unsigned int to, double step) {
    int sx = from % width_, sy = from / width_, gx = to % width_, gy = to / width_;
    double dx = gx - sx;
    double dy = gy - sy;
    double len = std::sqrt(dx*dx + dy*dy);

    if (len <= step) return to;

    int nx = sx + static_cast<int>(std::round(step * dx / len));
    int ny = sy + static_cast<int>(std::round(step * dy / len));

    return costmap_->getIndex(std::max(0, std::min((int)width_-1, nx)), 
                               std::max(0, std::min((int)height_-1, ny)));
}

} // namespace custom_planner