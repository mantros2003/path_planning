#include <custom_nav/rrt_star.h>
#include <custom_nav/kd_tree.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/line_iterator.h>
#include <cstdint>
#include <random>
#include <cmath>

#define INF 1e9

PLUGINLIB_EXPORT_CLASS(
    custom_planner::RRTStarPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

        ROS_INFO("Initialized RRT* planner with map of size %d * %d", height_, width_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

bool RRTStarPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    ROS_INFO("Making RRT* path");

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
    unsigned int max_iters = 20000; // Lowered slightly since RRT* per-iteration is heavier
    double goal_bias = 0.1;
    double step = 5.0;
    double search_radius = 12.0; // RRT* neighborhood radius
    double goal_tolerance = 5.0;

    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<unsigned int> uniform_dist(0, height_ * width_ - 1);
    std::uniform_real_distribution<> rand01(0.0, 1.0);

    kdTree tree;
    std::vector<int> parents(height_ * width_, -1);
    std::vector<double> costs(height_ * width_, INF); // Track costs from start

    point start_pt = { (int) sx, (int) sy };
    tree.insert(start_pt);
    parents[start_index] = start_index;
    costs[start_index] = 0.0;

    double best_goal_cost = INF;
    int best_goal_parent = -1;
    bool goal_reached = false;

    for (int iters = 0; iters < max_iters; iters++) {
        unsigned int rand_index;
        if (rand01(rng) < goal_bias) {
            rand_index = goal_index;
        } else {
            rand_index = uniform_dist(rng);
        }

        // Steer
        point rand_pt = { (int)(rand_index % width_), (int)(rand_index / width_) };
        point near_pt = tree.nearest(rand_pt);
        unsigned int near_index = costmap_->getIndex(near_pt.first, near_pt.second);
        unsigned int new_index = steer(near_index, rand_index, step);

        // Grid nodes can only be occupied once, if it's already in the tree, skip
        if (new_index == near_index || parents[new_index] != -1 || hasObstacle(near_index, new_index)) continue;

        point new_pt = { (int)(new_index % width_), (int)(new_index / width_) };
        std::vector<point> neighbors = tree.radius_search(new_pt, search_radius);

        // 1. RRT* Choose Best Parent
        unsigned int best_parent = near_index;
        double min_cost = costs[near_index] + distance(near_index % width_, near_index / width_, new_index % width_, new_index / width_);

        for (const auto& n_pt : neighbors) {
            unsigned int n_idx = costmap_->getIndex(n_pt.first, n_pt.second);
            double dist_to_new = distance(n_idx % width_, n_idx / width_, new_index % width_, new_index / width_);
            double cost = costs[n_idx] + dist_to_new;
            
            if (cost < min_cost && !hasObstacle(n_idx, new_index)) {
                min_cost = cost;
                best_parent = n_idx;
            }
        }

        // Add to tree
        tree.insert(new_pt);
        parents[new_index] = best_parent;
        costs[new_index] = min_cost;
        
        // 2. RRT* Rewire
        for (const auto& n_pt : neighbors) {
            unsigned int n_idx = costmap_->getIndex(n_pt.first, n_pt.second);
            if (n_idx == best_parent) continue; 

            double dist_to_n = distance(new_index % width_, new_index / width_, n_idx % width_, n_idx / width_);
            double rewired_cost = costs[new_index] + dist_to_n;
            
            if (rewired_cost < costs[n_idx] && !hasObstacle(new_index, n_idx)) {
                parents[n_idx] = new_index;
                costs[n_idx] = rewired_cost;
                // Note: In a fully strict RRT*, you would propagate this updated 
                // cost to all descendants of n_idx here.
            }
        }

        // Check Goal
        double dist_to_goal = distance(new_index % width_, new_index / width_, (int) gx, (int) gy);
        if (dist_to_goal < goal_tolerance) {
            double goal_cost = costs[new_index] + dist_to_goal;
            if (goal_cost < best_goal_cost && !hasObstacle(new_index, goal_index)) {
                best_goal_cost = goal_cost;
                best_goal_parent = new_index;
                goal_reached = true;
            }
        }
    }

    if (!goal_reached) {
        ROS_WARN("RRT* failed to find a path");
        return false;
    }

    // Path reconstruction
    parents[goal_index] = best_goal_parent;
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

    ROS_INFO("Optimal path found! Time: %.4fs, Nodes: %ld, Cost: %.2f", 
             (ros::Time::now() - start_time).toSec(), plan.size(), best_goal_cost);
    return true;
}

bool RRTStarPlanner::hasObstacle(unsigned int start, unsigned int end) {
    int sx = start % width_, sy = start / width_, gx = end % width_, gy = end / width_;
    for (base_local_planner::LineIterator line(sx, sy, gx, gy); line.isValid(); line.advance()) {
        if (costmap_->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE) return true;
    }
    return false;
}

unsigned int RRTStarPlanner::steer(unsigned int from, unsigned int to, double step) {
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