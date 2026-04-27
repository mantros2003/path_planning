#include <custom_nav/a_star.h>
#include <pluginlib/class_list_macros.h>
#include <queue>
#include <cmath>

#define INF 1e9

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::AStarPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

/**
 * Helper function to compute the path length of the generated path
 */
// double computePathLength(
//     const std::vector<geometry_msgs::PoseStamped>& plan)
// {
//     if (plan.size() < 2)
//         return 0.0;

//     double total_length = 0.0;

//     for (std::size_t i = 1; i < plan.size(); ++i) {
//         double dx = plan[i].pose.position.x -
//                     plan[i-1].pose.position.x;

//         double dy = plan[i].pose.position.y -
//                     plan[i-1].pose.position.y;

//         total_length += std::hypot(dx, dy);
//     }

//     return total_length;
// }

// NOTE: Convert all distances to `double`

inline double heuristic(int sx, int sy, int gx, int gy) {
    return std::sqrt((sx - gx)*(sx - gx) + (sy - gy)*(sy - gy));
}

// Function to initialize and store the costmap info once.
void AStarPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

        ROS_INFO("Initialized A* planner with map of size %d * %d", height_, width_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

// Main function that plans the trajectory
bool AStarPlanner::makePlan(
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

    // Containers to store data reagrding to the search and path
    std::vector<int> parent(height_ * width_, -1);
    std::vector<double> dist(parent.size(), INF);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_list;

    dist[start_index] = 0;
    open_list.push({0, start_index});

    unsigned int ux, uy, count = 0;
    int nx, ny, curr;
    while (!open_list.empty()) {
        curr = open_list.top().second;
        open_list.pop();

        // Search finishes when we find the goal
        if (curr == goal_index) {
            ROS_INFO("Goal found");
            break;
        }

        // Get map coordinates of curr
        ux = curr % width_, uy = curr / width_; count++;

        // if (count % 1000 == 0) ROS_INFO("Exploring %dth node", c ount);

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                nx = ux + dx, ny = uy + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny > height_) continue;

                unsigned char cost = costmap_->getCost(nx, ny);
                if (cost >= costmap_2d::LETHAL_OBSTACLE) continue;

                int v = costmap_->getIndex(nx, ny);
                double weight = (dx == 0 || dy == 0) ? 1.0: 1.414;
                double obstacle_penalty = cost;
                double h_cost = heuristic(nx, ny, gx, gy);
                double total_weight = weight + obstacle_penalty + h_cost;
                if (dist[curr] + total_weight < dist[v]) {
                    dist[v] = dist[curr] + total_weight;
                    parent[v] = curr;
                    open_list.push({dist[v], v});
                }
            }
        }
    }

    ROS_INFO("Searched %u nodes", count);
    if (open_list.empty()) ROS_INFO("Set is empty");

    // If parent of the goal is -1, then we failed to reach the goal
    if (parent[goal_index] == -1) return false;

    // Tracing back from goal to start
    for (curr = goal_index; curr != -1; curr = parent[curr]) {
        unsigned int cx = curr % width_, cy = curr / width_;
        double wx, wy;
        costmap_->mapToWorld(cx, cy, wx, wy);

        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx;
        p.pose.position.y = wy;

        plan.insert(plan.begin(), p);
    }

    ROS_INFO("Made a path of %ld points", plan.size());
    double path_len = computePathLength(plan);
    ROS_INFO("[RRTXPlanner] Successfully extracted path with %zu waypoints,\tPath length: %.3f m,\tPlanning time: %.3f ms", plan.size(), path_len, (ros::Time::now() - start_time).toSec() * 1000);

    return true;
}

} // namespace custom_planner
