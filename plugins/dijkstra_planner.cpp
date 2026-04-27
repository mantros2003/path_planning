#include <custom_nav/dijkstra_planner.h>
#include <pluginlib/class_list_macros.h>
#include <queue>

// Register as ROS plugin
PLUGINLIB_EXPORT_CLASS(
    custom_planner::DijkstraPlanner,
    nav_core::BaseGlobalPlanner
)

namespace custom_planner {

/**
 * Helper function to compute the path length of the generated path
 */
double computePathLengthD(
    const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (plan.size() < 2)
        return 0.0;

    double total_length = 0.0;

    for (std::size_t i = 1; i < plan.size(); ++i) {
        double dx = plan[i].pose.position.x -
                    plan[i-1].pose.position.x;

        double dy = plan[i].pose.position.y -
                    plan[i-1].pose.position.y;

        total_length += std::hypot(dx, dy);
    }

    return total_length;
}

void
DijkstraPlanner::initialize (std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        initialized_ = true;
        height_ = costmap_->getSizeInCellsY();
        width_ = costmap_->getSizeInCellsX();

        ROS_INFO("Initialized Dijkstra planner with map of size %d * %d", height_, width_);
    } else {
        ROS_WARN("This node has already been initialized...");
    }
}

bool DijkstraPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
) {
    ROS_INFO("Making path...");

    ros::Time start_time = ros::Time::now();
    
    unsigned int mx, my, gx, gy;

    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        ROS_ERROR("Goal or start is outside map boundaries");
        return false;   // Goal or start is outside map boundaries
    }

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    std::vector<double> dist(height_ * width_, 1e9);
    std::vector<int> parent(dist.size(), -1);

    int start_index = costmap_->getIndex(mx, my);
    int goal_index = costmap_->getIndex(gx, gy);

    ROS_INFO("Start index: %d, Goal index: %d", start_index, goal_index);

    dist[start_index] = 0;
    pq.push({0, start_index});

    int curr, ux, uy, nx, ny, count = 0;
    while (!pq.empty()) {
        curr = pq.top().second;
        pq.pop();

        if (goal_index == curr) {
            ROS_INFO("Found goal node");
            break;
        }

        ux = curr % width_, uy = curr / width_; count++;
        if (count % 1000 == 0) ROS_INFO("Exploring %dth node", count);
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                nx = ux + dx, ny = uy + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny > height_) continue;

                unsigned char cost = costmap_->getCost(nx, ny);
                if (cost >= costmap_2d::LETHAL_OBSTACLE) continue;

                int v = costmap_->getIndex(nx, ny);
                double weight = (dx == 0 || dy == 0) ? 1.0: 1.414;
                double obstacle_penalty = cost / 50.0;
                double total_weight = weight + obstacle_penalty;
                if (dist[curr] + total_weight < dist[v]) {
                    dist[v] = dist[curr] + total_weight;
                    parent[v] = curr;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    if (pq.empty()) ROS_INFO("Finally pq is empty");

    // Tracing back from the goal to the start
    if (parent[goal_index] == -1) return false;
    for (curr = goal_index; curr != -1; curr = parent[curr]) {
        unsigned int cx = curr % width_, cy = curr / width_;
        double wx, wy;
        costmap_->mapToWorld(cx, cy, wx, wy);

        geometry_msgs::PoseStamped p = goal;
        p.pose.position.x = wx;
        p.pose.position.y = wy;

        plan.insert(plan.begin(), p);
    }

    ROS_INFO("Made a path with %ld points", plan.size());
    double path_len = computePathLengthD(plan);
    ROS_INFO("[RRTXPlanner] Successfully extracted path with %zu waypoints,\tPath length: %.3f m,\tPlanning time: %.3f ms", plan.size(), path_len, (ros::Time::now() - start_time).toSec() * 1000);

    return true;
}

} // namespace custom_planner
