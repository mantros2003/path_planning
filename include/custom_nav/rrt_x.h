#ifndef CUSTOM_NAV_RRTX_PLANNER_H
#define CUSTOM_NAV_RRTX_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <random>

namespace custom_planner {

struct Node {
    double x, y;
    unsigned int par_idx;

    double g;       // Current estimate of cost-to-goal
    double lmc;     // Look ahead cost-to-goal

    std::unordered_set<Node> nbr_init_in, nbr_init_out;            // Initial neighbors
    std::unordered_set<Node> nbr_running_in, nbr_running_out;      // Neighbors within r(|V|)
    std::unordered_set<Node> children;
}

class RRTXPlanner : public nav_core::BaseGlobalPlanner
{
    public:
        RRTXPlanner() : costmap_(nullptr), initialized_(false), root =  {}
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(
            const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan
        );
        bool hasObstacle(unsigned int start, unsigned int end);
        unsigned int steer(unsigned int, unsigned int, double);

    private:
        costmap_2d::Costmap2D* costmap_;
        bool initialized_;
        bool planned_;
        int height_, width_;

        // Tree and node containers
        struct Node root;                   // Indicates the root of the tree which is the goal
        std::vector<struct Node> nodes_;    // Stores all the nodes
        std::unordered_set<> orphan_set;    // All the nodes which are cut-off from the main  tra

        // Hyperparams
        double rad_const_;                  // Constant used in radius
        double step_length_;                // Edge length of a tree edge
        double goal_tolerance_;
        double epsilon_;
        unsigned int max_iters_;

        // Ranom number generators
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_real_distribution<> rand01(0.0, 1.0);

        // Map to map points to nodes
        std::unordered_map<Point>
};

} // namespace custom_planner

#endif //CUSTOM_NAV_RRTX_PLANNER_H