#ifndef CUSTOM_NAV_RRT_STAR_PLANNER_H
#define CUSTOM_NAV_RRT_STAR_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

namespace custom_planner {

class RRTStarPlanner : public nav_core::BaseGlobalPlanner
{
    public:
        RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}
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
        int height_, width_;
};

} // namespace custom_planner

#endif //CUSTOM_NAV_RRT_STAR_PLANNER_H