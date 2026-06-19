#include <custom_nav/utils.h>

namespace utils {

double computePathLength(
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

}
