#ifndef UTILS_H_
#define UTILS_H_

#ifdef __linux__
#include <geometry_msgs/Point.h>
#endif

namespace utils {

#ifdef __linux__
/**
 * Helper function to compute the path length of the generated path
 */
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
#endif

/* Generic function to calculate the square of euclidean distance between points */
template <typename T>
inline double squared_dist(T x1, T y1, T x2, T y2) {
    double dx = static_cast<double>(x1) - static_cast<double>(x2);
    double dy = static_cast<double>(y1) - static_cast<double>(y2);
    return dx * dx + dy * dy;
}

/* Generic function to calculate euclidean distance between points */
template <typename T>
inline double distance(T x1, T y1, T x2, T y2) {
    return std::sqrt(squared_dist(x1, y1, x2, y2));
}

}

#endif // UTILS_H_