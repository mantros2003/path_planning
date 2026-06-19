#ifndef UTILS_H_
#define UTILS_H_

#ifdef __linux__
#include <geometry_msgs/PoseStamped.h>
#endif
#include <vector>
#include <cmath>

namespace utils {

#ifdef __linux__
/**
 * Helper function to compute the path length of the generated path
 */
double computePathLength(const std::vector<geometry_msgs::PoseStamped>&);
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
