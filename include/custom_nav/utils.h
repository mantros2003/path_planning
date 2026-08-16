#ifndef UTILS_H_
#define UTILS_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <boost/stacktrace.hpp>
#include <algorithm>
#include <vector>
#include <cmath>
#include <custom_nav/kd_tree_x.h>

namespace utils {

// Declarations
double computePathLength(const std::vector<geometry_msgs::PoseStamped>&);
inline void print_stack_trace_and_abort();
inline void print_stack_trace_and_abort_boost();
bool loadParamString(ros::NodeHandle& nh, std::string& param, std::string& output);

// Implemenrarions
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

// Class implementations
/* Thin wrapper over std::vector that checks against OOB access */
template <typename T, typename Allocator = std::allocator<T>>
class safeVec : public std::vector<T, Allocator> {
public:
    using std::vector<T, Allocator>::vector;

    typename std::vector<T, Allocator>::reference
    operator[] (typename std::vector<T, Allocator>::size_type n) {
        if (n >= this->size()) {
            fprintf(stderr, "writing oob at index: %lu in a vector of size: %lu", n, this->size());
            // print_stack_trace_and_abort_boost();
        }
        return std::vector<T, Allocator>::operator[](n);
    }

    typename std::vector<T, Allocator>::const_reference
    operator[] (typename std::vector<T, Allocator>::size_type n) const {
        if (n >= this->size()) {
            fprintf(stderr, "writing oob at index: %lu in a vector of size: %lu", n, this->size());
            print_stack_trace_and_abort();
        }
        return this->at(n);
    }
};

template <typename T, std::size_t D>
sortVectorByDist(std::vector<Point<T,D>>& points, const Point<T,D>& goal) {
    std::sort(points.begin(), points.end(),
            [&goal] (const Point<T,D>& a, const Point<T,D>& b) {
                a.squaredDistance(goal) < b.squaredDistance(goal);
            });
}

}

#endif // UTILS_H_
