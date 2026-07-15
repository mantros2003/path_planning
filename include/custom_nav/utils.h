#ifndef UTILS_H_
#define UTILS_H_

#ifdef __linux__
#include <geometry_msgs/PoseStamped.h>
#include <execinfo.h>
#include <boost::stacktrace.hpp>
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

inline void print_stack_trace_and_abort_boost() {
    fprintf(stderr, "\n[FATAL] SafeVector Out-of-Bounds Access Detected!\n");
    fprintf(stderr, "--- STACK TRACE ---\n");
    
    void* callstack[128];
    int frames = backtrace(callstack, 128);
    backtrace_symbols_fd(callstack, frames, STDERR_FILENO);
    
    fprintf(stderr, "-------------------\n");
    
    std::abort(); 
}

void print_stack_trace_and_abort() {
    std::cerr << "=========================================\n";
    std::cerr << "Fatal Error: Out of bounds or invalid access!\n";
    std::cerr << "Stack trace:\n";
    
    std::cerr << boost::stacktrace::stacktrace() << '\n';
    std::cerr << "=========================================\n";
    
    std::abort();
}

template <typename T, typename Allocator = std::allocator<T>>
class safeVec : public std::vector<T, Allocator> {
public:
    using std::vector<T, Allocator>::vector;

    typename std::vector<T, Allocator>::reference
    operator[] (typename std::vector<T, Allocator>::size_type n) {
        if (n >= this->size()) {
            fprintf(stderr, "writing oob at index: %lu in a vector of size: %lu", n, this->size());
            print_stack_trace_and_abort_boost();
        }
        return this->at(n);
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

}

#endif // UTILS_H_
