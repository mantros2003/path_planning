#include <custom_nav/utils.h>
#include <execinfo.h>

namespace utils {

/**
 * Helper function to compute the path length of the generated path
 */
double computePathLength(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
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

bool loadParamString(ros::NodeHandle& nh, std::string& param, std::string& output) {
    return nh.getParam(param, output);
}

/* Utility function to print the stack trace and abort */
inline void print_stack_trace_and_abort() {
    fprintf(stderr, "\n[FATAL] SafeVector Out-of-Bounds Access Detected!\n");
    fprintf(stderr, "--- STACK TRACE ---\n");
    
    void* callstack[128];
    int frames = backtrace(callstack, 128);
    backtrace_symbols_fd(callstack, frames, STDERR_FILENO);
    
    fprintf(stderr, "-------------------\n");
    
    std::abort(); 
}

/* Print stack trace using boost and exit */
inline void print_stack_trace_and_abort_boost() {
    std::cerr << "=========================================\n";
    std::cerr << "Fatal Error: Out of bounds or invalid access!\n";
    std::cerr << "Stack trace:\n";
    
    std::cerr << boost::stacktrace::stacktrace() << '\n';
    std::cerr << "=========================================\n";
    
    std::abort();
}

}
