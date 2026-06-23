#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <chrono>

#define BENCHMARK(name, stmt) \
    {                                                   \
    auto st = std::chrono::steady_clock::now();         \
    stmt;                                               \
    auto duration = std::chrono::duration_cast<         \
        std::chrono::microseconds>                      \
        (std::chrono::steady_clock::now() - st);        \
    profiling::name ## _stats.calls++;                  \
    profiling::name ## _stats.duration += duration;     \
    }

#define TIME(id, stmt) \
{                                                       \
    auto st = std::chrono::steady_clock::now();         \
    stmt;                                               \
    auto duration = std::chrono::duration_cast<         \
        std::chrono::microseconds>                      \
        (std::chrono::steady_clock::now() - st);        \
    ROS_INFO("%s took %ld us", id, duration.count());   \
}                                                       \

namespace profiling {

struct samplingStats {
    std::size_t obstacle_in_path;
    std::size_t pt_in_obstacle;

    double min_x, min_y;
    double max_x, max_y;
};

struct perfStats {
    std::size_t calls;
    std::chrono::microseconds duration;
};

void profileFunc(struct perfStats *ps);

struct perfStats addObs_stats;
struct perfStats remObs_stats;
struct perfStats redInc_stats;

}

#endif // BENCHMARK_H_
