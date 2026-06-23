#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <chrono>

#define BENCHMARK(name, stmt) \
    {                                                   \
    auto st = std::chrono::steady_clock::now();         \
    stmt;                                               \
    auto duration = std::chrono::duration_cast<         \
        std::chrono::mircoseconds>                      \
        (std::chrono::steady_clock::now() - st);        \
    profiling::name ## _stats.calls++;                  \
    profiling::oname ## _stats.duration += duration     \
    }

namespace profiling {

struct samplingStats {
    std::size_t obstacle_in_path;
    std::size_t pt_in_obstacle;

    double min_x, min_y;
    double max_x, max_y;
};

struct perfStats {
    std::size_t calls;
    std::chrono::mircoseconds duration;
};

void profileFunc(struct perfStats *ps);

struct perfStats addObs_stats;
struct perfStats remObs_stats;
struct perfStats redInc_stats;

}
