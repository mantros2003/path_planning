#include <custom_nav/kd_tree.h>
#include <iostream>
#include <random>
#include <chrono>
#include <limits>

point brute_force(const std::vector<point>& cloud, point target) {
    point best;
    double min_d = std::numeric_limits<double>::max();
    for (const auto& p : cloud) {
        double d = squared_dist(target.first, target.second, p.first, p.second);
        if (d < min_d) {
            min_d = d;
            best = p;
        }
    }
    return best;
}

int main() {
    const int NUM_SETS = 100;
    const int POINTS_PER_TREE = 10000;
    const int QUERIES_PER_SET = 100;

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist(0, 100000);

    long long total_kd_time = 0;
    long long total_brute_time = 0;
    int errors = 0;

    for (int s = 0; s < NUM_SETS; ++s) {
        kdTree tree;
        std::vector<point> point_cloud;

        // Build the tree
        for (int i = 0; i < POINTS_PER_TREE; ++i) {
            point p = {dist(rng), dist(rng)};
            tree.insert(p);
            point_cloud.push_back(p);
        }

        // Generate Queries
        for (int q = 0; q < QUERIES_PER_SET; ++q) {
            point query = {dist(rng), dist(rng)};

            // Time Brute Force
            auto start_b = std::chrono::high_resolution_clock::now();
            point res_b = brute_force(point_cloud, query);
            auto end_b = std::chrono::high_resolution_clock::now();
            total_brute_time += std::chrono::duration_cast<std::chrono::microseconds>(end_b - start_b).count();

            // Time KD-Tree
            auto start_kd = std::chrono::high_resolution_clock::now();
            point res_kd = tree.nearest(query);
            auto end_kd = std::chrono::high_resolution_clock::now();
            total_kd_time += std::chrono::duration_cast<std::chrono::microseconds>(end_kd - start_kd).count();

            // Verify Results (using squared distance to check for tie-breaks)
            if (squared_dist(query.first, query.second, res_b.first, res_b.second) != squared_dist(query.first, query.second, res_kd.first, res_kd.second)) {
                std::cout << "Error:" << '\t' << "Brute Force: " << '(' << res_b.first << ", " << res_b.second << ')' << '\t' << "kd Tree: " <<  '(' << res_kd.first << ", " << res_kd.second << ')' << std::endl;
                errors++;
            }
        }
    }

    std::cout << "--- Benchmark Results (" << NUM_SETS << " sets) ---" << std::endl;
    std::cout << "Total Errors: " << errors << " (Differences in distance)" << std::endl;
    std::cout << "Avg Brute Force time per query: " << (double)total_brute_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Avg KD-Tree time per query:     " << (double)total_kd_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Speedup: " << (double)total_brute_time / total_kd_time << "x" << std::endl;

    return 0;
}
