#include <custom_nav/kd_tree.h>
#include <iostream>
#include <random>
#include <chrono>
#include <limits>

typedef Point<int, 2> point;

point brute_force(const std::vector<point>& cloud, point target) {
    point best;
    double min_d = std::numeric_limits<double>::max();
    for (const auto& p : cloud) {
        double d = squared_dist(target[0], target[1], p[0], p[1]);
        if (d < min_d) {
            min_d = d;
            best = p;
        }
    }
    return best;
}

std::vector<point> brute_force_radius(const std::vector<point>& cloud, point target, double radius) {
    std::vector<point> result;
    double r_sq = radius * radius; // Compare squared distances to avoid expensive sqrt()
    
    for (const auto& p : cloud) {
        double d_sq = squared_dist(target[0], target[1], p[0], p[1]);
        if (d_sq <= r_sq) {
            result.push_back(p);
        }
    }
    return result;
}

void test_nns() {
    const int NUM_SETS = 100;
    const int POINTS_PER_TREE = 10000;
    const int QUERIES_PER_SET = 100;

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist(0, 100000);

    long long total_kd_time = 0;
    long long total_brute_time = 0;
    int errors = 0;

    for (int s = 0; s < NUM_SETS; ++s) {
        kdTree<int, 2> tree;
        std::vector<point> point_cloud;

        // Build the tree
        for (int i = 0; i < POINTS_PER_TREE; ++i) {
            Point<int, 2> p = {dist(rng), dist(rng)};
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
            double d_brute = squared_dist(query[0], query[1], res_b[0], res_b[1]);
            double d_kd = squared_dist(query[0], query[1], res_kd[0], res_kd[1]);
            if (d_brute != d_kd) {
                std::cout << "Error:" << '\t' << "Query: " << '(' << query[0] << ", " << query[1] << ')' << '\t' << "Brute Force: " << '(' << res_b[0] << ", " << res_b[1] << ')' << '\t' << "kd Tree: " <<  '(' << res_kd[0] << ", " << res_kd[1] << ')' << std::endl;
                errors++;
            }
        }
    }

    std::cout << "--- Benchmark Results (" << NUM_SETS << " sets) ---" << std::endl;
    std::cout << "Total Errors: " << errors << " (Differences in distance)" << std::endl;
    std::cout << "Avg Brute Force time per query: " << (double)total_brute_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Avg KD-Tree time per query:     " << (double)total_kd_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Speedup: " << (double)total_brute_time / total_kd_time << "x" << std::endl;
}

void test_radius_search() {
    const int NUM_SETS = 100;
    const int POINTS_PER_TREE = 10000;
    const int QUERIES_PER_SET = 100;
    const double SEARCH_RADIUS = 5000.0; // Adjust this based on your coordinate spread

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist(0, 100000);

    long long total_kd_time = 0;
    long long total_brute_time = 0;
    int errors = 0;

    // A lambda function to sort points so we can compare the output arrays
    auto point_cmp = [](const point& a, const point& b) {
        if (a[0] != b[0]) return a[0] < b[0];
        return a[1] < b[1];
    };

    for (int s = 0; s < NUM_SETS; ++s) {
        kdTree<int, 2> tree;
        std::vector<point> point_cloud;

        // Build the tree
        for (int i = 0; i < POINTS_PER_TREE; ++i) {
            Point<int, 2> p = {dist(rng), dist(rng)};
            tree.insert(p);
            point_cloud.push_back(p);
        }

        // Generate Queries
        for (int q = 0; q < QUERIES_PER_SET; ++q) {
            point query = {dist(rng), dist(rng)};

            // Time Brute Force
            auto start_b = std::chrono::high_resolution_clock::now();
            std::vector<point> res_b = brute_force_radius(point_cloud, query, SEARCH_RADIUS);
            auto end_b = std::chrono::high_resolution_clock::now();
            total_brute_time += std::chrono::duration_cast<std::chrono::microseconds>(end_b - start_b).count();

            // Time KD-Tree
            auto start_kd = std::chrono::high_resolution_clock::now();
            // Assuming your tree's method is named `radius_search` and returns a std::vector<point>
            std::vector<point> res_kd = tree.radius_search(query, SEARCH_RADIUS); 
            auto end_kd = std::chrono::high_resolution_clock::now();
            total_kd_time += std::chrono::duration_cast<std::chrono::microseconds>(end_kd - start_kd).count();

            // Verify Results
            if (res_b.size() != res_kd.size()) {
                std::cout << "Error: Size mismatch on query (" << query[0] << ", " << query[1] << "). "
                          << "Brute Force found " << res_b.size() << " points, KD-Tree found " << res_kd.size() << " points.\n";
                errors++;
            } else {
                // Sort both vectors to handle out-of-order results
                std::sort(res_b.begin(), res_b.end(), point_cmp);
                std::sort(res_kd.begin(), res_kd.end(), point_cmp);

                bool match = true;
                for (size_t i = 0; i < res_b.size(); ++i) {
                    if (res_b[i][0] != res_kd[i][0] || res_b[i][1] != res_kd[i][1]) {
                        match = false;
                        break;
                    }
                }

                if (!match) {
                    std::cout << "Error: Content mismatch on query (" << query[0] << ", " << query[1] << ")\n";
                    errors++;
                }
            }
        }
    }

    std::cout << "--- Radius Search Benchmark Results (" << NUM_SETS << " sets) ---" << std::endl;
    std::cout << "Search Radius:  " << SEARCH_RADIUS << std::endl;
    std::cout << "Total Errors:   " << errors << std::endl;
    std::cout << "Avg Brute Force time per query: " << (double)total_brute_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Avg KD-Tree time per query:     " << (double)total_kd_time / (NUM_SETS * QUERIES_PER_SET) << " us" << std::endl;
    std::cout << "Speedup: " << (double)total_brute_time / total_kd_time << "x" << std::endl;
}

int main() {
    test_nns();
    test_radius_search();

    return 0;
}