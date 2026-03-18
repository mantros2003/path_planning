#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>
#include <cmath>
#include <utility>

inline double squared_dist(int x1, int y1, int x2, int y2) {
    long long dx = (long long)x1 - x2;
    long long dy = (long long)y1 - y2;
    return (double)(dx * dx + dy * dy);
}

inline double distance(int x1, int y1, int x2, int y2) {
    return std::sqrt(squared_dist(x1, y1, x2, y2));
}

typedef unsigned int uint;
typedef std::pair<int, int> point;

// Made point into a generic struct with flexible dimensionality
template <typename T, std::size_t D>
struct Point {
    // Array to hold D coordinates of type T
    std::array<T, D> coords;

    // Default constructor (initializes all dimensions to 0)
    Point() { coords.fill(0); }

    // Constructor for initializer list: Point<double, 3> p({1.0, 2.0, 3.0});
    Point(std::initializer_list<T> list) {
        std::copy_n(list.begin(), std::min(list.size(), D), coords.begin());
    }

    // Access operator for intuitive use: p[0] instead of p.coords[0]
    T& operator[](std::size_t index) { return coords[index]; }
    const T& operator[](std::size_t index) const { return coords[index]; }

    double distanceSquared(const Point& other) const {
        double dist = 0;
        for (std::size_t i = 0; i < D; ++i) {
            double diff = static_cast<double>(coords[i]) - static_cast<double>(other.coords[i]);
            dist += diff * diff;
        }
        return dist;
    }

    double distance(const Point& other) const {
        return std::sqrt(distanceSquared(other));
    }
};

template <typename T, std::size_t D>
struct kdTreeNode {
    Point<T, D> p;
    int left, right;

    kdTreeNode(Point<T, D> pt) : p(pt), left(-1), right(-1) {}
};

template <typename T, std::size_t D>
class kdTree {
public:
    kdTree() : root(-1) {}

    void insert(Point<T, D> p);
    Point<T, D> nearest(Point<T, D> p);
    std::vector<Point<T, D>> radius_search(Point<T, D> p, double radius);

private:
    int root;
    std::vector<kdTreeNode<T, D>> nodes;

    // Helper functions use unsigned int for depth tracking
    void _insert(int node_idx, Point<T, D> p, uint depth);
    void _nearest(int node_idx, Point<T, D> p, uint depth, int& best_idx, double& min_dist_sq);
    void _radius_search(int node_idx, Point<T, D> p, double sq_radius, uint depth, std::vector<Point<T, D>>& result);
};

#endif // KD_TREE_H
