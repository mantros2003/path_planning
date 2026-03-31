#ifndef KD_TREE_X_H
#define KD_TREE_X_H

#include <vector>
#include <cmath>
#include <utility>
#include <random>
#include <chrono>
#include <limits>

template <typename T>
inline double squared_dist(T x1, T y1, T x2, T y2) {
    double dx = static_cast<double>(x1) - static_cast<double>(x2);
    double dy = static_cast<double>(y1) - static_cast<double>(y2);
    return dx * dx + dy * dy;
}

template <typename T>
inline double distance(T x1, T y1, T x2, T y2) {
    return std::sqrt(squared_dist(x1, y1, x2, y2));
}

// Made point into a generic struct with flexible dimensionality
// Point has D elements of type T
template <typename T, std::size_t D>
struct Point {
    // Array to hold D coordinates of type T
    std::array<T, D> coords;

    // Default constructor, initializes all dimensions to 0
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
    std::size_t index;      // Index of the node in the RRTXPlanner storage for nodes
    int left, right;

    kdTreeNode(Point<T, D> pt, std::size_t index) : p(pt), index(index), left(-1), right(-1) {}
    kdTreeNode(Point<T, D> pt, std::size_t index, int left, int right) : p(pt), index(index), left(left), right(right) {}
};

template <typename T, std::size_t D>
class kdTree {
public:
    kdTree() : root(-1) {}

    void insert(Point<T, D> p, std::size_t index);
    std::size_t nearest(Point<T, D> p);
    std::vector<std::size_t> radius_search(Point<T, D> p, double radius);
    void clear();

private:
    int root;
    std::vector<kdTreeNode<T, D>> nodes;

    // Helper functions use unsigned int for depth tracking
    void _insert(int node_idx, std::size_t index, Point<T, D> p, std::size_t depth);
    void _nearest(int node_idx, Point<T, D> p, std::size_t depth, int& best_idx, double& min_dist_sq);
    void _radius_search(int node_idx, Point<T, D> p, double sq_radius, std::size_t depth, std::vector<std::size_t>& result);
};

/**
 * Insert point in the tree preserving the structure
 * Internally calls the recursive insert function
 */
template <typename T, std::size_t D>
void kdTree<T, D>::insert(Point<T, D> p, std::size_t index) {
    if (root == -1 || nodes.size() == 0) {
        nodes.clear();
        nodes.emplace_back(p, index);
        root = 0;
    }

    else return _insert(root, index, p, 0);
}

/**
 * Returns the point nearest to the point p
 * If no point found, returns a special point
 */
template <typename T, std::size_t D>
std::size_t kdTree<T, D>::nearest(Point<T, D> p) {
    std::size_t point_not_found = std::numeric_limits<std::size_t>::max();

    if (root == -1) return point_not_found;

    int best_node = -1;
    double min_dist = std::numeric_limits<double>::max();

    _nearest(root, p, 0, best_node, min_dist);
    
    return (best_node == -1) ? point_not_found : nodes[best_node].index;
}

/**
 * Returns all the points that are within raius of p
 */
template <typename T, std::size_t D>
std::vector<std::size_t> kdTree<T, D>::radius_search(Point<T, D> p, double radius) {
    std::vector<std::size_t> result;
    if (root != -1) {
        _radius_search(root, p, radius * radius, 0, result);
    }
    return result;
}

// Internal recursive insert function
template <typename T, std::size_t D>
void kdTree<T, D>::_insert(int root, std::size_t index, Point<T, D> p, std::size_t depth) {
    std::size_t axis = depth % D;

    bool left = p[axis] < nodes[root].p[axis];
    int next_idx = (left) ? nodes[root].left : nodes[root].right;

    // Case when that sub-tree is empty
    if (next_idx == -1) {
        nodes.emplace_back(p, index);
        int new_node_idx = nodes.size() - 1;
        if (left) nodes[root].left = new_node_idx;
        else nodes[root].right = new_node_idx;
    }
    // When there is atleast one element in the sub-tree
    else return _insert(next_idx, index, p, depth + 1);
}

// Internal recursive nns function
template <typename T, std::size_t D>
void kdTree<T, D>::_nearest(int root, Point<T, D> p, std::size_t depth, int& best_node, double& min_dist) {
    if (root == -1) return;

    // double d = squared_dist(nodes[root].p[0], nodes[root].p[1],
    //                         p[0], p[1]);
    double d = nodes[root].p.distanceSquared(p);
    // Update the min distance and best point
    if (d < min_dist) {
        min_dist = d;
        best_node = root;
    }

    std::size_t axis = depth % D;

    // Determine the partition that contains the point
    double diff = p[axis] - nodes[root].p[axis];
    int near = (diff > 0) ? nodes[root].right : nodes[root].left;
    int far = (diff < 0) ? nodes[root].right : nodes[root].left;

    // Search the partition that contains the point
    _nearest(near, p, depth + 1, best_node, min_dist);

    // Only check the other side if there is a chance that a better point exists on the other side
    if ((diff * diff) < min_dist) _nearest(far, p, depth + 1, best_node, min_dist);
}

// Internal recursive radius search
template <typename T, std::size_t D>
void kdTree<T, D>::_radius_search(int node_idx, Point<T, D> p, double sq_radius, std::size_t depth, std::vector<std::size_t>& result) {
    if (node_idx == -1) return;

    double d_sq = nodes[node_idx].p.distanceSquared(p);

    if (d_sq <= sq_radius) result.push_back(nodes[node_idx].index);

    std::size_t axis = depth % D;
    double diff = static_cast<double>(p[axis]) - static_cast<double>(nodes[node_idx].p[axis]);
    int near = (diff < 0) ? nodes[node_idx].left : nodes[node_idx].right;
    int far = (diff < 0) ? nodes[node_idx].right : nodes[node_idx].left;

    _radius_search(near, p, sq_radius, depth + 1, result);

    if ((diff * diff) <= sq_radius) _radius_search(far, p, sq_radius, depth + 1, result);
}

template <typename T, std::size_t D>
void kdTree<T, D>::clear() {
    nodes.clear();
    root = -1;
}

#endif // KD_TREE_H
