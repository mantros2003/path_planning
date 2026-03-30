#include <random>
#include <chrono>
#include <limits>
#include <custom_nav/kd_tree.h>

/**
 * Insert point in the tree preserving the structure
 * Internally calls the recursive insert function
 */
void kdTree::insert(point p) {
    if (root == -1) {
        nodes.push_back(kdTreeNode{p, -1, -1});
        root = 0;
    }

    else return _insert(root, p, 0);
}

/**
 * Returns the point nearest to the point p
 * If no point found, returns a special point
 */
point kdTree::nearest(point p) {
    point point_not_found = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};;
    if (root == -1) return point_not_found;

    int best_node = -1;
    double min_dist = std::numeric_limits<double>::max();

    _nearest(root, p, 0, best_node, min_dist);

    if (best_node == -1) return point_not_found;
    else return nodes[best_node].p;
}

/**
 * Returns all the points that are within raius of p
 */
std::vector<point> kdTree::radius_search(point p, double radius) {
    std::vector<point> result;
    if (root != -1) {
        _radius_search(root, p, radius * radius, 0, result);
    }
    return result;
}

// Internal recursive insert function
void kdTree::_insert(int root, point p, uint depth) {
    // Extract the appropriate coordinate
    int root_coordinate = (depth % 2 == 0) ? nodes[root].p.first : nodes[root].p.second;
    int coordinate = (depth % 2 == 0) ? p.first : p.second;

    bool left = coordinate < root_coordinate;
    int next_idx = (left) ? nodes[root].left : nodes[root].right;

    // Case when that sub-tree is empty
    if (next_idx == -1) {
        nodes.push_back(kdTreeNode{p, -1, -1});
        int new_node_idx = nodes.size() - 1;
        if (left) nodes[root].left = new_node_idx;
        else nodes[root].right = new_node_idx;
    }
    // When there is atleast one element in the sub-tree
    else return _insert(left ? nodes[root].left : nodes[root].right, p, depth + 1);
}

// Internal recursive nns function
void kdTree::_nearest(int root, point p, uint depth, int& best_node, double& min_dist) {
    if (root == -1) return;

    double d = squared_dist(nodes[root].p.first, nodes[root].p.second,
                            p.first, p.second);
    // Update the min distance and best point
    if (d < min_dist) {
        min_dist = d;
        best_node = root;
    }

    // Determine the partition that contains the point
    double diff = (depth % 2 == 0) ? p.first - nodes[root].p.first : p.second - nodes[root].p.second;
    int near = (diff < 0) ? nodes[root].left : nodes[root].right;
    int far = (diff < 0) ? nodes[root].right : nodes[root].left;

    // Search the partition that contains the point
    _nearest(near, p, depth + 1, best_node, min_dist);

    // Only check the other side if there is a chance that a better point exists on the other side
    if ((diff * diff) < min_dist) _nearest(far, p, depth + 1, best_node, min_dist);
}

// Internal recursive radius search
void kdTree::_radius_search(int node_idx, point p, double sq_radius, uint depth, std::vector<point>& result) {
    if (node_idx == -1) return;

    double d = squared_dist(nodes[node_idx].p.first, nodes[node_idx].p.second, p.first, p.second);
    if (d <= sq_radius) {
        result.push_back(nodes[node_idx].p);
    }

    double diff = (depth % 2 == 0) ? p.first - nodes[node_idx].p.first : p.second - nodes[node_idx].p.second;
    int near = (diff < 0) ? nodes[node_idx].left : nodes[node_idx].right;
    int far = (diff < 0) ? nodes[node_idx].right : nodes[node_idx].left;

    _radius_search(near, p, sq_radius, depth + 1, result);

    if ((diff * diff) <= sq_radius) {
        _radius_search(far, p, sq_radius, depth + 1, result);
    }
}