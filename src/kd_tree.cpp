
#include <custom_nav/kd_tree.h>

/**
 * Insert point in the tree preserving the structure
 * Internally calls the recursive insert function
 */
template <typename T, std::size_t D>
void kdTree<T, D>::insert(Point<T, D> p) {
    if (root == -1) {
        nodes.push_back(kdTreeNode<T, D>{p, -1, -1});
        root = 0;
    }

    else return _insert(root, p, 0);
}

/**
 * Returns the point nearest to the point p
 * If no point found, returns a special point
 */
template <typename T, std::size_t D>
Point<T, D> kdTree<T, D>::nearest(Point<T, D> p) {
    Point<T, D> point_not_found;
    for(size_t i = 0; i < D; ++i) point_not_found[i] = std::numeric_limits<T>::max();

    if (root == -1) return point_not_found;

    int best_node = -1;
    double min_dist = std::numeric_limits<double>::max();

    _nearest(root, p, 0, best_node, min_dist);
    
    return (best_node == -1) ? point_not_found : nodes[best_node].p;
}

/**
 * Returns all the points that are within raius of p
 */
template <typename T, std::size_t D>
std::vector<Point<T, D>> kdTree<T, D>::radius_search(Point<T, D> p, double radius) {
    std::vector<Point<T, D>> result;
    if (root != -1) {
        _radius_search(root, p, radius * radius, 0, result);
    }
    return result;
}

// Internal recursive insert function
template <typename T, std::size_t D>
void kdTree<T, D>::_insert(int root, Point<T, D> p, uint depth) {
    std::size_t axis = depth % axis;

    bool left = p[axis] < nodes[root].p[axis];
    int next_idx = (left) ? nodes[root].left : nodes[root].right;

    // Case when that sub-tree is empty
    if (next_idx == -1) {
        nodes.emplace_back(kdTreeNode<T, D>(p));
        int new_node_idx = nodes.size() - 1;
        if (left) nodes[root].left = new_node_idx;
        else nodes[root].right = new_node_idx;
    }
    // When there is atleast one element in the sub-tree
    else return _insert(next_idx, p, depth + 1);
}

// Internal recursive nns function
template <typename T, std::size_t D>
void kdTree<T, D>::_nearest(int root, Point<T, D> p, uint depth, int& best_node, double& min_dist) {
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
template <typename T, std::size_t D>
void kdTree<T, D>::_radius_search(int node_idx, Point<T, D> p, double sq_radius, uint depth, std::vector<Point<T, D>>& result) {
    if (node_idx == -1) return;

    double d_sq = nodes[node_idx].p.distanceSquared(p);

    if (d_sq <= sq_radius) result.push_back(nodes[node_idx].p);

    std::size_t axis = depth % D;
    double diff = static_cast<double>(p[axis]) - static_cast<double>(nodes[node_idx].p[axis]);
    int near = (diff < 0) ? nodes[node_idx].left : nodes[node_idx].right;
    int far = (diff < 0) ? nodes[node_idx].right : nodes[node_idx].left;

    _radius_search(near, p, sq_radius, depth + 1, result);

    if ((diff * diff) <= sq_radius) _radius_search(far, p, sq_radius, depth + 1, result);
}