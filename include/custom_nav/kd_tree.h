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

struct kdTreeNode {
    point p;
    int left, right;
};

class kdTree {
public:
    kdTree() : root(-1), nodes{} {}

    void insert(point p);
    point nearest(point p);

private:
    int root;
    std::vector<kdTreeNode> nodes;

    void _insert(int root, point p, uint depth);
    void _nearest(int, point, uint, int&, double&);
};

#endif // KD_TREE_H
