#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <limits>
#include <algorithm>
#include <array>
#include <vector>

namespace kdtree {

template <typename T, std::size_t D>
struct Point {
    std::array<T, D> point_coords;

    /* Default constructor, initializes every coordinate using the default constructor of T */
    Point() {
        this->point_coords.fill(T{});
    }

    /* Allows us to initialize point by passing a list of coordinates; eg Point({1, 2, 3}) */
    Point(std::initializer_list<T> coords) {
        std::copy_n(
            coords.begin(),
            std::min(coords.size(), D),
            this->point_coords.begin()
        );
        if (coords.size() < D) {
            std::fill(this->point_coords.begin() + coords.size(), this->point_coords.end(), T{});
        }
    }

    T& operator[](std::size_t index) {
        return this->point_coords[index];
    }

    const T& operator[](std::size_t index) const {
        return this->point_coords[index];
    }

    double distanceSquared(const Point& other) const {
        double dist = 0;
        for (std::size_t i = 0; i < D; i++) {
            double diff = static_cast<double>(this->point_coords[i]) - static_cast<double>(other.point_coords[i]);
            dist += (diff * diff);
        }

        return dist;
    }

    double distance(const Point& other) const {
        return std::sqrt(this->distanceSquared(other));
    }
};

template <typename MD, typename T, std::size_t D>
class kdTree {
public:
    struct _kdTreeNode {
        Point<T, D> point;
        MD md;

        _kdTreeNode() : point(), md() {}
        _kdTreeNode(const Point<T, D>& p) : point(p), md() {}
        _kdTreeNode(const Point<T, D>& p, const MD& m) : point(p), md(m) {}
    };

private:
    std::vector<_kdTreeNode> treeNodes;
};

}

#endif // KD_TREE_H_