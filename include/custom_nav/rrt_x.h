#ifndef CUSTOM_NAV_RRTX_PLANNER_H
#define CUSTOM_NAV_RRTX_PLANNER_H

#ifdef __linux__
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#endif

#include <custom_nav/kd_tree_x.h>

#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <limits>

#define _VIS_RRTX_TREE

namespace custom_planner {

/**
 * Structure to store the information needed for the RRTx algorithm
 */
struct Node {
    double x, y;
    std::size_t par_idx;
    bool in_queue;  // Indicates if it is in the priority queue

    double g;       // Current estimate of cost-to-goal
    double lmc;     // Look ahead cost-to-goal

    // We are using unsigned int for storing index of the nodes
    // Using only one container for in and out as graph will be symmetric
    std::vector<std::size_t> nbr_init;                              // Initial neighbors
    std::vector<std::size_t> nbr_running;                           // Neighbors within r(|V|)
    std::set<std::size_t> blocked_nbrs;                             // Neighbors to whom distance is inf
    std::vector<std::size_t> children;

    // To indicate invalid index
    static constexpr std::size_t INVALID_IDX = std::numeric_limits<std::size_t>::max();

    // Constructors
    // Default
    Node()
        : x(0.0), y(0.0), par_idx(INVALID_IDX),
          g(std::numeric_limits<double>::infinity()),
          lmc(std::numeric_limits<double>::infinity()),
          in_queue(false) {}
    
    Node(double start_x, double start_y) 
        : x(start_x), y(start_y), par_idx(INVALID_IDX), 
          g(std::numeric_limits<double>::infinity()), 
          lmc(std::numeric_limits<double>::infinity()), 
          in_queue(false) {}
};

struct QKey {
    double k1, k2;
    std::size_t index;

    bool operator<(const QKey& other) const {
        if (k1 != other.k1) return k1 < other.k1;
        if (k2 != other.k2) return k2 < other.k2;
        return index < other.index;
    }
};

class RRTXPlanner : public nav_core::BaseGlobalPlanner
{
    public:
        RRTXPlanner() 
        : costmap_(nullptr), initialized_(false), planned_(false),
        height_m_(0), width_m_(0), height_c_(0), width_c_(0),
        rad_const_(10.0),
        obstacle_cost_threshold_(50),
        step_length_(0.2),          // Default edge length
        goal_tolerance_(0.2),       // Default tolerance to reach goal
        epsilon_(0.1),              // Default epsilon for collision checking/math
        max_iters_(1500),           // Default maximum iterations before giving up
        rng{dev()},                 // Initialize the random number genera with the random device
        rand01{0.0, 1.0},           // Initialize the distribution
        radius_(0.3),
        start_proxy(Node::INVALID_IDX),
        {}

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(
            const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan
        );

    private:
        costmap_2d::Costmap2D* costmap_;
        bool initialized_;
        bool planned_;
        double height_m_, width_m_;             // Map width in meters
        unsigned int height_c_, width_c_;       // Map width in number of cells
        double origin_x, origin_y;
        double resolution_;                     // Sizde of each cell
        unsigned int sampling_min_x_, sampling_max_x_;
        unsigned int sampling_min_y_, sampling_max_y_;
        double radius_;                         // The radius for neighborhood search
        std::vector<uint8_t> costmap_snapshot_; // Holds the last known values of the costmap
        std::vector<unsigned int> obstacles_;   // Indices of obstacles in the costmap
        std::vector<std::pair<unsigned int, unsigned int>> free_cells;   // Maintain a list of free cells and sample from this to make sampling efficient

        Point<double, 2> goal_, start_;

        std::size_t start_proxy;

        // Tree and node containers
        std::vector<struct Node> nodes_;                    // Stores all the nodes 
        std::unordered_set<std::size_t> orphan_set_;        // All the nodes which are cut-off from the main  tra

        // Hyperparams
        double rad_const_;                      // Constant used in radius
        double step_length_;                    // Edge length of a tree edge
        double goal_tolerance_;
        double epsilon_;                        // Constant used in consistency check
        unsigned int max_iters_;
        double start_dist_threshold_;
        unsigned char obstacle_cost_threshold_; // Treat all grids with cost more than this as an obstacle

        // Ranom number generators
        std::random_device dev;
        std::mt19937 rng{dev()};
        std::uniform_real_distribution<double> rand01{0.0, 1.0};

        // Global containers
        kdTree<double, 2> kd_tree;

        // Containers for keeping track of inconsistent nodes
        std::set<QKey> queue_;
        std::unordered_map<std::size_t, QKey> queueMap_;

        // Publisher for visualization messages
        ros::Publisher tree_pub_;

        // Functions
        bool addPointToTree(Point<double, 2>, std::size_t);
        bool extractPath(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&, std::vector<geometry_msgs::PoseStamped>&, ros::Time);
        void reduceInconsistency();
        void rewireNeighbors(std::size_t);
        void cullNeighbors(std::size_t);
        void updateLMC(std::size_t);
        void updateObstacles();
        void removeObstacle(unsigned int);
        void addObstacle(unsigned int);
        Point<double, 2> steer(double, double, double, double);
        bool isNewGoal(double, double);
        void resetTree();
        bool isConnected(double, double);
        bool hasObstacle(unsigned int, unsigned int);
        bool hasObstacle(Point<double, 2>, Point<double, 2>);
        bool hasObstacle(double, double, double, double);
        double getRadius() const;
        std::size_t findStartProxy();
        void propogateDescendents();
        void verifyOrphan(std::size_t);
        void verifyQueue(std::size_t);
        QKey makeKey(std::size_t);
        bool keyLess(const QKey&, const QKey&) const;
        bool isEdgeInCollision(double, double, double, double, 
                       double, double, double, double);
        std::pair<double, double> samplePoint();
        void _buildFreeCellList();
        double squaredDistance(const std::size_t, const std::size_t) const;
        double distance(const std::size_t, const std::size_t) const;
        void buildTreeMarker(const std::string& frame_id = "map");
};

} // namespace custom_planner

#endif //CUSTOM_NAV_RRTX_PLANNER_H
