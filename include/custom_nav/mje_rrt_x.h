#ifndef CUSTOM_NAV_MJE_RRTX_PLANNER_H
#define CUSTOM_NAV_MJE_RRTX_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <custom_nav/RRTXStats.h>
#include <custom_nav/kd_tree_x.h>
#include <custom_nav/utils.h>
#include <custom_nav/mjetable.h>

#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <limits>
#include <string>

#define _VIS_RRTX_TREE

#ifdef USE_SAFE_VECTOR
    template <typename T, typename Allocator = std::allocator<T>>
    using Vector = utils::safeVec<T, Allocator>;
#else
    template <typename T, typename Allocator = std::allocator<T>>
    using Vector = std::vector<T, Allocator>;
#endif

namespace custom_planner {

struct State {
    double x, y, theta;
};

struct Node {
    State state;
    std::size_t par_idx;
    bool in_queue;
    bool is_xi;

    double g;
    double lmc;

    Vector<std::size_t> nbr_init;
    Vector<std::size_t> nbr_running;
    std::set<std::size_t> blocked_nbrs;
    Vector<std::size_t> children;

    static constexpr std::size_t INVALID_IDX = std::numeric_limits<std::size_t>::max();

    Node(double start_x = 0.0, double start_y = 0.0, double theta = 0.0) 
        : state{start_x, start_y, theta}, par_idx(INVALID_IDX), 
          g(std::numeric_limits<double>::infinity()), 
          lmc(std::numeric_limits<double>::infinity()), 
          in_queue(false), is_xi(false) {}

    void removeChild(std::size_t);
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

class MJERRTXPlanner : public nav_core::BaseGlobalPlanner
{
    public:
        MJERRTXPlanner() 
        : costmap_(nullptr), initialized_(false), planned_(false),
        height_m_(0.0), width_m_(0.0), height_c_(0), width_c_(0),
        origin_x(0.0), origin_y(0.0), resolution_(0.0),
        sampling_min_x_(0.0), sampling_max_x_(0.0),
        sampling_min_y_(0.0), sampling_max_y_(0.0),
        rad_const_(4.0),
        obstacle_cost_threshold_(1),
        step_length_(0.2),          // Default edge length
        goal_tolerance_(0.2),       // Default tolerance to reach goal
        epsilon_(0.1),              // Default epsilon for collision checking/math
        max_iters_(6000),           // Default maximum iterations before giving up
        rng(std::random_device{}()),// Initialize the random number genera with the random device
        rand01(0.0, 1.0),           // Initialize the distribution
        radius_(0.3),
        start_proxy(Node::INVALID_IDX),
        kc(3.0)
        {}

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(
            const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan
        );

    private:
        costmap_2d::Costmap2D* costmap_;
        std::string global_frame_id_;
        bool initialized_;
        bool planned_;
        double height_m_, width_m_;             // Map width in meters
        unsigned int height_c_, width_c_;       // Map width in number of cells
        double origin_x, origin_y;
        double resolution_;                     // Sizde of each cell
        unsigned int sampling_min_x_, sampling_max_x_;
        unsigned int sampling_min_y_, sampling_max_y_;
        double radius_;                         // The radius for neighborhood search
        Vector<uint8_t> costmap_snapshot_;      // Holds the last known values of the costmap
        Vector<unsigned int> obstacles_;        // Indices of obstacles in the costmap
        Vector<std::pair<unsigned int, unsigned int>> free_cells;   // Maintain a list of free cells and sample from this to make sampling efficient

        Point<double, 2> goal_, start_;

        std::size_t start_proxy;

        // Tree and node containers
        Vector<Node> nodes_;                                // Stores all the nodes 
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
        std::mt19937 rng;
        std::uniform_real_distribution<double> rand01;

        // Global containers
        kdTree<double, 2> kd_tree;

        // Containers for keeping track of inconsistent nodes
        std::set<QKey> queue_;
        std::unordered_map<std::size_t, QKey> queueMap_;

        // Publishers
        ros::Publisher tree_pub_;           // For visualising RRTx tree
        ros::Publisher stats_pub_;          // For publishing stats

        // SI-QBC helpers
        lt::ConstraintTable constraint_table_;
        double kc;

        // Functions
        void updateCostmapParams();
        void addPointToTree(State, std::size_t);
        bool extractPath(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&, std::vector<geometry_msgs::PoseStamped>&, ros::Time);
        bool validatePoint(double, double);
        std::pair<std::size_t, State> feasibleNearAndSteer(Point<double, 2>);
        void reduceInconsistency();
        void flushQueue();
        void rewireNeighbors(std::size_t);
        void cullNeighbors(std::size_t);
        void updateLMC(std::size_t);
        std::pair<std::size_t, double> findBestParent(
                std::size_t, Vector<std::size_t>* = nullptr);
        void updateObstacles();
        void removeObstacle(unsigned int);
        void addObstacle(unsigned int);
        bool isFeasible(State, State, double&, bool&);
        double getarclength(State, State);
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
        void buildFreeCellList();
        double squaredDistance(const std::size_t, const std::size_t) const;
        double distance(const std::size_t, const std::size_t) const;
        void buildTreeMarker(const std::string& frame_id = "map");
};

} // namespace custom_nav

#endif // CUSTOM_NAV_MJE_RRTX_PLANNER_H
