#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <custom_nav/kd_tree.h>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <string>
#include <cmath>
#include <limits>

namespace custom_planner {

// ── per-node data ────────────────────────────────────────────────────────────

struct RRTxNode {
    unsigned int index;

    double lmc;   // locally minimum cost-to-goal (one-step lookahead)
    double g;     // best known cost-to-goal

    // Tree parent  p_T^+(v)
    unsigned int parent    = 0;
    bool         hasParent = false;

    // ── Neighbour sets ───────────────────────────────────────────────────────
    //  N0+(v)  –  nodes u reachable FROM v, no radius bound
    //  N0-(v)  –  nodes u that can reach v, no radius bound
    //  Nr+(v)  –  nodes u reachable FROM v within ball radius r
    //  Nr-(v)  –  nodes u that can reach v within r
    std::unordered_set<unsigned int> N0_plus;
    std::unordered_set<unsigned int> N0_minus;
    std::unordered_set<unsigned int> Nr_plus;
    std::unordered_set<unsigned int> Nr_minus;

    // Tree children (nodes whose tree-parent is this node)
    std::unordered_set<unsigned int> children;

    RRTxNode()
        : index(0),
          lmc(std::numeric_limits<double>::infinity()),
          g  (std::numeric_limits<double>::infinity()) {}

    explicit RRTxNode(unsigned int idx)
        : index(idx),
          lmc(std::numeric_limits<double>::infinity()),
          g  (std::numeric_limits<double>::infinity()) {}
};

// ── priority-queue key  key(v) = (min(g,lmc), lmc) ──────────────────────────

struct QKey {
    double       k1, k2;
    unsigned int index;       // tie-break for std::set uniqueness

    bool operator<(const QKey& o) const {
        if (k1 != o.k1) return k1 < o.k1;
        if (k2 != o.k2) return k2 < o.k2;
        return index < o.index;
    }
};

// ── planner ──────────────────────────────────────────────────────────────────

class RRTxPlanner : public nav_core::BaseGlobalPlanner {
public:
    RRTxPlanner()  = default;
    ~RRTxPlanner() = default;

    void initialize(std::string name,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    // ── RRTx core algorithms ─────────────────────────────────────────────────
    double shrinkingBallRadius() const;
    void   extend            (unsigned int v, double r);
    void   cullNeighbors     (unsigned int v, double r);
    void   rewireNeighbors   (unsigned int v);
    void   reduceInconsistency();
    void   findParent        (unsigned int v,
                              const std::vector<unsigned int>& U);
    void   updateLMC         (unsigned int v);

    // ── Dynamic obstacle handling ────────────────────────────────────────────

    // Compare the live costmap against costmap_snapshot_ and fill two lists:
    //   newly_blocked  – cells that flipped from free to obstacle
    //   newly_cleared  – cells that flipped from obstacle to free
    // Updates costmap_snapshot_ to the current map before returning.
    // Returns true if any cell changed.
    bool detectObstacleChanges(
            std::vector<unsigned int>& newly_blocked,
            std::vector<unsigned int>& newly_cleared);

    // Algorithm 1 lines 5-6.
    // Handles both newly blocked and newly cleared cells in one pass.
    void updateObstacles(const std::vector<unsigned int>& newly_blocked,
                         const std::vector<unsigned int>& newly_cleared);

    // ---- Blocked-edge helpers -----------------------------------------------

    // Cut the tree edge (parent(v) → v) because it is now blocked.
    // Disconnects v, sets lmc = g = INF, and recursively does the same for
    // every tree descendant of v (iterative BFS to avoid stack overflow).
    // Every disconnected node is added to orphan_set_ and pushed onto the
    // inconsistency queue so reduceInconsistency can attempt re-routing.
    void propagateOrphan(unsigned int v);

    // After all blocked edges have been handled, try to re-parent every node
    // in orphan_set_ using its surviving neighbour sets.
    // Nodes that find a valid finite-cost parent are removed from orphan_set_
    // and inserted into the queue; reduceInconsistency then propagates their
    // improved costs to their own descendants.
    void reintegrateOrphans();

    // Remove the directed edges (u→v) and (v→u) from all neighbour sets of
    // both u and v (called when a cell on the segment u-v becomes blocked).
    void removeEdgeFromSets(unsigned int u, unsigned int v);

    // ---- Cleared-edge helpers -----------------------------------------------

    // Restore the edge (u, v) in both nodes' neighbour sets if the segment
    // is now fully free.  Called for pairs whose edge was previously blocked
    // by a cell that has now been cleared.
    void restoreEdgeInSets(unsigned int u, unsigned int v);

    // ── Queue helpers ────────────────────────────────────────────────────────
    QKey makeKey        (unsigned int v) const;
    bool keyLess        (const QKey& a, const QKey& b) const;
    void verifyQueue    (unsigned int v);
    void removeFromQueue(unsigned int v);

    // ── Geometry / costmap helpers ───────────────────────────────────────────
    bool         hasObstacle      (unsigned int from, unsigned int to) const;
    double       edgeCost         (unsigned int from, unsigned int to) const;
    double       dist             (int x1, int y1, int x2, int y2)    const;
    unsigned int steer            (unsigned int from, unsigned int to,
                                   double step)                        const;
    // Returns true if the Bresenham segment from→to passes through 'cell'.
    bool         edgePassesThrough(unsigned int from, unsigned int to,
                                   unsigned int cell)                  const;

    // ── Node-map accessor  (creates entry on first touch) ───────────────────
    RRTxNode& node(unsigned int idx);

    // ════════════════════════════════════════════════════════════════════════
    //  State
    // ════════════════════════════════════════════════════════════════════════

    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    costmap_2d::Costmap2D*    costmap_     = nullptr;
    bool initialized_ = false;

    unsigned int width_  = 0;
    unsigned int height_ = 0;

    // Flat copy of the costmap char-map taken at the end of each makePlan
    // call.  Diffed against the live map on the next call to find changes.
    std::vector<uint8_t> costmap_snapshot_;

    // Current robot and goal cell indices
    unsigned int start_index_ = 0;   // v_bot
    unsigned int goal_index_  = 0;

    // Cached goal world-position to detect goal changes across calls
    double cached_goal_x_ = 1e18;
    double cached_goal_y_ = 1e18;

    // True once a valid tree has been built and can be reused across calls
    bool tree_initialized_ = false;

    // ── Hyper-parameters ────────────────────────────────────────────────────
    double       step_size_ = 5.0;
    double       goal_bias_ = 0.10;
    double       epsilon_   = 1e-6;
    unsigned int max_iters_ = 100000;
    double       gamma_     = 0.0;   // 0 → auto  (set to 2*step_size_)

    // ── Tree data structures ─────────────────────────────────────────────────
    std::unordered_map<unsigned int, RRTxNode> nodes_;
    kdTree tree_;

    // ── Inconsistency queue ──────────────────────────────────────────────────
    std::set<QKey>                         queue_;
    std::unordered_map<unsigned int, QKey> inQueue_;   // idx → current key

    // ── Orphan set ───────────────────────────────────────────────────────────
    // Nodes that have been disconnected from the tree due to blocked edges.
    // They stay in nodes_ so neighbour sets remain intact; lmc = g = INF.
    std::unordered_set<unsigned int> orphan_set_;
};

} // namespace custom_planner