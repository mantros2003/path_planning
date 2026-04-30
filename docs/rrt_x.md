# RRTx Planner — Algorithm Implementation Notes

This document explains the standard RRTx algorithm and the implementation details of the RRTx planner in this repository, including known deviations from the paper.

Common ROS planner infrastructure (plugin interface, costmap access, coordinate transforms, and path construction) is described in:

```
ros_global_planner_basics.md
```

---

# 1. What is RRTx?

**RRTx** (Rapidly-exploring Random Tree with ×-update) is a sampling-based motion planning algorithm introduced by Otte and Frazzoli (2016). It extends RRT* with the ability to **efficiently repair the tree when the environment changes** (obstacles appear or disappear), without rebuilding from scratch.

The key ideas that distinguish RRTx from RRT*:

| Feature | RRT* | RRTx |
|---|---|---|
| Optimality | Asymptotically optimal | Asymptotically optimal |
| Dynamic replanning | No (rebuild required) | Yes (in-place repair) |
| Cost propagation | Forward (start → goal) | Backward (goal → robot) |
| Two cost values per node | No | Yes (g and lmc) |
| Priority queue repair | No | Yes (reduceInconsistency) |

---

# 2. Core Concept — Goal-Rooted Tree

Unlike RRT and RRT*, which root the tree at the **start** and grow toward the **goal**, RRTx roots the tree at the **goal**.

Every node stores a cost representing the distance-to-goal, not distance-from-start. This inversion is the foundation that makes dynamic replanning efficient: when the robot moves or the environment changes, only the local subtree near the change needs to be repaired rather than the entire tree.

The root (goal node) always has:

```
g(root) = 0
lmc(root) = 0
```

---

# 3. Two Cost Values — g and lmc

Each node maintains two cost estimates:

```
g(v)    — current estimate of cost-to-goal (may be inconsistent)
lmc(v)  — look-ahead minimum cost: best cost through any neighbor
```

A node is **consistent** when:

```
g(v) == lmc(v)   (within epsilon tolerance)
```

A node is **inconsistent** when:

```
g(v) > lmc(v)    — lmc improved but g hasn't caught up yet
```

The algorithm continuously works to make all nodes consistent by propagating lmc improvements through the tree. This is analogous to how D* Lite maintains two cost values (g and rhs) to handle dynamic changes incrementally.

---

# 4. Priority Queue and Key Function

The priority queue orders nodes for processing based on how urgently they need their costs updated.

The key for each node v is:

```
key(v) = (min(g(v), lmc(v)),  g(v))
```

Nodes with lower keys are processed first. The first component prioritizes nodes that are cheapest to reach; the second breaks ties by preferring nodes whose g is already low (more reliable estimates).

---

# 5. Neighbor Lists — Initial and Running

RRTx maintains two neighbor sets per node:

```
nbr_init     — all neighbors ever added when this node was inserted (permanent)
nbr_running  — neighbors currently within the shrinking ball radius r(n)
```

The **shrinking ball** is central to RRT*-family asymptotic optimality. As more nodes are added, the radius shrinks:

```
r(n) = min( γ * sqrt(log(n) / n),  δ )
```

Where:
- `n` — current number of nodes
- `γ` — a constant (implementation uses `rad_const_ = 10.0`)
- `δ` — maximum edge length (`step_length_`)

This is the correct formula for 2D (dimension d=2, so 1/d = 0.5).

Keeping `nbr_init` around means old connections are never forgotten. They can be reconsidered when the graph needs repair, which is important for correctness during dynamic replanning.

---

# 6. Standard RRTx Algorithm

```
Initialize tree with goal node: g = 0, lmc = 0

loop:
    sample random point x_rand
    find nearest tree node x_near
    steer toward x_rand to get x_new

    if x_new is collision-free:
        find neighbors of x_new within radius r(n)
        choose best parent (minimizes lmc)
        add x_new to tree with consistent g = lmc
        add x_new to neighbors' lists
        rewireNeighbors(x_new)     ← offer x_new as parent to its neighbors
        reduceInconsistency()      ← propagate any lmc improvements

when obstacles change:
    for each new obstacle:
        mark blocked edges
        verifyOrphan(child)        ← disconnect child from parent
    propagateDescendants()         ← flood-fill cut subtrees, invalidate g
    verifyQueue(robot_proxy)
    reduceInconsistency()

    for each removed obstacle:
        restore edge (clear from blocked_nbrs)
        updateLMC(v), updateLMC(u) ← find better parents
        verifyQueue(v), verifyQueue(u)
    reduceInconsistency()

extract path: follow par_idx from robot_proxy to root
```

---

# 7. Core Subroutines

## reduceInconsistency

The main repair loop. Pops inconsistent nodes from the priority queue and propagates lmc updates until the robot's proxy node is consistent and no cheaper node remains unprocessed.

```
while queue not empty AND
      (top key < robot key   OR
       robot is inconsistent  OR
       robot g = inf          OR
       robot is in queue):

    pop v from queue
    if v.g - v.lmc > epsilon:
        updateLMC(v)
        rewireNeighbors(v)
    v.g = v.lmc
```

The stopping condition uses the **robot proxy's key** so the algorithm stops as soon as the path for the robot is resolved, without processing the entire queue.

## updateLMC

Finds the best parent for a node by checking all running and initial neighbors:

```
for each neighbor u of v:
    if u is not an orphan:
        cost = distance(v, u) + lmc(u)
        if cost < best_cost:
            best_cost = cost
            best_parent = u

v.lmc = best_cost
update v.par_idx to best_parent
```

## rewireNeighbors

When v's lmc has improved, offer v as a new parent to its neighbors:

```
if v.g - v.lmc <= epsilon: return   (not worth offering)

cullNeighbors(v)

for each neighbor u in v.nbr_init:
    if lmc(v) + distance(v, u) < lmc(u):
        reparent u to v
        u.lmc = lmc(v) + distance(v, u)
        verifyQueue(u)
```

## cullNeighbors

Prunes the running neighbor list by removing nodes now outside the current ball radius:

```
for each u in v.nbr_running:
    if distance(v, u) > r(n):
        remove u from v.nbr_running
        remove v from u.nbr_running
```

## propagateDescendants

When an obstacle severs a parent-child edge, all descendants of the severed node become **orphans** — disconnected from the goal. This function:

1. BFS through the children of each initial orphan to find all descendants
2. Invalidates `g` of non-orphan neighbors (so they re-route around the orphan subtree)
3. Severs all tree connections within the orphan set (sets g = lmc = ∞, clears parent pointers)

## verifyOrphan / verifyQueue

```
verifyOrphan(v):   remove v from queue if present; add to orphan_set

verifyQueue(v):    compute key(v); insert or update v in queue
```

---

# 8. Implementation Details

## Node Structure

```cpp
struct Node {
    double x, y;
    std::size_t par_idx;       // Parent in tree

    double g;                  // Current cost-to-goal
    double lmc;                // Look-ahead min cost

    std::vector<std::size_t> nbr_init;      // All-time neighbors
    std::vector<std::size_t> nbr_running;   // Neighbors within r(n)
    std::set<std::size_t> blocked_nbrs;     // Edges blocked by obstacles
    std::vector<std::size_t> children;      // Child nodes in tree

    bool in_queue;
};
```

## QKey

```cpp
struct QKey {
    double k1;         // min(g, lmc)
    double k2;         // g
    std::size_t index; // for set ordering uniqueness
};
```

The `std::set<QKey>` with a custom `operator<` serves as the priority queue, and `std::unordered_map<size_t, QKey> queueMap_` allows O(1) key lookup and update.

## Obstacle Collision Checking

Two methods are used:

- **`hasObstacle(Point, Point)`** — uses `LineIterator` to walk the costmap cells along a segment. Used for edge validity during tree building and start proxy selection.
- **`isEdgeInCollision(...)`** — parametric line-AABB intersection (Cohen–Sutherland style). Used for fast obstacle-cell intersection testing in `addObstacle`.

## Distance Function

`distance(idx1, idx2)` returns `∞` if `idx2` is in `idx1.blocked_nbrs`. This allows the cost functions to treat blocked edges as infinite-cost without special casing.

## KD-Tree

A custom 2D KD-tree supports:
- `insert(point, index)` — O(log n) average
- `nearest(point)` — O(log n) average
- `radius_search(point, r)` — O(log n + k) where k is the result count

---

# 9. Hyperparameters

| Parameter | Default | Description |
|---|---|---|
| `step_length_` | 0.3 m | Maximum edge length; also maximum ball radius |
| `rad_const_` | 10.0 | Constant γ in the shrinking ball formula |
| `epsilon_` | 0.1 | Consistency tolerance: |g − lmc| ≤ ε means consistent |
| `max_iters_` | 10000 | Maximum sampling iterations per planning call |
| `goal_tolerance_` | 0.2 m | Unused in current loop; retained for future use |
| `obstacle_cost_threshold_` | 140 | Costmap cells ≥ this are treated as obstacles |

---

# 10. Correctness Analysis

The implementation captures the spirit of RRTx but has several deviations from the paper that affect correctness and performance.

---

## Bug 1 — rewireNeighbors early return fires on new node insertion

**Location**: [rrt_x.cpp:318](../plugins/rrt_x.cpp#L318), called from [rrt_x.cpp:196](../plugins/rrt_x.cpp#L196)

In the main loop, a new node is inserted with `g = lmc = min_cost` (initially consistent). Then `rewireNeighbors(new_idx)` is called, but its first line returns immediately:

```cpp
if (v.g - v.lmc <= epsilon_) return;
```

Since `g == lmc` at insertion, the function always exits without ever offering the new node as a parent to its neighbors. **Rewiring never happens during tree construction.**

This is the most significant correctness issue. RRTx (and RRT*) asymptotic optimality depends on rewiring: each new node should check whether any existing neighbor can reach the goal more cheaply by routing through it. Without rewiring, the algorithm degrades to RRT with best-parent selection — paths improve during construction only via fortuitous sampling, not systematic rewiring.

**Fix**: Either remove the early-return guard from `rewireNeighbors` and call it only when appropriate, or initialize the new node with `g = ∞` and `lmc = min_cost` so it is inconsistent, then let `reduceInconsistency` set `g = lmc` (the paper's intended flow).

---

## Bug 2 — start_proxy unset during initial tree construction

**Location**: [rrt_x.cpp:282](../plugins/rrt_x.cpp#L282), [rrt_x.cpp:116](../plugins/rrt_x.cpp#L116)

`start_proxy` is only assigned when `nodes_.size() > 1`:

```cpp
if (nodes_.size() > 1) {
    start_proxy = findStartProxy();
    updateObstacles();
}
```

During the very first call (building a new tree), `start_proxy` is never set. The member is also not initialized in the constructor. In practice it defaults to 0 (goal node index) if the object is zero-initialized by the OS/allocator, but this is not guaranteed.

Using index 0 (the goal) as `start_proxy` in `reduceInconsistency` means `botKey = {0, 0}` and `bot.g = 0`. The loop condition:

```
keyLess(topKey(), botKey)       → all queued nodes cost > 0, so false
bot.lmc - bot.g > epsilon_      → 0, false
bot.g == inf                    → false
bot.in_queue                    → false (root never queued)
```

All conditions are false, so **`reduceInconsistency` exits immediately during all of tree construction**, never propagating any lmc updates.

Combined with Bug 1 (rewireNeighbors also a no-op), the initial tree is built with no propagation whatsoever — equivalent to RRT with greedy parent selection.

**Fix**: Initialize `start_proxy` in the constructor (e.g., to `Node::INVALID_IDX`) and handle the invalid case in `reduceInconsistency`, or unconditionally set `start_proxy` at the top of the planning loop before the sampling starts.

---

## Bug 3 — radius_ uninitialized

**Location**: [rrt_x.h:103](../include/custom_nav/rrt_x.h#L103), [rrt_x.cpp:155](../plugins/rrt_x.cpp#L155)

`radius_` is declared as a `double` member but is not listed in the constructor's initializer list. The first `radius_search(new_pt, radius_)` call on the very first iteration uses an indeterminate value, which is undefined behavior.

`radius_ = getRadius()` is only called **after** `nodes_.push_back(new_node)`, so the first search uses the uninitialized value.

**Fix**: Add `radius_(step_length_)` to the constructor initializer list. A reasonable initial value is `step_length_` (the maximum ball size).

---

## Deviation — rewireNeighbors iterates nbr_init instead of nbr_running

**Location**: [rrt_x.cpp:323](../plugins/rrt_x.cpp#L323)

The paper's `rewireNeighbors` iterates over the **running** out-neighbors (those within the current ball radius). The implementation iterates `nbr_init` (all neighbors ever added):

```cpp
for (std::size_t nbr_index: v.nbr_init) { ... }
```

Using `nbr_init` is a superset of `nbr_running`, so no valid rewire opportunity is missed. However, it does unnecessary work for edges that have grown outside the current radius, and it diverges from the paper's specification. After calling `cullNeighbors`, the running list is up-to-date; using it would be more correct and more efficient.

---

## Deviation — findStartProxy selects nearest by distance, not by path cost

**Location**: [rrt_x.cpp:702](../plugins/rrt_x.cpp#L702)

The proxy for the robot's position should be the visible node that minimizes the estimated total path cost:

```
best = argmin over visible u:  lmc(u) + dist(robot, u)
```

The implementation instead picks the **closest visible node** (minimizing `dist(robot, u)` alone). This can select a node that is close but has a high lmc (long detour to goal), while ignoring a slightly farther node with a much shorter path.

`isConnected()` (used to check whether a path exists) does use the correct cost `n.g + dist`, so there is an inconsistency between how connectivity is checked and how the proxy is chosen.

---

## Minor — start_dist_threshold_ declared but never used

**Location**: [rrt_x.h:121](../include/custom_nav/rrt_x.h#L121)

```cpp
double start_dist_threshold_;
```

This member is declared and never assigned or read. It should be removed.

---

# 11. Comparison with Standard RRTx

| Aspect | Standard RRTx | This Implementation |
|---|---|---|
| Tree root | Goal | Goal ✓ |
| Cost direction | Toward goal | Toward goal ✓ |
| g and lmc per node | Yes | Yes ✓ |
| Key function (min(g,lmc), g) | Yes | Yes ✓ |
| Rewiring on insertion | Yes | **No (Bug 1)** |
| reduceInconsistency on insertion | Yes | **No (Bug 2)** |
| Shrinking ball radius formula | γ·(log n/n)^(1/d) | ✓ (d=2) |
| Dynamic obstacle handling | Yes | Partially ✓ |
| Blocked edge tracking (blocked_nbrs) | Yes | Yes ✓ |
| propagateDescendants | Yes | Yes ✓ |
| Neighbor culling | Yes | Yes ✓ |
| nbr_init and nbr_running | Yes | Yes ✓ |

---

# 12. Computational Complexity

Let n = number of nodes.

| Operation | Complexity |
|---|---|
| KD-tree insert | O(log n) avg |
| KD-tree nearest | O(log n) avg |
| KD-tree radius search | O(log n + k) |
| reduceInconsistency | O(k log n) per call, k = inconsistent nodes |
| propagateDescendants | O(subtree size) |
| Total per iteration | O(log n) amortized |

---

# 13. Differences from RRT and RRT*

| Property | RRT | RRT* | RRTx |
|---|---|---|---|
| Optimal | No | Asymptotically | Asymptotically |
| Dynamic replanning | No | No | Yes |
| Tree root | Start | Start | **Goal** |
| Cost per node | 1 (parent ptr) | g | g and lmc |
| Rewiring | No | Yes | Yes + queue repair |
| Obstacle response | Rebuild | Rebuild | In-place repair |
