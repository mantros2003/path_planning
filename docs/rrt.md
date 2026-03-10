# RRT Planner — Algorithm Implementation Notes

This document explains the algorithm-specific implementation details of the Rapidly-exploring Random Tree (RRT) planner implemented in this repository.

Common ROS planner infrastructure (plugin interface, costmap access, coordinate transforms, and path construction) is described in:

```
ros_global_planner_basics.md
```


# 1. Algorithm Overview

Rapidly-exploring Random Trees (RRT) are **sampling-based motion planning algorithms**.

Instead of searching every grid cell like Dijkstra or A*, RRT builds a **tree of feasible states** by repeatedly sampling random points and extending the tree toward them.

General procedure:

```
Initialize tree with start node

repeat
    sample random point
    find nearest node in tree
    steer toward sampled point
    if collision-free
        add new node to tree

until goal reached or iteration limit
```

RRT does **not guarantee optimal paths**, but it can find feasible paths quickly in complex spaces.

Properties:

| Property | Value                |
| -------- | -------------------- |
| Complete | Probabilistically    |
| Optimal  | No                   |
| Speed    | Fast in large spaces |

---

# 2. Tree Representation

The tree consists of:

```
nodes  → map cells
edges  → parent relationships
```

The implementation stores the tree using:

```
parent[node_index]
```

Initialization:

```cpp
std::vector<int> parents(height_ * width_, -1);
```

Meaning:

| Value | Meaning              |
| ----- | -------------------- |
| -1    | node not yet in tree |
| index | parent node          |

The **start node is its own parent**:

```
parents[start_index] = start_index
```

---

# 3. KD-Tree for Nearest Neighbor Search

A major computational bottleneck in RRT is the **nearest neighbor search**.

Naive implementation:

```
scan all nodes in tree
```

Complexity:

```
O(N)
```

This implementation uses a **KD-Tree** for spatial indexing.

```cpp
kdTree tree;
```

Each time a node is added:

```
tree.insert(point)
```

Nearest node query:

```
tree.nearest(point)
```

This reduces nearest neighbor search complexity to approximately:

```
O(log N)
```

which significantly improves scalability.

---

# 4. Hyperparameters

The planner defines several key hyperparameters.

```cpp
unsigned int max_iters = 100000;
double goal_bias = 0.1;
double step = 5.0;
double goal_tolerance = 5.0;
```



## Maximum Iterations

```
max_iters = 100000
```

Limits the number of sampling attempts.

Prevents infinite loops when no path exists.


## Goal Bias

```
goal_bias = 0.1
```

With probability 0.1:

```
sample = goal
```

Otherwise:

```
sample = random cell
```

Goal bias helps guide the tree toward the goal.

Without it, RRT may wander randomly.


## Step Size

```
step = 5 cells
```

Controls how far the tree extends toward a sampled point.

Large step:

```
faster exploration
less precise
```

Small step:

```
slower
more accurate
```


## Goal Tolerance

```
goal_tolerance = 5 cells
```

If a node reaches within this radius of the goal:

```
connect to goal
```

This helps terminate search earlier.

---

# 5. Random Sampling

Random sampling uses C++’s Mersenne Twister RNG.

```cpp
std::mt19937 rng(dev());
```

Two distributions are used.

### Uniform grid sampling

```cpp
std::uniform_int_distribution<unsigned int>
```

Used to select random map cells.


### Goal bias sampling

```cpp
std::uniform_real_distribution<>(0.0, 1.0)
```

If:

```
rand < goal_bias
```

Then the goal is sampled.

---

# 6. Sampling Loop

The algorithm runs:

```
for iters in max_iters
```

Each iteration performs one tree expansion attempt.


## Random Node Selection

Sample either:

```
goal node
```

or

```
random grid cell
```

```cpp
if (rand01(rng) < goal_bias)
    rand_index = goal_index;
else
    rand_index = uniform_dist(rng);
```


## Avoid Duplicate Nodes

If the sampled cell is already part of the tree:

```
skip iteration
```

Check:

```cpp
parents[rand_index] != -1
```


## Nearest Neighbor Search

Convert sampled node to coordinates:

```
(x_rand, y_rand)
```

Then query KD-Tree:

```
nearest_node = tree.nearest(rand_pt)
```

This finds the **closest node already in the tree**.


## Steering Function

RRT does not connect directly to the sampled point.

Instead, it **moves a limited distance toward it**.

Implemented by:

```
steer(from, to, step)
```

Steps:

```
compute direction vector
normalize
move by step distance
```

Implementation:

```
new = from + step * unit_direction
```

If the sampled point is within step distance:

```
return sampled point
```


## Boundary Clamping

After computing the new point, coordinates are clamped:

```
x ∈ [0, width-1]
y ∈ [0, height-1]
```

This prevents invalid grid indices.


## Collision Checking

Before adding a new edge, the planner checks whether the path intersects an obstacle.

The check uses:

```
base_local_planner::LineIterator
```

which discretizes the line segment between two cells.

Procedure:

```
for each cell along line
    if obstacle
        reject edge
```

Implementation:

```cpp
if (costmap_->getCost(x, y) >= LETHAL_OBSTACLE)
```

If any obstacle is detected:

```
extension rejected
```


## Adding a Node to the Tree

If the new node is valid:

```
insert node into KD-Tree
set parent pointer
```

```cpp
tree.insert(new_pt);
parents[new_index] = near_index;
```

This effectively **adds an edge to the RRT tree**.


## Goal Detection

The planner checks two goal conditions.

---

### Direct goal connection

```
new_node == goal
```

---

### Within tolerance

If:

```
distance(new_node, goal) < goal_tolerance
```

Then:

```
connect new_node → goal
```

```cpp
parents[goal_index] = new_index;
```

This terminates the search.

---

# 7. Path Reconstruction

Once the goal is reached, the path is reconstructed by following parent pointers.

```
goal → parent → parent → ... → start
```

Each node is converted to world coordinates and added to the plan.

Because the traversal starts from the goal, the path is reversed at the end:

```cpp
std::reverse(plan.begin(), plan.end());
```

Final path order:

```
start → ... → goal
```

---

# 8. Termination Conditions

The planner stops when:

```
goal reached
```

or

```
iteration limit reached
```

If the goal was not reached:

```
planner returns failure
```

---

# 9. Computational Complexity

Let:

```
N = number of nodes added to tree
```

Nearest neighbor search:

```
O(log N)
```

Total complexity:

```
O(N log N)
```

This is significantly faster than naive RRT implementations.

---

# 10. Differences from Grid Search Planners

| Property            | Dijkstra / A* | RRT            |
| ------------------- | ------------- | -------------- |
| Search type         | Graph search  | Sampling       |
| Nodes explored      | grid cells    | random samples |
| Optimality          | guaranteed    | not guaranteed |
| Speed in large maps | slower        | faster         |

RRT is particularly useful for **large or continuous spaces**.