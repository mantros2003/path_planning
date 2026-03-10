# Dijkstra Global Planner (ROS) — Implementation Notes

This document explains how the **Dijkstra path planner** is implemented as a ROS `nav_core::BaseGlobalPlanner` plugin. The planner computes the shortest path on a **2D costmap grid** using **Dijkstra’s algorithm with an 8-connected neighborhood**.



# Overview

The planner works in four stages:

1. **Initialization**
2. **Convert start/goal world coordinates → grid indices**
3. **Run Dijkstra search on the grid**
4. **Backtrack using parent pointers to generate the final path**

The planner treats the costmap as a **weighted graph**:

* Each grid cell = node
* Edges connect neighboring cells
* Edge weight depends on:

  * movement distance
  * obstacle proximity cost


# ROS Plugin Registration

```cpp
PLUGINLIB_EXPORT_CLASS(
    custom_planner::DijkstraPlanner,
    nav_core::BaseGlobalPlanner
)
```

This registers the planner so that the **ROS navigation stack can dynamically load it**.

The planner must implement the `nav_core::BaseGlobalPlanner` interface:

Required methods:

```cpp
void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
bool makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
);
```


# Initialization

### Purpose

The `initialize()` function retrieves the **costmap pointer and map dimensions**.

### Implementation

```cpp
void DijkstraPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
```

Steps:

1. Store pointer to the costmap
2. Extract map dimensions
3. Mark planner as initialized

```cpp
costmap_ = costmap_ros->getCostmap();
height_ = costmap_->getSizeInCellsY();
width_  = costmap_->getSizeInCellsX();
```

This converts the map into a **2D grid of size**

```
height_ × width_
```

Each cell stores a cost:

| Cost value | Meaning              |
| ---------- | -------------------- |
| 0          | Free space           |
| 1–252      | Inflation layer cost |
| 253        | Inscribed obstacle   |
| 254        | Lethal obstacle      |
| 255        | Unknown              |

---

# Planning Entry Point

Planning starts inside:

```cpp
bool makePlan(start, goal, plan)
```

The function must:

1. Compute the shortest path
2. Fill the `plan` vector with poses
3. Return `true` if successful


# Converting World Coordinates to Map Coordinates

The navigation stack provides start/goal in **world coordinates**.

These must be converted to **grid indices**.

```cpp
costmap_->worldToMap(wx, wy, mx, my)
```

Example:

```
World position (meters)
        ↓
Map cell coordinates
        ↓
Grid index
```

Then convert to a **1D index**:

```cpp
index = costmap_->getIndex(mx, my);
```

Internally:

```
index = y * width + x
```

This simplifies storage of nodes in **1D arrays**.


# Data Structures for Dijkstra

### Priority Queue (Min Heap)

```cpp
std::priority_queue<
    std::pair<double,int>,
    std::vector<std::pair<double,int>>,
    std::greater<>
> pq;
```

Stores:

```
(distance, node_index)
```

The smallest distance is always expanded first.


### Distance Array

```cpp
std::vector<double> dist(height_ * width_, INF);
```

Stores the **best known distance from the start** to each cell.

---

### Parent Array

```cpp
std::vector<int> parent(height_ * width_, -1);
```

Stores the **previous node in the shortest path tree**.

Used later for **path reconstruction**.


# Dijkstra Initialization

```cpp
dist[start_index] = 0;
pq.push({0, start_index});
```

This initializes the search frontier with the **start node**.

---

# Main Dijkstra Loop

```
while pq not empty
    pop node with smallest distance
    if goal reached → stop
    expand neighbors
```

Implementation:

```cpp
curr = pq.top().second;
pq.pop();
```



## Converting Index to Grid Coordinates

We recover `(x,y)` from the 1D index:

```cpp
ux = curr % width_;
uy = curr / width_;
```

This allows neighbor exploration.

---

## Neighbor Expansion (8-connected grid)

The planner checks all neighbors:

```
(-1,-1) (-1,0) (-1,1)
( 0,-1)  curr  (0,1)
( 1,-1) ( 1,0) (1,1)
```

Implementation:

```cpp
for dx in [-1,0,1]
    for dy in [-1,0,1]
```

Skip:

```
dx = 0 AND dy = 0
```


## Boundary Check

Ensure neighbors remain inside the map.

```cpp
if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
    continue;
```


## Obstacle Check

Cells marked as **lethal obstacles** are not traversable.

```cpp
if (cost >= costmap_2d::LETHAL_OBSTACLE)
    continue;
```

---

## Edge Cost Calculation

Movement cost depends on direction.

### Cardinal directions

```
cost = 1.0
```

### Diagonal directions

```
cost = √2 ≈ 1.414
```

Implementation:

```cpp
double weight = (dx == 0 || dy == 0) ? 1.0 : 1.414;
```

---

## Obstacle Proximity Penalty

Cells near obstacles receive higher cost.

```cpp
double obstacle_penalty = cost / 50.0;
```

This encourages paths to stay **away from obstacles**.

Total edge weight:

```
total_weight = movement_cost + obstacle_penalty
```

---

## Relaxation Step

Standard **Dijkstra relaxation**:

```
if dist[u] + weight < dist[v]
    update
```

Implementation:

```cpp
if (dist[curr] + total_weight < dist[v]) {
    dist[v] = dist[curr] + total_weight;
    parent[v] = curr;
    pq.push({dist[v], v});
}
```

This updates the shortest path tree.

---

## Goal Detection

The search stops when the goal is extracted from the priority queue.

```cpp
if (goal_index == curr)
    break;
```

Because Dijkstra guarantees:

> The first time a node is popped from the queue, the shortest path to it is known.

---

## Path Reconstruction

Once the goal is reached, the planner reconstructs the path by **traversing the parent pointers backwards**.

```
goal → parent → parent → ... → start
```

Implementation:

```cpp
for (curr = goal_index; curr != -1; curr = parent[curr])
```

---

## Convert Map Cells Back to World Coordinates

Each grid cell must be converted to a ROS pose.

```cpp
costmap_->mapToWorld(cx, cy, wx, wy);
```

Create a `PoseStamped` and add it to the path.

---

## Reverse Path Order

Because reconstruction happens from **goal → start**, the pose is inserted at the **front**:

```cpp
plan.insert(plan.begin(), p);
```

Final order:

```
start → ... → goal
```

---

# 20. Algorithm Complexity

Let:

```
N = number of cells = width × height
```

Time complexity:

```
O(N log N)
```

due to the priority queue operations.

Memory complexity:

```
O(N)
```

for `dist` and `parent` arrays.

---

# 21. Summary

The planner performs the following pipeline:

```
Start/Goal (world)
        ↓
Convert to map indices
        ↓
Run Dijkstra on costmap grid
        ↓
Store parents during search
        ↓
Backtrack from goal
        ↓
Convert to world poses
        ↓
Return path
```

---

# 22. Possible Improvements

### A* Search

Use a heuristic to guide the search and reduce explored nodes.

### Costmap Inflation Awareness

Better cost modeling based on inflation radius.

### Early Exit Conditions

Stop when distance exceeds a threshold.

### Path Smoothing

Apply post-processing to remove jagged edges.