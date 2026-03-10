# ROS Global Planner Implementation Guide

This document explains the components common to all path planners implemented as
`nav_core::BaseGlobalPlanner` plugins in ROS.

All planners in this repository follow the same structure and differ only in the
search algorithm used.

---

# 1. ROS Navigation Stack Context

In the ROS navigation stack:

```

Goal → Global Planner → Global Path → Local Planner → Velocity Commands

````

The **global planner** computes a collision-free path on a 2D costmap.

Typical planners:

- Dijkstra
- A*
- RRT
- RRT*
- Hybrid A*

---

# 2. Plugin Architecture

Global planners are loaded dynamically using ROS `pluginlib`.

Every planner must register itself:

```cpp
PLUGINLIB_EXPORT_CLASS(
    custom_planner::PlannerName,
    nav_core::BaseGlobalPlanner
)
````

This allows the navigation stack to load the planner specified in:

```
move_base_params.yaml
```

Example:

```yaml
base_global_planner: custom_planner/DijkstraPlanner
```

---

# 3. Required Interface

Every planner must implement two functions.

### initialize()

```cpp
void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
```

Responsibilities:

* store pointer to costmap
* retrieve map dimensions
* perform one-time initialization

Example:

```cpp
costmap_ = costmap_ros->getCostmap();
width_ = costmap_->getSizeInCellsX();
height_ = costmap_->getSizeInCellsY();
```

---

### makePlan()

```cpp
bool makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan
);
```

Responsibilities:

1. Convert start/goal into map coordinates
2. Run path planning algorithm
3. Reconstruct the path
4. Fill the `plan` vector

Return:

```
true  → path found
false → no path exists
```

---

# 4. Costmap Representation

ROS represents the environment using a **2D grid costmap**.

Each cell stores a cost value:

| Value | Meaning            |
| ----- | ------------------ |
| 0     | free space         |
| 1–252 | inflation cost     |
| 253   | inscribed obstacle |
| 254   | lethal obstacle    |
| 255   | unknown            |

Cells with cost ≥ `LETHAL_OBSTACLE` should not be traversed.

Access cost:

```cpp
unsigned char cost = costmap_->getCost(x, y);
```

---

# 5. Coordinate Transformations

Planners operate in **map grid coordinates**, but ROS provides poses in **world coordinates**.

### World → Map

```cpp
costmap_->worldToMap(wx, wy, mx, my);
```

### Map → World

```cpp
costmap_->mapToWorld(mx, my, wx, wy);
```

---

# 6. Grid Indexing

Internally we often store nodes in a **1D array**.

Conversion:

```
index = y * width + x
```

ROS provides this helper:

```cpp
int index = costmap_->getIndex(x, y);
```

Recover coordinates:

```
x = index % width
y = index / width
```

---

# 7. Path Representation

The final path returned to ROS is a vector of poses:

```cpp
std::vector<geometry_msgs::PoseStamped> plan;
```

Each pose contains:

```
position (x,y)
orientation
frame_id
```

Typical workflow:

```
grid cell → world coordinates → PoseStamped → add to plan
```

---

# 8. Typical Planner Pipeline

Most planners follow the same structure:

```
1. Convert start/goal → grid cells
2. Initialize algorithm structures
3. Expand nodes
4. Stop when goal reached
5. Backtrack using parent pointers
6. Convert to world coordinates
7. Return path
```

---

# 9. Debugging Tools

Useful ROS logs:

```
ROS_INFO()
ROS_WARN()
ROS_ERROR()
```

Example:

```cpp
ROS_INFO("Start index: %d Goal index: %d", start_index, goal_index);
```