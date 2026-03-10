# A* Planner — Algorithm Implementation

This document explains the algorithm-specific implementation details of the A* global planner used in this repository.

Common ROS planner infrastructure (plugin interface, costmap access, coordinate transforms, and path construction) is described in:

```
ros_global_planner_basics.md
```


# 1. Algorithm Overview

A* is an extension of **Dijkstra's algorithm** that introduces a **heuristic function** to guide the search toward the goal.

Instead of expanding nodes purely based on distance from the start, A* expands nodes according to:

```
f(n) = g(n) + h(n)
```

Where:

```
g(n) = cost from start → node
h(n) = heuristic estimate from node → goal
f(n) = total estimated path cost
```

Properties:

| Property             | Value                            |
| -------------------- | -------------------------------- |
| Complete             | Yes                              |
| Optimal              | Yes (if heuristic is admissible) |
| Faster than Dijkstra | Usually                          |

The heuristic biases exploration **toward the goal**, reducing the number of expanded nodes.

---

# 2. Heuristic Function

The implementation uses **Euclidean distance** as the heuristic.

```cpp
inline double heuristic(int sx, int sy, int gx, int gy) {
    return std::sqrt((sx - gx)*(sx - gx) + (sy - gy)*(sy - gy));
}
```

This corresponds to:

```
h(n) = √((x − x_goal)² + (y − y_goal)²)
```

### Why Euclidean distance?

Because the planner allows **diagonal motion** on the grid.

If movement were limited to cardinal directions, **Manhattan distance** would be more appropriate.

---

# 3. Graph Representation

The costmap grid is interpreted as a **weighted graph**.

```
cell → node
neighbor connection → edge
```

This planner uses an **8-connected grid**:

```
(-1,-1) (-1,0) (-1,1)
( 0,-1)  curr  (0,1)
( 1,-1) ( 1,0) (1,1)
```

Each node may connect to up to **8 neighbors**.

---

# 4. Data Structures

The planner maintains three key data structures.



## Parent Array

```
parent[node]
```

Stores the **previous node in the discovered path**.

Used later for **path reconstruction**.

Initialization:

```cpp
std::vector<int> parent(height_ * width_, -1);
```

Value `-1` indicates that the node has **not yet been reached**.


## Distance Array

```
dist[node]
```

Stores the **best known cost from the start to that node**.

```cpp
std::vector<double> dist(parent.size(), INF);
```

Initialization:

```
dist[start] = 0
```



## Open List (Priority Queue)

The **open list** stores nodes that are candidates for expansion.

```cpp
std::priority_queue<
    std::pair<double,int>,
    std::vector<std::pair<double,int>>,
    std::greater<>
> open_list;
```

Each element stores:

```
(f_cost, node_index)
```

The node with the **lowest estimated cost** is expanded first.

---

# 5. Initialization

The search begins by inserting the start node into the open list.

```cpp
dist[start_index] = 0;
open_list.push({0, start_index});
```

At this point:

```
g(start) = 0
h(start) = heuristic(start, goal)
```

---

# 6. Main Search Loop

The planner repeatedly expands nodes until:

```
goal found
OR
open list becomes empty
```

Pseudo-flow:

```
while open_list not empty
    node ← pop lowest f(n)
    if node == goal
        stop search
    expand neighbors
```

Implementation:

```cpp
curr = open_list.top().second;
open_list.pop();
```

---

## Node Expansion

The algorithm evaluates the **8 neighbors** of the current node.

```cpp
for dx in [-1,0,1]
    for dy in [-1,0,1]
```

Skip the current cell:

```
dx == 0 AND dy == 0
```

Neighbor coordinates:

```
nx = ux + dx
ny = uy + dy
```

---

## Traversability Check

Before expanding a neighbor, the planner checks whether the cell is valid.

### Boundary check

```
nx within map width
ny within map height
```

### Obstacle check

Cells with cost ≥ `LETHAL_OBSTACLE` are not traversable.

---

## Movement Cost

Movement cost depends on direction.

### Cardinal moves

```
cost = 1
```

### Diagonal moves

```
cost = √2 ≈ 1.414
```

Implementation:

```cpp
double weight = (dx == 0 || dy == 0) ? 1.0 : 1.414;
```

---

## Obstacle Penalty

To encourage paths away from obstacles, the planner adds a penalty proportional to the costmap value.

```cpp
double obstacle_penalty = cost;
```

Higher cost cells therefore become less attractive during search.

---

## Heuristic Cost

The heuristic estimates the remaining distance to the goal.

```cpp
double h_cost = heuristic(nx, ny, gx, gy);
```

This guides the search **toward the goal**.

---

## Total Cost Function

The implementation computes:

```
total_weight =
    movement_cost
  + obstacle_penalty
  + heuristic
```

Then the candidate distance becomes:

```
new_cost = g(curr) + total_weight
```

---

## Relaxation Step

If the new cost is better than the previously recorded cost:

```
update dist[v]
update parent[v]
push node into open list
```

Implementation:

```cpp
if (dist[curr] + total_weight < dist[v]) {
    dist[v] = dist[curr] + total_weight;
    parent[v] = curr;
    open_list.push({dist[v], v});
}
```

This ensures that the **best path discovered so far** is always tracked.

---

## Goal Detection

The search terminates when the goal node is removed from the open list.

```cpp
if (curr == goal_index)
    break;
```

At this point, the optimal path has been found (assuming an admissible heuristic).

---

# 7. Path Reconstruction

After reaching the goal, the planner reconstructs the path using the parent pointers.

```
goal → parent → parent → ... → start
```

For each node:

1. convert grid cell → world coordinates
2. create `PoseStamped`
3. insert into the plan

Because reconstruction starts from the goal, poses are inserted **at the beginning of the plan vector**.

Final path order:

```
start → ... → goal
```

---

# 8. Complexity

Let:

```
N = number of grid cells
```

Worst-case complexity:

```
O(N log N)
```

due to priority queue operations.

In practice, A* expands **far fewer nodes than Dijkstra** because the heuristic directs the search toward the goal.

---

# 9. Difference from Dijkstra

| Component        | Dijkstra          | A*            |
| ---------------- | ----------------- | ------------- |
| Priority         | g(n)              | g(n) + h(n)   |
| Heuristic        | none              | Euclidean     |
| Search direction | uniform expansion | goal-directed |
| Nodes explored   | large             | smaller       |