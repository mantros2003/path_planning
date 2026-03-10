# Dijkstra Planner Implementation

This document explains the algorithm-specific parts of the Dijkstra global planner.

Common ROS planner infrastructure (plugin interface, costmap access, coordinate transforms, and path construction) is described in:

```
ros_global_planner_basics.md
```


# 1. Algorithm Overview

Dijkstra computes the **shortest path from a start node to all other nodes**.

It expands nodes in order of **increasing path cost**.

Properties:

- optimal
- complete
- no heuristic

Time complexity:

```

O(E log V)

```

# 2. Graph Representation

The costmap grid is treated as a graph:

```
cell → node
neighbor connection → edge
```

This implementation uses an **8-connected grid**.

```
(-1,-1) (-1,0) (-1,1)
( 0,-1)   cell  (0,1)
( 1,-1) ( 1,0) (1,1)
```


# 3. Data Structures

### Priority Queue

Stores nodes ordered by distance.

```cpp
std::priority_queue<
    std::pair<double,int>,
    std::vector<std::pair<double,int>>,
    std::greater<>
> pq;
````

Each entry:

```
(distance_from_start, node_index)
```

---

### Distance Array

```
dist[node]
```

Stores shortest distance from start.

Initialized with large values:

```
INF
```

---

### Parent Array

```
parent[node]
```

Stores previous node in shortest path tree.

Used to reconstruct the path.


# 4. Initialization

```
dist[start] = 0
push start into PQ
```

# 5. Node Expansion

Loop until queue empty.

```
pop node with smallest distance
```

If node is the goal → terminate search.


# 6. Neighbor Exploration

For each of the 8 neighbors:

1. check map bounds
2. check obstacle cost
3. compute movement cost
4. relax edge


# 7. Movement Cost

Two movement types:

### Cardinal

```
cost = 1
```

### Diagonal

```
cost = √2 ≈ 1.414
```

Implementation:

```cpp
(dx == 0 || dy == 0) ? 1.0 : 1.414
```


# 8. Obstacle Penalty

Cells near obstacles have higher cost.

```
obstacle_penalty = cost / 50
```

Final weight:

```
weight = movement_cost + obstacle_penalty
```

This keeps paths away from obstacles.


# 9. Relaxation Step

```
if dist[u] + weight < dist[v]
    update dist
    set parent
    push to queue
```

# 10. Termination

Search ends when:

```
goal node is popped from PQ
```

At that moment the shortest path is guaranteed.


# 11. Path Reconstruction

Follow parent pointers:

```
goal → parent → parent → ... → start
```

For each cell:

```
map → world
world → PoseStamped
```

Insert into plan.


# 12. Final Path

The planner returns a vector:

```
start → ... → goal
```

which is then used by the ROS local planner.