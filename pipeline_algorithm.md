# TurtleBot3 Autonomous Pipeline — Algorithm Deep-Dive

> **Source files analysed**
> `pipeline_orchestrator.py` · `frontier_explorer.py` · `path_planner.py` · `params_pipeline.yaml`

---

## 1. Pipeline Overview

The autonomous pipeline transforms TurtleBot3 from a single-goal mover into a fully autonomous exploration and navigation system. It chains three phases:

| Phase | Module | Algorithm | Purpose |
|-------|--------|-----------|---------|
| **A** | `FrontierExplorer` | Frontier-based BFS exploration | Build a complete SLAM map autonomously |
| **B** | `PathPlanner` | A* with obstacle inflation | Find collision-free paths on the map |
| **C** | `PipelineOrchestrator` | Waypoint sequencing + replanning | Execute paths via the absolute_move action |

The pipeline sits **on top of** the existing `absolute_move_node` — it does not modify it. The `absolute_move_node` remains responsible for low-level motion control, gap-based obstacle avoidance, and stuck recovery.

---

## 2. Frontier-Based Exploration

### 2.1 What Is a Frontier?

A **frontier** is the boundary between explored (known) and unexplored (unknown) space on the occupancy grid. By driving toward frontiers, the robot systematically reveals the entire environment.

```
Map visualisation (top-down):

  ████████████████
  █              █
  █    KNOWN     █  ← free cells (value = 0)
  █              █
  █  ░░░░░░░░░░  █  ← FRONTIER (free cells adjacent to unknown)
  █  ??????????  █
  █  ??????????  █  ← UNKNOWN cells (value = -1)
  █  ??????????  █
  ████████████████   ← occupied cells (value ≥ 50)
```

### 2.2 Frontier Detection Algorithm

```
Input: OccupancyGrid (H × W array, values: -1=unknown, 0=free, ≥50=occupied)
Output: Boolean mask (H × W) marking frontier cells

For each cell (r, c):
  if data[r, c] == 0:                          # must be free
    for each of 8 neighbours (nr, nc):
      if data[nr, nc] == -1:                    # any unknown neighbour?
        frontier[r, c] = True
        break
```

The implementation uses **vectorised numpy operations** for efficiency — shifting the array by each of the 8 neighbour offsets and performing element-wise comparisons, avoiding slow Python loops over every cell.

### 2.3 Frontier Clustering (BFS Flood-Fill)

Individual frontier cells are grouped into **clusters** using BFS:

```
1. For each unvisited frontier cell:
   a. Start BFS from this cell
   b. Add all connected frontier cells (8-connectivity) to the cluster
   c. Mark them as visited
2. Discard clusters with < min_frontier_size cells
```

This produces a list of discrete frontier regions, each representing a distinct area of unexplored space.

### 2.4 Frontier Scoring

Each cluster is scored to determine which frontier to explore next:

```
score = cluster_size × (1 / (1 + distance × cost_weight))
```

| Factor | Effect |
|--------|--------|
| `cluster_size` | Larger frontiers = more unexplored area to reveal |
| `distance` | Euclidean distance from robot to cluster centroid |
| `cost_weight` | Balances size vs. proximity (0.5 default) |

The centroid of the highest-scoring cluster becomes the next exploration goal.

### 2.5 Exploration Termination

The exploration phase ends when **any** of these conditions is met:

| Condition | Default | Purpose |
|-----------|---------|---------|
| No valid frontiers for `no_frontier_patience` consecutive cycles | 3 | Map is complete |
| `exploration_timeout` exceeded | 300s (5 min) | Safety timeout |

The `get_exploration_progress()` method reports the fraction of cells that are known (free or occupied), providing a real-time coverage metric.

---

## 3. A* Path Planning

### 3.1 Obstacle Inflation

Before running A*, all occupied cells are **inflated** by a radius equal to the robot's physical radius plus a safety margin:

```
inflation_radius = robot_radius + safety_margin
                 = 0.14 m      + 0.04 m
                 = 0.18 m (default)
```

This is implemented using **scipy's distance transform**:

```python
dist = distance_transform_edt(~obstacle_mask)
inflated = dist < inflation_radius_in_cells
```

This converts the problem from "avoid collisions with the polygon-shaped robot" to "find a path for a point robot on the inflated map" — which A* can solve directly.

If `unknown_as_free` is False (default), unknown cells are also treated as obstacles during planning — the robot won't plan paths through unexplored territory.

### 3.2 A* Algorithm

The A* implementation uses:

| Property | Value |
|----------|-------|
| **Grid connectivity** | 8-connected (cardinal + diagonal) |
| **Cardinal move cost** | 1.0 |
| **Diagonal move cost** | √2 ≈ 1.414 |
| **Heuristic** | Euclidean distance (admissible + consistent) |
| **Data structures** | Binary heap (heapq), numpy arrays for g_score and closed set |

```
Standard A* pseudocode:

open_set = priority queue with start node (f=0)
g_score[start] = 0

while open_set not empty:
    current = pop node with lowest f
    if current == goal: reconstruct and return path
    
    for each 8-connected neighbour:
        if blocked or closed: skip
        tentative_g = g_score[current] + move_cost
        if tentative_g < g_score[neighbour]:
            g_score[neighbour] = tentative_g
            f = tentative_g + euclidean(neighbour, goal)
            push neighbour to open_set
```

**Why Euclidean heuristic?** It's both admissible (never overestimates) and consistent (satisfies triangle inequality), guaranteeing A* finds the optimal path without re-expanding nodes.

### 3.3 Unreachable Goals

If the goal falls inside an inflated obstacle:

1. **BFS outward** from the goal cell to find the nearest free cell (within 50 cells radius)
2. Plan to that cell instead
3. If no free cell is found within radius → return `None` (no path)

### 3.4 Path Simplification (Line-of-Sight Pruning)

Raw A* output follows the grid, producing many unnecessary waypoints along straight segments. The simplification algorithm:

```
simplified = [path[0]]  # keep start
i = 0

while i < len(path) - 1:
    # Find farthest point with clear line-of-sight from path[i]
    farthest = i + 1
    for j in range(len(path)-1, i+1, -1):
        if line_of_sight(path[i], path[j]):
            farthest = j
            break
    simplified.append(path[farthest])
    i = farthest
```

**Line-of-sight check** uses **Bresenham's line algorithm** on the inflated grid — it traces the discrete cells along the line and checks that none are obstacles.

### 3.5 Waypoint Spacing

After simplification, waypoints closer than `waypoint_spacing` (default 0.3 m) are removed, keeping the first and last. This prevents the absolute_move_node from receiving redundant micro-goals.

---

## 4. Pipeline Execution

### 4.1 Waypoint Sequencing

For each goal, the orchestrator:

1. Plans an A* path from the current pose to the goal
2. Simplifies the path into waypoints
3. Sends each waypoint to `absolute_move_node` via the AbsoluteMove action
4. For intermediate waypoints: heading = direction toward the next waypoint
5. For the final waypoint: heading = the user-specified heading

### 4.2 Replanning Strategy

If a waypoint fails (the absolute_move result has `success=False`):

```
for attempt in 1..max_replan_attempts:
    1. Wait for updated /map (SLAM may have new data after the robot moved)
    2. Replan from current position to the original goal
    3. Execute the new path
    4. If successful → done
    5. If failed → increment attempt counter
If all attempts fail → skip this goal, move to next
```

### 4.3 Exploration During Navigation

During the exploration phase, the pipeline uses the A* planner to reach frontier goals. The path planner uses the **live SLAM map** (subscribed to `/map`), so it automatically incorporates newly discovered obstacles as the robot explores.

---

## 5. System Integration

### 5.1 ROS 2 Topic Flow

```
/map (OccupancyGrid)
  ├── FrontierExplorer reads this for frontier detection
  ├── PathPlanner reads this for A* planning
  └── Published by SLAM Toolbox (online_async)

/odom (Odometry)
  ├── PipelineOrchestrator reads current position for planning
  ├── absolute_move_node reads for control
  └── Published by TurtleBot3 base (Gazebo or real hardware)

/scan (LaserScan)
  ├── absolute_move_node reads for gap-finding obstacle avoidance
  ├── SLAM Toolbox reads for scan matching + map building
  └── Published by TurtleBot3 LiDAR

/cmd_vel (Twist)
  └── absolute_move_node publishes → TurtleBot3 base subscribes

/absolute_move_node/absolute_move (AbsoluteMove action)
  └── PipelineOrchestrator sends goals → absolute_move_node executes

/planned_path (nav_msgs/Path) — RViz visualisation
/frontiers (MarkerArray) — RViz visualisation
/exploration_goal (Marker) — RViz visualisation
```

### 5.2 Threading Model

The `PipelineOrchestrator` uses a `MultiThreadedExecutor(num_threads=4)` with a `ReentrantCallbackGroup`. The main pipeline logic runs on a separate thread from the ROS callbacks:

```
Thread 1 (spin): handles /odom, /map subscriptions
Thread 2 (spin): handles action client callbacks
Thread 3 (main): runs the pipeline (explore → plan → execute)
Thread 4 (spare): available for concurrent callbacks
```

Shared state (`_x`, `_y`, `_yaw`, `_latest_map`) is protected by a `threading.Lock`.

### 5.3 Simulation vs. Hardware

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Launch | `pipeline.launch.py` starts Gazebo + everything | `pipeline.launch.py mode:=hardware` (bringup runs on SBC) |
| SLAM | Always on (default) | Always on (needed for exploration) |
| `use_sim_time` | `true` | `false` |
| absolute_move params | `params_sim.yaml` | `params_hw.yaml` |
| Pipeline params | Same `params_pipeline.yaml` | Same |
| `exploration_timeout` | 300s recommended | 600s recommended (increase for large areas) |

---

## 6. Comparison: This Pipeline vs. Nav2

| Feature | This Pipeline | Nav2 |
|---------|--------------|------|
| **Path planning** | A* on inflated occupancy grid | Multiple planners (NavFn, Smac, Theta*) |
| **Local avoidance** | Gap-finding on LiDAR (reactive) | DWB/MPPI controller with local costmap |
| **Exploration** | Frontier-based (built-in) | Requires separate package (explore_lite) |
| **Recovery** | Back-up + rotate toward gap | Configurable recovery behaviours (spin, backup, wait) |
| **Complexity** | ~1500 lines of Python, zero external deps | Full behaviour tree + multiple plugins |
| **Setup** | `colcon build` — works immediately | Extensive configuration (params, behaviour tree XML) |
| **Best for** | Education, simple environments, TurtleBot3 | Production, complex environments, multiple robots |

This pipeline is intentionally **lightweight and self-contained**. For complex multi-room environments or production deployments, Nav2 with its costmap layers, behaviour trees, and sophisticated planners is the better choice.

---

## 7. Parameter Tuning Guide

### Exploration Speed

| Symptom | Adjustment |
|---------|------------|
| Exploration is too slow | Increase `max_linear_speed` in sim/hw params |
| Robot revisits areas | Increase `min_frontier_size` to skip small pockets |
| Robot explores far-away frontiers first | Increase `frontier_cost_weight` |
| Exploration stops too early | Increase `no_frontier_patience` |

### Path Quality

| Symptom | Adjustment |
|---------|------------|
| Path too close to walls | Increase `inflation_radius` |
| Path not found in narrow corridors | Decrease `inflation_radius` |
| Too many waypoints (slow) | Ensure `path_simplification: true`, increase `waypoint_spacing` |
| Path goes through unexplored area | Ensure `unknown_as_free: false` |

### Execution Reliability

| Symptom | Adjustment |
|---------|------------|
| Robot gives up too easily | Increase `max_replan_attempts` |
| Individual moves fail | Tune `absolute_move_node` params (see main README) |
| Robot can't reach tight goals | Decrease `waypoint_reached_tolerance` |

---

## 8. Known Limitations

1. **No costmap layers**: Unlike Nav2, the planner uses a single binary inflation — no gradual cost decay near obstacles.

2. **No global exploration strategy**: Frontier scoring is greedy (best local frontier). A TSP-style tour optimisation would visit all frontiers more efficiently.

3. **No dynamic obstacle handling in planner**: The A* plan is static — if an obstacle moves after planning, the robot relies on `absolute_move_node`'s reactive gap-finding for avoidance.

4. **Single-threaded planning**: A* runs synchronously. For very large maps (>500×500 cells), consider the JPS (Jump Point Search) optimisation.

5. **Map saving requires nav2_map_server**: The `map_saver_cli` tool is part of the Nav2 stack. Install with `sudo apt install ros-humble-nav2-map-server`.

6. **Exploration in simulation empty_world**: An empty world has no features for SLAM to match against, so the map may not build correctly. Use `turtlebot3_world` or `turtlebot3_house` for meaningful exploration testing.
