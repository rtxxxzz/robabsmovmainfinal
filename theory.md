# Theoretical Foundations of the Autonomous Navigation Pipeline

This document details the theoretical concepts powering the TurtleBot3 autonomous navigation pipeline. It explains the mechanics of SLAM, the concept of "Absolute Move," the mathematics of the P-Controller, and the algorithms used for exploration and path planning.

---

## 1. Simultaneous Localization and Mapping (SLAM)

SLAM is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. In this project, we use **SLAM Toolbox** in `online_async` mode.

### 1.1 The Odometry Drift Problem
TurtleBot3 estimates its position using **wheel odometry** (encoders that count wheel rotations) and an IMU. However, odometry accumulates error over time:
- Wheel slippage on the floor.
- Tiny variations in wheel diameter.
- Quantization errors in the encoders.

If you drive a robot in a perfect square based solely on odometry, the calculated position will gradually shift away from the true physical position. This is called **odometry drift**.

### 1.2 Scan Matching and the TF Tree
SLAM solves odometry drift by comparing the robot's real-time LiDAR scan (what the robot "sees") with the map it has built so far. This process is called **scan matching**.

In ROS 2, this correction is represented in the Transform (TF) tree:
```
map → odom → base_footprint → base_link → laser
```

1. **`odom → base_footprint`**: Published by the TurtleBot3 motor controller. This is the raw, drifting odometry.
2. **`map → odom`**: Published by SLAM Toolbox. This transform shifts the entire `odom` frame to perfectly align the robot's current LiDAR scan with the global `map`.

**Result**: Even though the robot's raw odometry drifts, the `map → odom` correction ensures that the robot's position relative to the `map` frame remains highly accurate.

---

## 2. Absolute Move: The Core Controller

"Absolute Move" refers to navigating to a specific `(x, y, heading)` coordinate in the world frame, rather than just driving "forward 1 meter" (a relative move).

The `absolute_move_node` is the low-level controller responsible for this. It uses a **Three-Phase Proportional Controller (P-Controller)** to drive the robot.

### 2.1 The Mathematics of the P-Controller

A Proportional Controller calculates an output command (motor speed) that is directly proportional to the current error (distance to the goal).

*   **Error ($e$)**: The difference between the desired state and the current state.
*   **Proportional Gain ($K_p$)**: A tuning parameter that determines how aggressively the robot reacts to the error.
*   **Control Output ($u$)**: $u = K_p \times e$

The controller operates in three distinct phases:

#### Phase 1: Rotate to Target
The robot rotates in place to face the target `(x, y)` coordinate.
*   **Target Angle ($\theta_{target}$)**: $\text{atan2}(y_{target} - y_{current}, x_{target} - x_{current})$
*   **Heading Error ($e_{heading}$)**: $\theta_{target} - \theta_{current}$ (normalized to $[-\pi, \pi]$)
*   **Command**: `angular.z` = $K_{p,angular} \times e_{heading}$
*   The phase completes when $|e_{heading}| < \text{heading\_tolerance}$.

#### Phase 2: Translate to Target
The robot drives forward while continuously correcting its heading.
*   **Distance Error ($e_{distance}$)**: $\sqrt{(x_{target} - x_{current})^2 + (y_{target} - y_{current})^2}$
*   **Linear Command**: `linear.x` = $K_{p,linear} \times e_{distance}$
*   **Angular Command**: `angular.z` = $K_{p,angular} \times e_{heading}$
*   **Speed Reduction to prevent arcing**: The linear speed is multiplied by $\max(0, \cos(e_{heading}))$. If the robot is knocked off course (e.g., heading error is 90°), the cosine becomes 0, stopping forward motion until the robot rotates back to face the target.

#### Phase 3: Final Heading Alignment
Once the `(x, y)` position is reached, the robot rotates to the user's requested final heading.
*   **Heading Error**: $\theta_{final\_target} - \theta_{current}$
*   **Command**: `angular.z` = $K_{p,angular} \times e_{heading}$

### 2.2 Reactive Obstacle Avoidance (LiDAR Gap-Finding)
During Phase 2, the `absolute_move_node` uses raw `/scan` data to avoid dynamic obstacles (like people walking by).
1. It checks a cone directly in front of the robot. If an obstacle is closer than `obstacle_distance`, the direct path is blocked.
2. It analyzes the full 360° LiDAR scan and identifies "gaps" (contiguous sections where the range is > `obstacle_distance`).
3. It selects the gap whose center is closest to the desired target heading.
4. It temporarily steers toward this gap at a reduced speed. Once the direct path is clear, it resumes normal Phase 2 control.

---

## 3. The Autonomous Pipeline

While `absolute_move` provides robust point-to-point driving, it lacks global awareness. If a wall is in the way, the reactive gap-finder might guide the robot into a corner.

The **Pipeline Orchestrator** solves this by adding high-level reasoning on top of the SLAM map. It consists of three components: Frontier Explorer, Path Planner, and the Orchestrator itself.

### 3.1 Frontier-Based Exploration
How does a robot know where to go when mapping an unknown room? It looks for "frontiers."

1. **The Occupancy Grid**: SLAM builds a grid where cells are Free (0), Occupied (100), or Unknown (-1).
2. **Frontier Detection**: A frontier cell is any Free cell that is adjacent to an Unknown cell. This represents the boundary of explored space.
3. **Clustering (BFS Flood-Fill)**: The algorithm groups adjacent frontier cells into clusters using a Breadth-First Search (BFS).
4. **Scoring**: Each cluster is scored based on its size and distance from the robot:
   $$ \text{Score} = \text{Size} \times \left( \frac{1}{1 + \text{Distance} \times \text{cost\_weight}} \right) $$
5. The robot navigates to the centroid of the highest-scoring cluster. As it moves, its LiDAR reveals the unknown space, destroying the frontier and creating new ones further out. This repeats until no frontiers remain.

### 3.2 A* Path Planning
When navigating to a distant goal (either a frontier or a user goal), the robot must find a collision-free path through the known map.

1. **Obstacle Inflation**: Because the robot has physical width, we cannot treat it as a single point. The planner "inflates" all obstacles on the grid by the robot's radius plus a safety margin using a distance transform. If the point-center of the robot avoids the inflated obstacles, the physical robot will clear the actual obstacles.
2. **A* Search Algorithm**: A* is an optimal graph traversal algorithm. It evaluates grid cells based on $f(n) = g(n) + h(n)$:
   *   $g(n)$: The cost to move from the start to the current cell (1.0 for straight moves, $\sqrt{2}$ for diagonal).
   *   $h(n)$: The heuristic. We use **Euclidean distance** to the goal. Because Euclidean distance never overestimates the true cost, the heuristic is *admissible*, guaranteeing A* finds the shortest possible path.
3. **Path Simplification**: The raw A* output follows the grid (resulting in zig-zag paths). The planner uses a **Bresenham line-of-sight** algorithm to remove redundant intermediate waypoints, resulting in a smooth, direct path consisting of minimal waypoints.

### 3.3 The Orchestrator Workflow
The `pipeline_orchestrator` ties everything together:
1. **Auto-Detect Map**: Checks if a saved map exists.
2. **Exploration Phase** (if no map): Repeatedly runs the Frontier Explorer, plans paths to the frontiers using A*, and sends the waypoints to the `absolute_move_node`.
3. **Map Saving**: Once exploration is complete, it saves the map to disk using `nav2_map_server`.
4. **Goal Execution Phase**: Accepts user goals (either from a YAML file or interactively via the `goal_input` tool), runs A* on the complete map, and executes the paths.

### 3.4 Interactive Goal Input Workflow
The user interacts with the system using two terminals:
1. **Terminal 1**: Runs the pipeline (`ros2 launch turtlebot3_absolute_move pipeline.launch.py`). This manages SLAM, Gazebo, and the `absolute_move_node`.
2. **Terminal 2**: Runs the interactive tool (`ros2 run turtlebot3_absolute_move goal_input`).
   * This tool acts as a dedicated Action Client.
   * **Crucially**, it is map-aware. It subscribes to the SLAM `/map` topic.
   * When the user enters coordinates, the `goal_input` tool runs the A* path planner locally.
   * It then sends the resulting collision-free waypoints to the `absolute_move_node` in Terminal 1. If no map is available, it gracefully falls back to sending a direct straight-line goal.
