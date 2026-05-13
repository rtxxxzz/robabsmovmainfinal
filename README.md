# TurtleBot3 Absolute Move — ROS 2 Humble

Move a TurtleBot3 Burger to an **absolute (x, y, heading)** pose in the `odom` frame.  
Works identically in **Gazebo simulation** and on **real hardware** via the standard TurtleBot3 remote-PC workflow.

**SLAM Toolbox is enabled by default** in the simulation launch. It publishes the `map → odom` TF correction that keeps odometry accurate across the entire run — no extra configuration needed.

---

## Table of Contents

1. [Overview](#overview)
2. [Theoretical Foundations](theory.md) — Deep-dive into SLAM, absolute move math, and the autonomous pipeline algorithms
3. [Architecture](#architecture)
4. [SLAM Integration](#slam-integration)
5. [Absolute Move vs. Relative Move](#absolute-move-vs-relative-move)
6. [Prerequisites](#prerequisites)
7. [Build](#build)
8. [Simulation Mode](#simulation-mode)
9. [Hardware Mode](#hardware-mode)
10. [Action Interface Reference](#action-interface-reference)
11. [CLI Client Usage](#cli-client-usage-goal_input)
12. [Configuration & Tuning](#configuration--tuning)
13. [Coordinate Frames & Odometry](#coordinate-frames--odometry)
14. [Validation Checklist](#validation-checklist)
15. [Troubleshooting](#troubleshooting)
16. [Known Limitations](#known-limitations)
17. [Autonomous Pipeline](#autonomous-pipeline)

---

## Overview

This package provides a **three-phase proportional controller** with **reactive obstacle avoidance** that drives a TurtleBot3 Burger to an absolute pose in the odometry frame:

| Phase | Behavior |
|-------|----------|
| **1 — Rotate to target** | Turn in place to face the goal (x, y) |
| **2 — Translate** | Drive toward the goal with gap-based LiDAR obstacle avoidance |
| **3 — Align heading** | Turn in place to the desired final heading |

**Key features:**
- **LiDAR gap-finding**: when an obstacle blocks the direct path, the node scans the full LiDAR for the best open gap closest to the goal direction and steers through it
- **Stuck recovery**: if the robot makes no progress for `stuck_timeout` seconds, it backs up and rotates toward the best gap, retrying up to `max_recovery_attempts` times
- **Best-effort reaching**: if the goal is unreachable (e.g. inside a wall), the robot stops at the closest achievable position and still aligns its heading
- **Tilt detection**: in Gazebo, detects chassis tipping and calls the Gazebo API to restore the robot upright and continue
- **SLAM integration**: SLAM Toolbox runs by default, keeping the odom frame drift-corrected via the `map→odom` TF

The controller is exposed as a **ROS 2 action server**, making it usable from:
- The included interactive CLI client
- Any ROS 2 action client (Python, C++, CLI)
- Your own autonomy stack

---

## Architecture

### With SLAM (default)

```
┌──────────────────┐         ┌───────────────────────────────┐
│  CLI Client or   │ action  │  absolute_move_node           │
│  Custom Client   ├────────►│                               │
│                  │         │  • /odom subscriber (pose)    │
└──────────────────┘         │  • /scan subscriber (LiDAR)  │
                             │  • /cmd_vel publisher        │
                             │  • 3-phase P-controller      │
                             │  • Gap-finding obstacle avoid│
                             │  • Stuck recovery + tilt det.│
                             └───────┬───────────────────────┘
                                     │ cmd_vel
                                     ▼
                             ┌───────────────────┐
                             │  TurtleBot3 Base  │
                             │  (Gazebo or HW)   │
                             └───────┬───────────┘
                                     │ /scan (LiDAR)  +  /odom
                                     ▼
                             ┌───────────────────────────────┐
                             │  SLAM Toolbox (online async)  │
                             │  • Builds /map in real time   │
                             │  • Publishes map→odom TF      │
                             └───────────────────────────────┘
```

TF tree with SLAM running:
```
map ──[SLAM correction]──► odom ──[wheel odom]──► base_footprint ──► base_link
                                       ↑
                           absolute_move_node reads /odom here
```

**Packages:**

| Package | Type | Purpose |
|---------|------|---------|
| `turtlebot3_absolute_move_interfaces` | CMake | `AbsoluteMove.action` definition |
| `turtlebot3_absolute_move` | Python (ament_python) | Node, client, launch files, config |

---

## SLAM Integration

### Why SLAM is the Default

Without SLAM, the robot uses **dead reckoning** (wheel-encoder integration). This accumulates drift over time:

| Move distance | Typical odom error |
|--------------|-------------------|
| < 2 m        | < 1 cm — excellent |
| 2–5 m        | 1–3 cm — usually OK |
| > 5 m        | Drift accumulates — SLAM recommended |

With **SLAM Toolbox** running, LiDAR scan matching continuously corrects the odom frame so absolute goals remain accurate for the full session, regardless of distance.

### How It Works — No Node Changes Required

The absolute_move_node continues to subscribe to `/odom` and publish to `/cmd_vel` exactly as before. SLAM Toolbox works **transparently** by publishing a `map → odom` TF transform that keeps the odom frame calibrated:

```
Without SLAM:
  odom origin = where robot booted (drifts over time)

With SLAM:
  odom origin = continuously corrected by LiDAR scan matching
  absolute goals in odom frame ≈ true world positions
```

### SLAM Mode Reference

The `params_slam.yaml` config uses **online_async** mode:

| Mode | What it does |
|------|-------------|
| `mapping` (default) | Builds a new map every run; no prior map needed |
| `localization` | Loads a saved map and localises against it |

Switch to `localization` mode after saving a map with `ros2 run slam_toolbox lifelong_slam_toolbox_node` or the map saver tool.

### Saving the Map

```bash
# Save the current map to disk:
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# This creates my_map.pgm and my_map.yaml
```

### SLAM Topic / TF Summary

| Published | Type | Description |
|-----------|------|-------------|
| `/map` | `OccupancyGrid` | 2-D occupancy map (5 cm resolution) |
| `/map_metadata` | `MapMetaData` | Map dimensions and origin |
| TF `map → odom` | Transform | Drift correction (50 Hz) |

| Subscribed | Type | Description |
|-----------|------|-------------|
| `/scan` | `LaserScan` | TurtleBot3 LiDAR (360°, 3.5 m range) |
| `/odom` | `Odometry` | Wheel odometry for motion prediction |

---

## Absolute Move vs. Relative Move

| | Absolute Move (this package) | Relative Move |
|---|---|---|
| **Goal** | Move to **(x, y, θ) in the odom frame** | Move **(Δx, Δy, Δθ) from current pose** |
| **Reference** | Odom frame origin (where the robot booted) | Current pose at command time |
| **Repeatability** | Same goal → same odom-frame position | Same goal → different odom-frame position |
| **Use case** | Waypoint navigation, return-to-home | "Go forward 1 m", "Turn 90°" |

> **Important:** With SLAM enabled (default), the odom frame stays calibrated against the map — absolute moves are accurate for the entire session, not just from boot.

---

## Prerequisites

### Common (Remote PC)

- **Ubuntu 22.04** (Jammy)
- **ROS 2 Humble** desktop install (`ros-humble-desktop`)
- TurtleBot3 packages:
  ```bash
  sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
  ```
- **SLAM Toolbox** (required for the default `slam:=true` mode):
  ```bash
  sudo apt install ros-humble-slam-toolbox
  ```
- Shell environment:
  ```bash
  echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
  source ~/.bashrc
  ```

### Hardware Only (TurtleBot3 SBC / Raspberry Pi)

- **Ubuntu 22.04** Server on the Pi
- ROS 2 Humble base + TurtleBot3 bringup
- Network configured per the [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- `ROS_DOMAIN_ID` matching on both PC and SBC

---

## Build

```bash
# 1. Clone/copy this workspace
cd /path/to/robabsmov

# 2. Install dependencies (includes slam_toolbox)
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install

# 4. Source
source install/setup.bash
```

> **Tip:** Add `source ~/robabsmov/install/setup.bash` to your `~/.bashrc` for convenience.

---

## Simulation Mode

### Terminal 1 — Launch Gazebo + SLAM + Node

```bash
export TURTLEBOT3_MODEL=burger
source install/setup.bash

# Default: SLAM enabled (slam:=true)
ros2 launch turtlebot3_absolute_move simulation.launch.py
```

This starts:
- Gazebo with an empty world and a TurtleBot3 Burger at origin (0, 0)
- **SLAM Toolbox** (online async, mapping mode) — publishes `/map` and `map → odom` TF
- The `absolute_move_node` with simulation parameters (`use_sim_time: true`)

### Launch Options

```bash
# SLAM disabled (raw odometry only):
ros2 launch turtlebot3_absolute_move simulation.launch.py slam:=false

# TurtleBot3 World (hexagonal obstacle course):
ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_world x_pose:=-2.0 y_pose:=-0.5

# House environment:
ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_house x_pose:=-2.0 y_pose:=-0.5

# DQN reinforcement learning stages:
ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_dqn_stage1
ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_dqn_stage4

# No Gazebo (Gazebo is already running separately):
ros2 launch turtlebot3_absolute_move simulation.launch.py launch_gazebo:=false
```

#### Available Worlds

| World Name | Description | Recommended Spawn |
|------------|-------------|-------------------|
| `empty_world` | Flat plane, no obstacles (default) | `x_pose:=0.0 y_pose:=0.0` |
| `turtlebot3_world` | Hexagonal obstacle course | `x_pose:=-2.0 y_pose:=-0.5` |
| `turtlebot3_house` | Indoor house with rooms & furniture | `x_pose:=-2.0 y_pose:=-0.5` |
| `turtlebot3_dqn_stage1` | Simple enclosed area (RL training) | `x_pose:=0.0 y_pose:=0.0` |
| `turtlebot3_dqn_stage2` | Enclosed area with 4 obstacles | `x_pose:=0.0 y_pose:=0.0` |
| `turtlebot3_dqn_stage3` | Enclosed area with 6 obstacles | `x_pose:=0.0 y_pose:=0.0` |
| `turtlebot3_dqn_stage4` | Complex enclosed area with many obstacles | `x_pose:=0.0 y_pose:=0.0` |

> **Tip:** For `turtlebot3_world` and `turtlebot3_house`, always set `x_pose:=-2.0 y_pose:=-0.5` to avoid spawning the robot inside a wall. The default (0, 0) works for `empty_world` and DQN stages.

### Terminal 2 — (Optional) RViz

```bash
source install/setup.bash
ros2 launch turtlebot3_absolute_move rviz.launch.py use_sim_time:=true
```

> **Tip:** Add the `/map` topic in RViz to see the live SLAM map building.

### Terminal 3 — Send Goals

```bash
source install/setup.bash

# Interactive mode:
ros2 run turtlebot3_absolute_move absolute_move_client

# Or single-shot:
ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 1.0 0.5 90
```

---

## Hardware Mode

### Step 1 — Start the Robot (on the TurtleBot3 SBC)

SSH into the Raspberry Pi and launch bringup:

```bash
# From Remote PC:
ssh ubuntu@<TURTLEBOT3_IP>

# On the SBC:
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```

> Leave this terminal open. The robot is now publishing `/odom` and `/scan`, and subscribing to `/cmd_vel`.

### Step 2 — Launch the Pipeline (on Remote PC)

The `hardware.launch.py` now perfectly mirrors the simulation pipeline, running SLAM, RViz, and the pipeline orchestrator by default.

```bash
export TURTLEBOT3_MODEL=burger
source install/setup.bash

# Full pipeline (explore + interactive goals):
ros2 launch turtlebot3_absolute_move hardware.launch.py

# With a predefined goals file:
ros2 launch turtlebot3_absolute_move hardware.launch.py goals_file:=goals.yaml

# Just absolute_move + SLAM (no pipeline orchestrator):
ros2 launch turtlebot3_absolute_move hardware.launch.py pipeline:=false
```

### Step 3 — Send Goals (on Remote PC, new terminal)

```bash
source install/setup.bash
ros2 run turtlebot3_absolute_move goal_input
```

### Step 4 — (Optional) RViz

```bash
source install/setup.bash
ros2 launch turtlebot3_absolute_move hardware.launch.py rviz:=true
```

### Network Checklist

| Item | How to verify |
|------|---------------|
| Ping robot from PC | `ping <TURTLEBOT3_IP>` |
| Same `ROS_DOMAIN_ID` | `echo $ROS_DOMAIN_ID` on both machines |
| Topics visible | `ros2 topic list` should show `/odom`, `/cmd_vel`, `/scan` |
| Odom data flowing | `ros2 topic echo /odom --once` |
| LiDAR data flowing | `ros2 topic echo /scan --once` |

---

## Action Interface Reference

**Action name:** `~/absolute_move`  
**Full topic (default):** `/absolute_move_node/absolute_move`

### Goal

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `target_x` | `float64` | meters | X in odom frame |
| `target_y` | `float64` | meters | Y in odom frame |
| `target_heading` | `float64` | **radians** | Heading in odom frame, range [-π, π] |

### Feedback

| Field | Type | Description |
|-------|------|-------------|
| `current_x` | `float64` | Current X |
| `current_y` | `float64` | Current Y |
| `current_heading` | `float64` | Current heading (rad) |
| `distance_remaining` | `float64` | Distance to target (m) |
| `heading_error` | `float64` | Heading error (rad) |
| `phase` | `string` | `rotate_to_target` / `translate` / `final_heading` |

### Result

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | `true` if goal was reached |
| `message` | `string` | Human-readable status |
| `final_x/y/heading` | `float64` | Achieved pose |
| `position_error` | `float64` | Residual position error (m) |
| `heading_error` | `float64` | Residual heading error (rad) |

### Sending Goals from the CLI (without the client node)

```bash
# Using ros2 action send_goal:
ros2 action send_goal /absolute_move_node/absolute_move \
  turtlebot3_absolute_move_interfaces/action/AbsoluteMove \
  "{target_x: 1.0, target_y: 0.5, target_heading: 1.5708}" \
  --feedback
```

---

## CLI Client Usage (`goal_input`)

The `goal_input` tool acts as a dedicated action client. It accepts headings in **degrees** for convenience (converted to radians internally).

> **NEW:** The interactive client is now **map-aware**. It subscribes to the `/map` and `/odom` topics, and runs A* path planning on the SLAM occupancy grid to find the optimal collision-free path to your goal. It sends a sequence of waypoints to `absolute_move_node`. If no map is available, it gracefully falls back to sending a direct straight-line goal.

### Interactive Mode

```bash
# In a separate terminal while the simulation/robot is running:
$ ros2 run turtlebot3_absolute_move goal_input

Connecting to absolute_move action server...
  ✓ Action server connected
Waiting for /odom...
  ✓ Odom received — robot at (-1.96, 0.99, 2.2°)
Waiting for /map...
  ✓ Map received — 112×103 cells @ 0.05 m/cell

============================================================
  Interactive Absolute Move (A* Path Planning)
============================================================
Enter goal coordinates to move the robot.
The robot will plan an optimal path using A* on the SLAM map.

Format:  x y heading_degrees
  Example: 1.0 0.5 90
Type "quit" or Ctrl+C to exit.
============================================================

Goal 1 (x y heading_deg): 1.0 0.5 90
  → Planning path to (1.00, 0.50, 90.0°)...
  ✓ A* path planned: 3 waypoints
    Waypoint 1/3: (-1.18, 0.75, -6.5°)
    Waypoint 2/3: (0.34, 0.58, -3.1°)
    Waypoint 3/3: (1.00, 0.50, 90.0°)
  ✓ Goal reached (0.979, 0.500)
```

### Single-Shot Mode

```bash
ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 2.0 1.0 45
```

### Example Goals

| Goal | Meaning |
|------|---------| 
| `1.0 0.0 0` | Move 1 m forward along X, face 0° |
| `0.0 1.0 90` | Move 1 m along Y, face 90° (left) |
| `1.0 1.0 -45` | Move to (1, 1), face -45° |
| `0.0 0.0 0` | Return to origin, face forward |

---

## Configuration & Tuning

### Config Files

| File | Used by | Purpose |
|------|---------|---------|
| `config/params_sim.yaml` | `simulation.launch.py` | Gazebo simulation gains & tolerances |
| `config/params_hw.yaml` | `hardware.launch.py` | Real-hardware gains & tolerances |
| `config/params_slam.yaml` | Both launches (when `slam:=true`) | SLAM Toolbox settings |

### Absolute Move Parameter Reference

**Core control:**

| Parameter | Sim | HW | Unit | Description |
|-----------|-----|-----|------|-------------|
| `max_linear_speed` | 0.12 | 0.18 | m/s | Forward speed cap |
| `max_angular_speed` | **1.0** | 2.0 | rad/s | Rotation speed cap |
| `position_tolerance` | 0.05 | 0.03 | m | Goal position threshold |
| `heading_tolerance` | 0.05 | 0.03 | rad | Goal heading threshold |
| `kp_linear` | 1.0 | 1.0 | — | Proportional gain for linear velocity |
| `kp_angular` | **1.5** | 3.0 | — | Proportional gain for angular velocity |
| `control_rate` | 30.0 | 30.0 | Hz | Control loop frequency |

**Stuck detection & recovery:**

| Parameter | Sim | HW | Unit | Description |
|-----------|-----|-----|------|-------------|
| `stuck_timeout` | 5.0 | 8.0 | s | Time before declaring stuck |
| `stuck_distance` | 0.03 | 0.03 | m | Min progress required per timeout |
| `max_recovery_attempts` | 3 | 3 | — | Back-up + rotate attempts before abort |
| `recovery_backup_speed` | 0.05 | 0.05 | m/s | Reverse speed during recovery |
| `recovery_backup_time` | 1.5 | 1.5 | s | Duration of reverse |

**LiDAR obstacle avoidance:**

| Parameter | Sim | HW | Unit | Description |
|-----------|-----|-----|------|-------------|
| `obstacle_distance` | 0.25 | 0.30 | m | Distance threshold for obstacle detection |
| `obstacle_angle_deg` | 30.0 | 30.0 | deg | Half-angle of forward detection cone |

**Tilt detection (simulation only):**

| Parameter | Sim | HW | Unit | Description |
|-----------|-----|-----|------|-------------|
| `tilt_threshold_deg` | 40.0 | 40.0 | deg | Combined roll+pitch = tipped |
| `use_gazebo_reset` | true | false | — | Call Gazebo API to restore upright pose |

> ⚠️ **Why sim `max_angular_speed` is 1.0 (not 2.84):**  
> Gazebo's default physics timestep cannot handle sustained commands at 2.84 rad/s. The chassis tips, a wheel lifts, and odometry corrupts. **Do not raise above ~1.2 rad/s in sim.**

### SLAM Toolbox Key Settings (`params_slam.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `mapping` | `mapping` (new map each run) or `localization` |
| `resolution` | `0.05` | Map cell size in meters (5 cm) |
| `max_laser_range` | `3.5` | LiDAR range used for mapping (meters) |
| `map_update_interval` | `5.0` | Seconds between full map rasterisation |
| `transform_publish_period` | `0.02` | `map→odom` TF publish period (50 Hz) |
| `minimum_travel_distance` | `0.05` | Min distance before adding a pose node |
| `minimum_travel_heading` | `0.1` | Min rotation before adding a pose node |

### Tuning Tips

- **Robot flips / wheels lift off ground in sim:** This is caused by `max_angular_speed` being too high. Keep it at **1.0 rad/s** or below in Gazebo. Never set it above 1.2 rad/s in simulation.
- **Robot overshoots position:** Decrease `kp_linear` or `max_linear_speed`
- **Robot oscillates at heading:** Decrease `kp_angular` or `max_angular_speed`
- **Robot stops short of goal:** Decrease `position_tolerance`
- **Robot is too slow:** Increase `kp_linear` (up to a point — too high causes overshoot)
- **Hardware is jerky:** Lower `max_angular_speed` to 1.5 and `kp_angular` to 2.0
- **Robot arcs instead of driving straight:** The translate phase already reduces speed proportional to `cos(heading_error)`, but further reducing `kp_linear` can help
- **SLAM map is coarse:** Decrease `resolution` (e.g. `0.025` for 2.5 cm resolution, uses more RAM)
- **SLAM CPU is too high:** Increase `minimum_travel_distance` / `minimum_travel_heading`

### Runtime Parameter Override

```bash
ros2 launch turtlebot3_absolute_move hardware.launch.py
# In another terminal:
ros2 param set /absolute_move_node kp_linear 2.0
```

> **Note:** Runtime param changes only affect new goals (existing execution uses the values read at startup). Restart the node for parameter-reload during an active goal.

---

## Coordinate Frames & Odometry

### Frame Setup

#### Without SLAM

```
odom  ──►  base_footprint  ──►  base_link
 │
 │  Origin = robot boot position (drifts over time)
```

#### With SLAM (default)

```
map ──[SLAM correction]──► odom ──►  base_footprint  ──►  base_link
 │                          │
 │  Origin = world origin   │  SLAM keeps this accurate
```

- **`map` frame**: Stable world frame. SLAM publishes `map → odom` to keep it consistent.
- **`odom` frame**: Reference frame for all absolute move goals. With SLAM running, it stays calibrated.
- **`base_link`**: The robot body frame.

### Odometry Reset

> ⚠️ **Odom is initialized to (0, 0, 0) when the robot starts (or Gazebo resets).**
>
> All absolute move goals are relative to this initial pose. If you restart bringup or reset the simulation, the odom origin resets too. SLAM will rebuild the map from scratch.

**Practical implications:**
- Always start by noting the odom origin = physical starting position
- Goals like `(1.0, 0.0, 0.0)` mean "1 meter in front of where the robot started"
- With SLAM enabled, the map frame provides a stable global reference across multiple runs (if you save/load the map)

---

## Validation Checklist

Use this checklist to verify the node before deploying on hardware:

### In Simulation

1. **Build succeeds:**
   ```bash
   colcon build --symlink-install && source install/setup.bash
   ```

2. **Topics are correct:**
   ```bash
   ros2 topic list | grep -E "odom|cmd_vel|scan|map"
   # Should show /odom, /cmd_vel, /scan, /map (with SLAM)
   ```

3. **SLAM TF is publishing:**
   ```bash
   ros2 run tf2_ros tf2_echo map odom
   # Should show a transform (with slam:=true)
   ```

4. **Action server is up:**
   ```bash
   ros2 action list
   # Should show /absolute_move_node/absolute_move
   ```

5. **Simple forward move:**
   ```bash
   ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 0.5 0.0 0
   ```
   The robot should move ~0.5 m forward and stop.

6. **Diagonal move with heading change:**
   ```bash
   ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 1.0 1.0 90
   ```
   Robot should move to (1, 1) and face left (90°).

7. **Return to origin:**
   ```bash
   ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 0.0 0.0 0
   ```

8. **Emergency stop:**
   Press `Ctrl+C` during a move — robot should stop immediately and `cmd_vel` should go to zero.

### On Hardware

Run the same tests with smaller distances (0.3 m) first, in an open area.

---

## Troubleshooting

### "Action server not available"

- Verify `absolute_move_node` is running: `ros2 node list`
- Check the action topic: `ros2 action list`
- Ensure both client and node are using the same `ROS_DOMAIN_ID`

### Robot doesn't move

- Check odom is publishing: `ros2 topic hz /odom`
- Check cmd_vel is being received: `ros2 topic echo /cmd_vel`
- In Gazebo: make sure the simulation is unpaused (press Play)
- On hardware: verify bringup is running on the SBC

### SLAM is not starting

- Check `slam_toolbox` is installed: `ros2 pkg list | grep slam`
- If missing: `sudo apt install ros-humble-slam-toolbox`
- Check `/scan` is publishing: `ros2 topic hz /scan`
- Check the SLAM node is alive: `ros2 node list | grep slam`
- Disable SLAM for testing: `ros2 launch ... slam:=false`

### SLAM map looks wrong / empty

- Verify `/scan` is valid: `ros2 topic echo /scan --once`
- In Gazebo: make sure the robot model includes the LiDAR plugin
- Drive the robot around to let SLAM build the map from LiDAR data

### `map → odom` TF not publishing

- SLAM Toolbox must be running: `ros2 node list | grep slam`
- Check TF tree: `ros2 run tf2_ros tf2_echo map odom`
- If not present, SLAM may not have enough scan data yet — move the robot slightly

### Robot moves but never reaches the goal

- The robot may be stuck behind an obstacle. Check the log for `Phase 2 stuck` messages
- If stuck with obstacles, the robot will try up to `max_recovery_attempts` times (back up + rotate) before reporting best-effort position
- Odom drift may exceed tolerance for long distances
- Try increasing `position_tolerance` temporarily
- With SLAM disabled (`slam:=false`) drift accumulates — re-enable SLAM for long runs

### Robot reports "Partial: closest approach"

- The goal position is blocked by obstacles. The robot reached the closest achievable position
- Check if a different approach angle works: try sending the robot to a nearby unblocked point first
- The `position_error` in the result tells you how far the robot is from the requested goal

### "No odometry received" error

- The node waits 5 seconds for the first `/odom` message
- Sim: ensure Gazebo is running and unpaused
- HW: ensure `turtlebot3_bringup` is running on the SBC

### Robot oscillates near the goal

- Decrease `kp_angular` (try 2.0)
- Increase `heading_tolerance` (try 0.05)
- Decrease `max_angular_speed`

### Simulation time issues

- Sim mode must have `use_sim_time: true` in params and launch
- Verify: `ros2 param get /absolute_move_node use_sim_time`
- If clock is not publishing: `ros2 topic hz /clock`

### Namespace / topic mismatch

The node uses relative topic names (`cmd_vel`, `odom`) so they work with or without namespaces. The action server is at `~/absolute_move` (node-relative), which expands to `/absolute_move_node/absolute_move` by default.

---

## Known Limitations

1. **Reactive, not global**: The obstacle avoidance uses gap-finding on the live LiDAR scan (reactive). It does not plan a global path on the SLAM map. For complex mazes or narrow corridors, Nav2 with a global planner is more reliable.

2. **Single goal at a time**: The action server processes one goal at a time. Sending a new goal while one is executing will preempt the current goal.

3. **P-only control**: The controller uses proportional-only control. For most TurtleBot3 use cases this is sufficient, but aggressive gains may cause overshoot. PID could be added if needed.

4. **Best-effort is not guaranteed optimal**: When a goal is unreachable, the robot stops at the closest position it reached during its attempts. This may not be the globally closest reachable point.

5. **Odom frame assumption**: All goals are in the `odom` frame. With SLAM enabled (default), the `odom` frame is kept calibrated against the `map` frame. Without SLAM, goals are relative to the boot position only.

6. **SLAM requires LiDAR**: SLAM Toolbox needs `/scan` data (TurtleBot3 LiDAR). If the LiDAR is not working or `/scan` is not publishing, SLAM will not build a map and the `map → odom` TF will not be published. Disable SLAM with `slam:=false` to fall back to raw odometry.

7. **Tilt recovery is sim-only**: The Gazebo `set_entity_state` API is used to restore the robot when tipped. On real hardware, tilt detection will abort the goal but cannot physically right the robot.

---

## Autonomous Pipeline

The **Autonomous Navigation Pipeline** automates the entire workflow — from SLAM-based mapping to path-planned navigation — with a single command. It works identically in Gazebo simulation and on real hardware.

### Pipeline Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    SINGLE COMMAND EXECUTION                         │
│                                                                     │
│  Phase A: EXPLORE & MAP (autonomous frontier exploration)           │
│    └─ Robot drives itself to build a complete SLAM map              │
│    └─ Uses frontier-based exploration (BFS cluster detection)       │
│    └─ Stops when no more frontiers or timeout reached               │
│    └─ Saves the occupancy grid to disk                              │
│                                                                     │
│  Phase B: PLAN PATH (A* on the occupancy grid)                      │
│    └─ Takes goals from YAML file or interactive CLI                 │
│    └─ Runs A* with obstacle inflation on the SLAM map               │
│    └─ Produces waypoint sequence with path simplification           │
│                                                                     │
│  Phase C: EXECUTE (absolute move with obstacle avoidance)           │
│    └─ Sends waypoints to the existing absolute_move action server   │
│    └─ Replans if execution fails (up to max_replan_attempts)        │
│    └─ Gap-finding + stuck recovery handle local obstacles           │
│                                                                     │
│  Repeat Phase B+C for each goal in the list                         │
└─────────────────────────────────────────────────────────────────────┘
```

### Pipeline Quick Start

#### Simulation — Explore & Navigate

```bash
export TURTLEBOT3_MODEL=burger
source install/setup.bash

# Terminal 1 — Launch the full pipeline (explore, then enter goals interactively):
ros2 launch turtlebot3_absolute_move pipeline.launch.py

# Terminal 2 — (Optional) RViz with pipeline displays:
ros2 launch turtlebot3_absolute_move rviz.launch.py use_sim_time:=true
# Or use the pipeline-specific RViz config:
rviz2 -d install/turtlebot3_absolute_move/share/turtlebot3_absolute_move/config/pipeline.rviz
```

#### Simulation — With Predetermined Goals

```bash
# Navigate to goals defined in a YAML file after exploring:
ros2 launch turtlebot3_absolute_move pipeline.launch.py \
  goals_file:=$(ros2 pkg prefix turtlebot3_absolute_move)/share/turtlebot3_absolute_move/config/goals_example.yaml

# Or with the CLI directly:
ros2 run turtlebot3_absolute_move pipeline -- --goals /path/to/goals.yaml
```

#### Simulation — With Obstacles

```bash
# TurtleBot3 World (hexagonal obstacles):
ros2 launch turtlebot3_absolute_move pipeline.launch.py \
  world:=turtlebot3_world x_pose:=-2.0 y_pose:=-0.5

# House environment:
ros2 launch turtlebot3_absolute_move pipeline.launch.py \
  world:=turtlebot3_house x_pose:=-2.0 y_pose:=-0.5
```

#### Skip Exploration — Use Saved Map

```bash
# If you already have a map from a previous run:
ros2 run turtlebot3_absolute_move pipeline -- --map ~/pipeline_map.yaml --goals goals.yaml
```

#### Explore Only — Just Build the Map

```bash
ros2 run turtlebot3_absolute_move pipeline -- --explore-only
# Map is saved to ~/pipeline_map.pgm + ~/pipeline_map.yaml

# Custom save path:
ros2 run turtlebot3_absolute_move pipeline -- --explore-only --save-map ~/my_map
```

#### Hardware Mode

```bash
# On the SBC (first):
ssh ubuntu@<TURTLEBOT3_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py

# On the Remote PC:
export TURTLEBOT3_MODEL=burger
source install/setup.bash
ros2 launch turtlebot3_absolute_move pipeline.launch.py mode:=hardware
```

### Pipeline Architecture

```
┌───────────────┐      ┌────────────────────┐      ┌───────────────────────┐
│  Pipeline     │      │  Frontier Explorer  │      │  Path Planner         │
│  Orchestrator │─────►│  (in-process)       │      │  (A* on OccGrid)      │
│               │      │                     │      │  (in-process)         │
│  • CLI entry  │      │  • /map subscriber  │      │                       │
│  • Goal mgmt  │      │  • BFS clustering   │      │  • Obstacle inflation │
│  • Replan     │      │  • Scoring          │      │  • Line-of-sight      │
└───────┬───────┘      └────────────────────┘      └───────────────────────┘
        │  AbsoluteMove action
        ▼
┌───────────────────────────────────────────────────────────────────────┐
│                     absolute_move_node (EXISTING — UNCHANGED)         │
│  • 3-phase controller • gap-finding • stuck recovery • tilt detect   │
└───────────────────────────────────────────────────────────────────────┘
        │                                        ▲
        ▼                                        │
┌─────────────────────┐                ┌──────────────────────┐
│  TurtleBot3 Base    │──── /scan ────►│  SLAM Toolbox        │
│  (Gazebo / Real HW) │──── /odom ────►│  (builds /map, TF)   │
└─────────────────────┘                └──────────────────────┘
```

### Pipeline Algorithms

#### Frontier Exploration

The exploration phase uses a **frontier-based** algorithm:

1. A **frontier cell** is a free cell (value = 0) adjacent to an unknown cell (value = -1)
2. Frontier cells are clustered using **BFS flood-fill** on 8-connected neighbours
3. Clusters smaller than `min_frontier_size` are discarded
4. Each cluster is scored: `size × 1/(1 + distance × cost_weight)`
5. The highest-scoring frontier centroid becomes the next exploration goal
6. A* plans a path to it, absolute_move executes the waypoints
7. Repeats until no frontiers remain or `exploration_timeout` is reached

#### A* Path Planning

1. The SLAM occupancy grid is **inflated** by `inflation_radius` (robot radius + margin)
2. A* runs with 8-connected neighbours (diagonal cost = √2, cardinal = 1)
3. Heuristic: Euclidean distance (admissible, consistent)
4. The raw path is **simplified** using line-of-sight pruning (Bresenham)
5. Waypoints are spaced at least `waypoint_spacing` metres apart
6. If the goal is inside an inflated obstacle, the nearest free cell is used

> For a full algorithm deep-dive, see [pipeline_algorithm.md](pipeline_algorithm.md).

### Pipeline Goals File Format

```yaml
# goals.yaml — each entry is [x_metres, y_metres, heading_degrees]
goals:
  - [1.0, 0.0, 0]        # 1 m forward, face 0°
  - [1.0, 1.0, 90]       # diagonal to (1,1), face 90°
  - [0.0, 0.0, 0]        # return to origin
```

### Pipeline Parameter Reference

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `min_frontier_size` | 5 | cells | Minimum frontier cluster size |
| `exploration_timeout` | 300.0 | s | Max exploration time (5 min) |
| `frontier_cost_weight` | 0.5 | — | Higher = prefer closer frontiers |
| `no_frontier_patience` | 3 | — | Empty scans before stopping exploration |
| `inflation_radius` | 0.18 | m | Obstacle inflation (robot radius + margin) |
| `path_simplification` | true | — | Remove collinear intermediate waypoints |
| `waypoint_spacing` | 0.3 | m | Min distance between waypoints |
| `unknown_as_free` | false | — | Treat unknown cells as free space |
| `max_replan_attempts` | 3 | — | Replanning attempts per goal |
| `waypoint_reached_tolerance` | 0.10 | m | Tolerance for intermediate waypoints |

### Pipeline RViz Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | `nav_msgs/Path` | A* planned path (green line) |
| `/frontiers` | `visualization_msgs/MarkerArray` | Frontier cluster centroids (cyan spheres) |
| `/exploration_goal` | `visualization_msgs/Marker` | Current exploration target (orange arrow) |
| `/map` | `OccupancyGrid` | SLAM occupancy map |

### Pipeline Troubleshooting

#### Exploration doesn't start

- SLAM must be running: `ros2 node list | grep slam`
- `/map` must be publishing: `ros2 topic hz /map`
- `/scan` must have data: `ros2 topic echo /scan --once`

#### Exploration never stops

- Increase `no_frontier_patience` or decrease `exploration_timeout`
- Small gaps between walls create persistent frontiers — increase `min_frontier_size`

#### A* finds no path

- The goal may be inside an inflated obstacle — try a nearby position
- The map may not extend to the goal yet — ensure exploration covers the area
- Decrease `inflation_radius` if the corridors are very narrow

#### Pipeline hangs waiting for action server

- `absolute_move_node` must be running: `ros2 node list | grep absolute_move`
- Check the action: `ros2 action list`

---

## License

Apache-2.0
