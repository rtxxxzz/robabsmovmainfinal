# TurtleBot3 Absolute Move — Algorithm Deep-Dive

> **Source files analysed**  
> `absolute_move_node.py` · `absolute_move_client.py` · `AbsoluteMove.action` · `params_sim.yaml` · `params_hw.yaml`

---

## 1. What Does "Absolute Move" Mean?

A **relative move** means _"go 1 m forward from wherever I am right now."_  
An **absolute move** means _"go to world-coordinate (x=1.0, y=0.5, θ=90°)."_

The goal pose is expressed in the **`odom` frame** — the coordinate system whose origin is wherever the robot was when wheel odometry was initialised (boot or Gazebo reset). All goals are therefore repeatable within a single session, regardless of where the robot happens to be when the command is issued.

---

## 2. System Architecture

### Default — With SLAM Toolbox (slam:=true)

```
┌───────────────────┐   AbsoluteMove action   ┌──────────────────────────────────┐
│  absolute_move_   │ ──────────────────────► │        absolute_move_node        │
│  client (CLI or   │                          │                                  │
│  custom client)   │ ◄────────────────────── │  • /odom  subscriber             │
└───────────────────┘  feedback + result       │  • /cmd_vel publisher            │
                                               │  • 3-phase P-controller          │
                                               │  • MultiThreadedExecutor (4 thr) │
                                               └────────────────┬─────────────────┘
                                                                │ /cmd_vel  (Twist)
                                                                ▼
                                                   ┌───────────────────────┐
                                                   │  TurtleBot3 Burger    │
                                                   │  (Gazebo / real HW)   │
                                                   └─────────┬─────────────┘
                                                             │
                                               ┌─────────────┴──────────────────┐
                                               │             │                  │
                                          /odom (Odometry)  │  /scan (LiDAR)   │
                                               │             │                  │
                                               ▼             ▼                  │
                                    ┌──────────────────────────────────┐        │
                                    │   SLAM Toolbox (online async)    │        │
                                    │   • Builds /map in real time     │        │
                                    │   • Publishes map → odom TF      │        │
                                    │     (keeps odom frame accurate)  │        │
                                    └──────────────────────────────────┘        │
```

**TF tree with SLAM running:**
```
map ──[SLAM correction, 50 Hz]──► odom ──[wheel odom]──► base_footprint ──► base_link
                                              ↑
                              absolute_move_node reads /odom here
                              (SLAM keeps this frame drift-corrected)
```

The `absolute_move_node` requires **no code changes** to benefit from SLAM — it continues to subscribe to `/odom` and publish to `/cmd_vel`. SLAM Toolbox works transparently by publishing a `map → odom` TF transform that keeps the odom frame calibrated.

### ROS 2 Concurrency Model

The node uses a `ReentrantCallbackGroup` + `MultiThreadedExecutor(num_threads=4)`.  
This means the **odom callback** and the **execute callback** run on separate threads, safely accessing the shared pose variables through a `threading.Lock`.

```python
# Thread-safe pose snapshot used inside the control loop
def _get_pose(self):
    with self._lock:
        return self._x, self._y, self._yaw, self._odom_received
```

---

## 3. The Core Algorithm — Three-Phase Proportional Controller

The entire motion is broken into three sequential phases. Each phase runs its own closed-loop control at **30 Hz** (configurable via `control_rate`).

```
GOAL RECEIVED
     │
     ▼
┌─────────────┐       dist < pos_tol?  YES
│ Wait for    │──────────────────────────────────────────────────┐
│   odom      │                                                  │
└──────┬──────┘                                                  │
       │                                                         │
       ▼                                                         │
┌──────────────────────────────────┐                            │
│  Phase 1: Rotate to face target  │◄───── skip if at target ───┘
│  Pure rotation, zero linear vel  │
└──────────────┬───────────────────┘
               │ |heading_err| ≤ heading_tol
               ▼
┌──────────────────────────────────────────┐
│  Phase 2: Translate to target position   │
│  Combined linear + angular velocity      │
└──────────────┬───────────────────────────┘
               │ dist ≤ pos_tol
               ▼
┌──────────────────────────────────────────┐
│  Phase 3: Final heading alignment        │
│  Pure rotation to desired final heading  │
└──────────────┬───────────────────────────┘
               │ |heading_err| ≤ heading_tol
               ▼
           STOP & SUCCEED
```

---

## 4. Mathematical Foundations

### 4.1 Quaternion → Yaw Extraction

Odometry orientation arrives as a quaternion `(w, x, y, z)`.  
The yaw (Z-axis rotation) is extracted with:

```
siny_cosp = 2 * (w·z + x·y)
cosy_cosp = 1 - 2 * (y² + z²)
yaw = atan2(siny_cosp, cosy_cosp)
```

This is the standard rotation-matrix approach and is numerically stable for all quaternion values.  
Source: [absolute_move_node.py L38–42]

### 4.2 Angle Normalisation

All angle arithmetic must stay in `(-π, π]` to avoid "going the long way round":

```python
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle
```

Every heading error is passed through this function before being fed to the controller.

### 4.3 Proportional Control Law

For both rotation and translation, the system uses a simple **P-controller** (no I, no D):

```
output = Kp × error
output_clamped = clamp(output, -limit, +limit)
```

| Quantity        | Kp (sim / hw) | Limit (sim / hw) |
|-----------------|---------------|-----------------|
| Angular (rad/s) | 4.0 / 3.0     | 2.84 / 2.0 rad/s |
| Linear (m/s)    | 1.5 / 1.0     | 0.22 / 0.18 m/s |

The P-controller is self-dampening by nature: as the robot approaches the goal, the error shrinks, so the command velocity proportionally decreases, preventing full-speed impacts with the target pose.

---

## 5. Phase-by-Phase Algorithm Detail

### Phase 1 — Rotate to Face Target

**Goal:** Turn in place until the robot is facing the (tx, ty) position.

**Skip condition:** If `dist < pos_tol` (already at destination), this phase is skipped entirely to avoid a meaningless rotation.

**Control loop (30 Hz):**
```
angle_to_target = atan2(ty - cy, tx - cx)   # direction vector in odom frame
heading_err     = normalize(angle_to_target - yaw_current)
cmd.angular.z   = clamp(kp_angular × heading_err, max_angular_speed)
cmd.linear.x    = 0                          # zero forward velocity
```

**Termination:** `|heading_err| ≤ heading_tolerance` (default 0.02 rad ≈ 1.15°)

The `atan2` function correctly handles all four quadrants and the discontinuity at ±π is resolved by `normalize_angle`.

---

### Phase 2 — Translate to Target Position (with Reactive Obstacle Avoidance)

**Goal:** Drive to (tx, ty) while continuously correcting heading drift and avoiding obstacles.

**Control loop (30 Hz):**
```
dist            = hypot(tx - cx, ty - cy)
angle_to_target = atan2(ty - cy, tx - cx)
heading_err     = normalize(angle_to_target - yaw_current)

# --- Gap-based obstacle avoidance ---
gap_dir = find_best_gap(angle_to_target)    # returns None if path clear

if gap_dir is None:
    # Direct path clear — normal P-control
    linear_speed = clamp(kp_linear × dist, max_linear_speed)
    linear_speed ×= max(0.0, cos(heading_err))
    angular_speed = clamp(kp_angular × heading_err, max_angular_speed)
else:
    # Obstacle ahead — steer toward the best gap at half speed
    steer_err     = normalize(gap_dir - yaw_current)
    linear_speed  = max_linear_speed × 0.5 × max(0.0, cos(steer_err))
    angular_speed = clamp(kp_angular × steer_err, max_angular_speed)
```

**The Gap-Finding Algorithm (`_find_best_gap`):**

1. Check the forward ±30° cone for obstacles closer than `obstacle_distance`
2. If clear → return `None` (normal P-control)
3. If blocked → scan the full 360° LiDAR for consecutive "open" indices
4. Group them into gaps; filter out gaps narrower than the robot body (~20 cm)
5. Pick the gap whose centre direction is closest to the goal heading
6. Return that direction as the steering target

**The `cos(heading_err)` Heading Factor — why it matters:**

| heading_err | cos(heading_err) | Effect |
|-------------|------------------|--------|
| 0° (perfectly aligned) | 1.0 | Full speed forward |
| 45° | 0.707 | 70% speed |
| 90° | 0.0 | Stop! Pure rotation |
| > 90° (behind) | negative → clamped to 0.0 | Never go backward |

**Stuck Detection & Recovery:**

During Phase 2, the node monitors distance progress. If the robot fails to improve by `stuck_distance` metres within `stuck_timeout` seconds:

1. **Recovery maneuver:** backs up at `recovery_backup_speed` for `recovery_backup_time`, then rotates toward the best LiDAR gap (or 90° if no gap found)
2. Retries up to `max_recovery_attempts` times
3. If all attempts fail → returns `'partial'` (best-effort)

**Best-Effort Reaching:**

When Phase 2 cannot reach the goal (all recovery attempts exhausted), it does NOT abort. Instead:
- Phase 3 still runs (aligns heading at the closest position)
- The result reports `success=False` with a descriptive message and the actual `position_error`
- The caller can decide what to do next (send a different goal, accept partial, etc.)

**Tilt Detection:**

Every tick, the odom quaternion is checked for roll+pitch magnitude. If it exceeds `tilt_threshold_deg` (robot tipping):
- In sim: calls `/gazebo/set_entity_state` to restore the robot upright at the same (x, y, yaw)
- On hw: reports failure (cannot physically right the robot)

**Termination:** `dist ≤ position_tolerance` (default 0.05 m = 5 cm sim, 0.03 m hw)

---

### Phase 3 — Final Heading Alignment

**Goal:** Rotate in place to the desired final heading `target_heading`.

This is identical in structure to Phase 1, but the error is now measured against the _final_ heading rather than the direction toward the target.

```
heading_err   = normalize(target_heading - yaw_current)
cmd.angular.z = clamp(kp_angular × heading_err, max_angular_speed)
cmd.linear.x  = 0
```

**Termination:** `|heading_err| ≤ heading_tolerance`

---

## 6. Feedback, Cancellation, and Termination

### Live Feedback (published at every control tick)

```
feedback.current_x          # odom x
feedback.current_y          # odom y
feedback.current_heading     # yaw
feedback.distance_remaining  # Euclidean dist to target
feedback.heading_error       # current angular error (rad)
feedback.phase               # "rotate_to_target" | "translate" | "final_heading"
```

### Preemption / Cancellation

At the start of every control loop iteration across all three phases:

```python
if goal_handle.is_cancel_requested:
    self._stop()    # publishes Twist() with zero velocity
    return False    # unwinds the phase stack
```

The node also installs `SIGINT`/`SIGTERM` handlers that call `_stop()` before shutdown, ensuring the robot doesn't keep moving if the node is killed.

### Result

On full success:
```
result.success        = True
result.message        = 'Goal reached.'
result.final_x/y/heading = actual achieved pose from odom
result.position_error  = hypot(target - achieved)  [m]
result.heading_error   = |normalize(target_heading - achieved_yaw)|  [rad]
```

On partial (best-effort) reach:
```
result.success        = False
result.message        = 'Partial: closest approach X.XXX m from goal ...'
result.final_x/y/heading = closest position achieved
result.position_error  = remaining distance to goal  [m]
result.heading_error   = remaining heading error  [rad]
```

The robot always attempts Phase 3 (heading alignment) even on partial reach, so `heading_error` is minimised regardless of whether the position was fully reached.

---

## 7. SLAM Integration — Default Behaviour

### SLAM is ON by Default

Since **SLAM Toolbox** is enabled by default (`slam:=true` in `simulation.launch.py`), this system now operates in **SLAM-corrected odometry** mode rather than pure dead reckoning.

The `absolute_move_node` itself is **unchanged** — it still reads `/odom` and publishes to `/cmd_vel`. SLAM Toolbox works transparently:

```
Without SLAM (slam:=false):
  Encoder ticks → wheel velocities → integrate over dt → /odom  [drifts]

With SLAM (slam:=true, default):
  Encoder ticks → /odom   ←── SLAM Toolbox corrects this via map→odom TF
  /scan (LiDAR) ──────────→   (SLAM matches scans to map, corrects drift)
```

### How the TF Correction Works

SLAM Toolbox runs **online_async** mode and continuously:
1. Subscribes to `/scan` (TurtleBot3 LiDAR, 360°, 3.5 m range)
2. Matches each scan against the growing occupancy map
3. Computes the robot's true pose in the `map` frame
4. Publishes `map → odom` at 50 Hz to correct accumulated drift

The `absolute_move_node` reads `/odom` — with this TF correction in place, the odom frame stays aligned with the map frame across the entire session.

```
TF tree (SLAM running):
  map ──[SLAM correction, 50 Hz]──► odom ──[wheel odom]──► base_footprint
                                                ↑
                                absolute_move_node reads /odom
                                (SLAM keeps this drift-corrected)
```

### SLAM vs. Raw Odometry — Comparison

| | Raw Odometry (`slam:=false`) | SLAM-corrected (`slam:=true`, default) |
|---|---|---|
| **Source** | Wheel encoders only | Encoders + LiDAR scan matching |
| **Drift** | Accumulates over time/distance | Corrected continuously |
| **Map** | None | `/map` published (OccupancyGrid) |
| **Latency** | Near-zero | Near-zero (async scan matching) |
| **Dependencies** | None extra | `slam_toolbox` package |
| **Best for** | Short moves (< 2 m), no LiDAR | All move distances, loop closure |

### Disabling SLAM (Raw Odometry Mode)

```bash
# Run without SLAM — pure dead reckoning:
ros2 launch turtlebot3_absolute_move simulation.launch.py slam:=false
```

### SLAM Configuration

SLAM settings live in `config/params_slam.yaml`. Key parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `mode` | `mapping` | `mapping` = new map each run; `localization` = use saved map |
| `resolution` | `0.05` m | Map cell size (5 cm) |
| `max_laser_range` | `3.5` m | LiDAR range used for scan matching |
| `transform_publish_period` | `0.02` s | `map→odom` TF publish rate (50 Hz) |
| `map_update_interval` | `5.0` s | How often the full map is rasterised |

### Practical Accuracy (with SLAM enabled)

- **< 2 m moves:** Drift already minimal — SLAM adds loop-closure robustness.
- **2–5 m moves:** SLAM keeps error < 1–2 cm vs 1–3 cm without.
- **> 5 m moves:** SLAM continuously corrects drift — long-range accuracy maintained.
- **Spinning in place:** LiDAR scan matching corrects heading drift that gyros miss.

### For Nav2 Integration

With SLAM already running and the `map → odom` TF live, integrating **Nav2** is straightforward: Nav2's `controller_server` and `bt_navigator` can take over path planning while SLAM continues to provide localization. `absolute_move_node` is a lightweight alternative when you want direct control without the full Nav2 stack.

---

## 8. Control Loop Convergence Analysis

### Does Phase 1 Always Converge?

The angular P-controller is:
```
ω(t) = kp_ang × e(t)
```

As long as `kp_ang > 0` and no mechanical dead-zone exists, the system converges monotonically for pure rotation. The `normalize_angle` wrapper ensures the error never jumps sign unexpectedly.

The practical convergence rate (settling time to `heading_tol = 0.02 rad`) at `kp_ang = 4.0`:
- Starting 180° off → max speed (2.84 rad/s) for ~1 s, then P-ramp into tol
- Starting 30° off → reaches tolerance in ~0.3 s

### Does Phase 2 Always Converge?

Yes, given the heading factor. Without it, a robot with even slight lateral drift would overshoot and oscillate. The `cos(heading_err)` term ensures:
1. When misaligned, rotation dominates and forward speed goes to zero.
2. Once aligned, forward speed ramps up.
3. As `dist → 0`, `linear_speed → 0` (P-gain damping).

The combined effect creates an emergent "seek-and-approach" behaviour without explicit path planning.

### Oscillation Risk

Oscillation can occur when `kp_angular` is too high relative to `max_angular_speed` and `control_rate`. The robot overshoots the heading, corrects, overshoots again. Mitigations already in the code:
- Hard velocity clamp prevents instantaneous infinite correction.
- 30 Hz control rate is faster than typical robot dynamics.
- Tuning guide in `README.md`: reduce `kp_angular` or increase `heading_tolerance`.

---

## 9. Parameter Summary

**Core control:**

| Parameter | Sim | HW | Effect |
|-----------|-----|-----|--------|
| `max_linear_speed` | 0.12 m/s | 0.18 m/s | Caps Phase 2 forward velocity |
| `max_angular_speed` | **1.0 rad/s** | 2.0 rad/s | Caps all phases angular velocity |
| `position_tolerance` | 0.05 m | 0.03 m | Phase 2 exit threshold |
| `heading_tolerance` | 0.05 rad | 0.03 rad | Phases 1 & 3 exit threshold |
| `kp_linear` | 1.0 | 1.0 | Phase 2 linear P gain |
| `kp_angular` | **1.5** | 3.0 | All phases angular P gain |
| `control_rate` | 30.0 Hz | 30.0 Hz | Loop frequency |

**Stuck detection & recovery:**

| Parameter | Sim | HW | Effect |
|-----------|-----|-----|--------|
| `stuck_timeout` | 5.0 s | 8.0 s | Grace period before recovery |
| `stuck_distance` | 0.03 m | 0.03 m | Min progress per timeout |
| `max_recovery_attempts` | 3 | 3 | Back-up + rotate attempts |
| `recovery_backup_speed` | 0.05 m/s | 0.05 m/s | Reverse speed |
| `recovery_backup_time` | 1.5 s | 1.5 s | Reverse duration |

**LiDAR obstacle avoidance:**

| Parameter | Sim | HW | Effect |
|-----------|-----|-----|--------|
| `obstacle_distance` | 0.25 m | 0.30 m | Obstacle detection threshold |
| `obstacle_angle_deg` | 30.0° | 30.0° | Half-angle of forward cone |

**Tilt detection:**

| Parameter | Sim | HW | Effect |
|-----------|-----|-----|--------|
| `tilt_threshold_deg` | 40.0° | 40.0° | Roll+pitch = tipped |
| `use_gazebo_reset` | true | false | Gazebo API to restore upright |

> ⚠️ **Why sim `max_angular_speed` is 1.0 (not 2.84):**  
> Gazebo's physics destabilises at 2.84 rad/s. The chassis tips, a wheel lifts, and odometry corrupts. **Never raise above ~1.2 rad/s in Gazebo.**

HW gains are higher than sim because real-world factors require more authority:
- Floor surface variations (slip, friction)
- Motor dead-band (minimum voltage to move)
- Mechanical backlash in the wheel gear train


---

## 10. Known Algorithmic Limitations

| Limitation | Root Cause | Mitigation |
|------------|-----------|------------|
| **Reactive, not global planning** | Gap-finding uses live LiDAR only | Use Nav2 for complex mazes |
| **Odometry drift** | Dead reckoning accumulates error | Use SLAM (default) for long moves |
| **P-only control** | No I or D term | Add PID if overshoot is problematic |
| **Best-effort not optimal** | Closest approach ≠ globally closest reachable | Use Nav2 costmap for optimal routing |
| **Single active goal** | Sequential action server | Preemption is supported |
| **Tilt recovery sim-only** | Gazebo API not available on real hardware | Abort on tilt on real hardware |

---

## 11. Summary Diagram — Full Data Flow

```
┌──────────────────────────────────────────────────────────────────────┐
│                         absolute_move_node                           │
│                                                                      │
│  /odom ──► _odom_cb ──► [lock] _x, _y, _yaw, _odom_q (tilt check)   │
│  /scan ──► _scan_cb ──► [lock] _scan (LiDAR for gap-finding)         │
│                                      │                               │
│  Action Goal ──► _execute_cb         │                               │
│                       │              │                               │
│                  _wait_for_odom ◄────┘                               │
│                       │                                              │
│             ┌─────────▼──────────┐                                   │
│             │  Phase 1           │  ω = clamp(kp_ang × e_head, max)  │
│             │  Rotate to target  │  v = 0                            │
│             │  + tilt detection  │                                   │
│             └─────────┬──────────┘                                   │
│                       │ |e_head| ≤ tol                               │
│             ┌─────────▼──────────┐                                   │
│             │  Phase 2           │  gap = _find_best_gap(goal_dir)   │
│             │  Translate         │  if gap: steer toward gap @ 50%   │
│             │  + gap avoidance   │  else: normal P-control           │
│             │  + stuck recovery  │  stuck? → back up + rotate + retry│
│             │  + tilt detection  │  exhausted? → 'partial' result    │
│             └─────────┬──────────┘                                   │
│                       │ d ≤ pos_tol  OR  'partial'                   │
│             ┌─────────▼──────────┐                                   │
│             │  Phase 3           │  ω = clamp(kp_ang × e_fin, max)   │
│             │  Final heading     │  v = 0                            │
│             │  + tilt detection  │  (runs even on partial reach)     │
│             └─────────┬──────────┘                                   │
│                       │ |e_fin| ≤ tol                                │
│                  STOP → Publish Result (success or partial)           │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 12. ArUco-Based Absolute Move — What You Would Need to Add

### 12.1 Why ArUco? The Problem With Odom-Only Control

The current algorithm has one fundamental weakness: **wheel odometry drifts**. Every time a wheel slips, hits a carpet edge, or the robot spins on a slippery floor, the cumulative pose estimate gets a little bit worse. For moves > 2 m this matters.

**ArUco markers** are printed square patterns that a camera can detect and compute an exact 6-DOF pose from (x, y, z, roll, pitch, yaw). If you place ArUco markers at known positions in the environment, the robot can see them and calculate its position relative to them with millimetre-level accuracy — with zero drift, every frame.

Think of ArUco markers as **GPS waypoints for indoor robots**.

---

### 12.2 Conceptual Architecture Change

**Current flow (odom-only):**
```
Wheel encoders → /odom → absolute_move_node → /cmd_vel → Robot
```

**With ArUco correction:**
```
Camera + ArUco markers → /aruco_pose → aruco_localizer_node ──┐
                                                               │ corrected pose
Wheel encoders → /odom ────────────────────────────────────────┤
                                                               ▼
                                                  absolute_move_node
                                                         │
                                                    /cmd_vel → Robot
```

The `aruco_localizer_node` fuses the marker observation with the odom to produce a corrected, drift-free pose estimate that the controller uses instead of raw odom.

---

### 12.3 Hardware You Need

| Component | Example | Notes |
|-----------|---------|-------|
| **Camera** | Raspberry Pi Camera v2 / USB webcam | Must be calibrated (intrinsics + distortion) |
| **ArUco markers** | A4 printed, laminated | Use a fixed dictionary: `DICT_4X4_50` or `DICT_6X6_250` |
| **Marker placement** | Fixed positions in the environment | Measure their `(x, y, θ)` in the odom/map frame at setup time |
| **Camera mount** | Rigid mount on robot | Any flex introduces pose error |

---

### 12.4 Software Stack Required

#### New ROS 2 Packages

```bash
# OpenCV ArUco detection (usually bundled with OpenCV >= 4.7)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Optional: ros2-aruco (community package, wraps OpenCV detection)
# https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
sudo apt install ros-humble-ros2-aruco

# Camera driver (for Pi Camera or USB cam)
sudo apt install ros-humble-usb-cam        # USB webcam
sudo apt install ros-humble-v4l2-camera    # V4L2-compatible cameras
```

#### Camera Calibration (mandatory first step)

You must calibrate your camera before ArUco pose estimation will be accurate. Without calibration, lens distortion causes systematic pose errors.

```bash
# Use the ROS 2 camera_calibration package:
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw

# This produces a camera_info.yaml with:
# - camera_matrix (K): focal lengths and principal point
# - distortion_coefficients (D): radial + tangential distortion
```

---

### 12.5 The Pose Pipeline — Step by Step

#### Step 1: Camera Detects Marker → Gets Pose in Camera Frame

The ArUco detector (OpenCV) takes an image and outputs a **rotation vector (rvec)** and **translation vector (tvec)** for each visible marker. These describe where the marker is **relative to the camera**.

```python
import cv2
import cv2.aruco as aruco
import numpy as np

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector   = aruco.ArucoDetector(dictionary, parameters)

corners, ids, rejected = detector.detectMarkers(gray_image)

if ids is not None:
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
        corners, marker_size_meters, camera_matrix, dist_coeffs)
    # tvec[i] = [x, y, z] of marker in camera frame (meters)
    # rvec[i] = Rodrigues rotation vector of marker in camera frame
```

#### Step 2: Transform Marker Pose to Robot Frame (`base_link`)

The camera is mounted somewhere on the robot — not at the robot's centre. You need a static TF transform from `camera_optical_frame` to `base_link`:

```yaml
# static_tf.yaml — describes camera mount position on robot
# Example: camera is 15 cm forward, 10 cm up from base_link, facing forward
translation: [0.15, 0.0, 0.10]
rotation:    [0.0,  0.0, 0.0, 1.0]   # quaternion, no rotation
```

Publish this at startup:
```bash
ros2 run tf2_ros static_transform_publisher \
  0.15 0.0 0.10  0 0 0 1  base_link camera_optical_frame
```

With this TF, the marker pose in `camera_optical_frame` → `base_link` frame is handled by `tf2`:
```python
from tf2_ros import Buffer, TransformListener
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

pose_camera = PoseStamped()
pose_camera.header.frame_id = 'camera_optical_frame'
# ... fill in from rvec/tvec ...

pose_base = tf_buffer.transform(pose_camera, 'base_link')
```

#### Step 3: From Marker Pose in Robot Frame → Robot Pose in World Frame

This is the **inverse**: if you know where the marker is in the world (from your map/setup) and where the marker is relative to the robot, you can invert to get where the robot is in the world.

```
robot_pose_world = marker_pose_world  ×  inv(marker_pose_robot)
```

In matrix form:
```
T_robot_world = T_marker_world × inv(T_marker_robot)
```

where `T_marker_world` is the known world position of the marker (measured at setup time).

#### Step 4: Publish Corrected Pose → Use in Controller

The corrected robot pose is published as a `PoseWithCovarianceStamped` on `/aruco_pose` (or injected as a TF `map → base_link`).

---

### 12.6 Two Integration Strategies

#### Strategy A — Replace `/odom` with ArUco Pose (Simple, but Lossy)

When a marker is visible, use ArUco pose directly as the robot's ground truth. When no marker is visible, fall back to odom.

**Changes to `absolute_move_node.py`:**

1. Add a new subscriber:
   ```python
   self._aruco_sub = self.create_subscription(
       PoseWithCovarianceStamped, '/aruco_pose',
       self._aruco_cb, 10, callback_group=self._cb_group)
   self._aruco_received = False
   self._aruco_timeout = 0.5  # seconds since last marker seen
   ```

2. Add a new pose callback:
   ```python
   def _aruco_cb(self, msg):
       with self._lock:
           self._x   = msg.pose.pose.position.x
           self._y   = msg.pose.pose.position.y
           self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
           self._aruco_received = True
           self._last_aruco_time = self.get_clock().now()
   ```

3. Modify `_get_pose()` to prefer ArUco when fresh:
   ```python
   def _get_pose(self):
       with self._lock:
           aruco_fresh = (
               self._aruco_received and
               (self.get_clock().now() - self._last_aruco_time).nanoseconds < 5e8
           )
           source = 'aruco' if aruco_fresh else 'odom'
           return self._x, self._y, self._yaw, True, source
   ```

**Pros:** Simple to implement, very accurate when marker is visible.  
**Cons:** Pose jumps when switching between aruco/odom; no smoothing.

---

#### Strategy B — Fuse ArUco + Odom with EKF (Robust, Production-Grade)

Use the **`robot_localization`** package's Extended Kalman Filter (EKF). The EKF continuously fuses wheel odometry and ArUco pose corrections to produce a smooth, drift-corrected pose estimate.

```bash
sudo apt install ros-humble-robot-localization
```

**EKF configuration (`ekf.yaml`):**
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true        # planar motion only for TurtleBot3

    odom0: /odom
    odom0_config: [true, true, false,   # x, y, z
                   false, false, true,  # roll, pitch, yaw
                   false, false, false, # vx, vy, vz
                   false, false, false, # vroll, vpitch, vyaw
                   false, false, false] # ax, ay, az

    pose0: /aruco_pose
    pose0_config: [true, true, false,   # x, y, z (use x, y from aruco)
                   false, false, true,  # yaw
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_rejection_threshold: 0.8     # reject outlier detections

    publish_tf: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
```

**Launch:**
```bash
ros2 run robot_localization ekf_node --ros-args --params-file ekf.yaml
```

The EKF publishes a corrected `/odometry/filtered` topic. Change `absolute_move_node` to subscribe to `/odometry/filtered` instead of `/odom`.

**Pros:** Smooth pose, handles marker occlusion gracefully, industry-standard approach.  
**Cons:** Requires tuning process/measurement noise covariances (Q and R matrices).

---

### 12.7 Marker Map Setup

Before running, you need to know each marker's exact pose in the world (odom or map frame). The typical workflow:

```
1. Place markers at fixed physical locations in your environment.
2. Manually drive the robot to the origin and reset odom.
3. Drive to each marker, record its odom-frame pose.
4. Store these in a YAML map file:
```

```yaml
# aruco_map.yaml
markers:
  - id: 0
    x: 1.500
    y: 0.000
    theta: 0.000
    size: 0.10    # meters (physical marker side length)
  - id: 1
    x: 0.000
    y: 2.000
    theta: 1.5708
    size: 0.10
  - id: 5
    x: 3.200
    y: 1.100
    theta: 3.1416
    size: 0.15
```

The `aruco_localizer_node` loads this map at startup and uses it to convert camera detections into world-frame robot poses.

---

### 12.8 New Node: `aruco_localizer_node.py` (Outline)

This is the new node you would write (or use from `ros2_aruco`):

```python
class ArucoLocalizerNode(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')
        # Load camera calibration
        self._K, self._D = self._load_calibration('camera_info.yaml')
        # Load marker map
        self._marker_map = self._load_marker_map('aruco_map.yaml')
        # ArUco detector
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._detector   = cv2.aruco.ArucoDetector(self._dictionary, ...)
        # Camera subscriber
        self._image_sub  = self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        # Corrected pose publisher
        self._pose_pub   = self.create_publisher(PoseWithCovarianceStamped, '/aruco_pose', 10)

    def _image_cb(self, msg):
        frame  = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in self._marker_map:
                continue
            # Estimate pose of marker in camera frame
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], self._marker_map[marker_id]['size'],
                self._K, self._D)
            # Convert to world-frame robot pose
            robot_pose = self._compute_robot_world_pose(marker_id, rvec[0], tvec[0])
            # Publish
            self._pose_pub.publish(robot_pose)
```

---

### 12.9 What Exactly Changes in `absolute_move_node.py`

| Change | Detail |
|--------|--------|
| **New subscriber** | `/aruco_pose` → `PoseWithCovarianceStamped` |
| **New state variables** | `_aruco_received`, `_last_aruco_time` |
| **`_get_pose()` logic** | Prefer ArUco if fresh (< 0.5 s old), else fall back to odom |
| **New parameter** | `aruco_timeout: 0.5` — seconds before treating marker as gone |
| **New parameter** | `use_aruco: true` — feature flag to enable/disable |
| **Phase feedback** | Add `pose_source: "aruco"/"odom"` to feedback message |
| **(Optional) Action definition** | Add `pose_source` field to `AbsoluteMove.action` Feedback |

---

### 12.10 New Parameters (`params_aruco.yaml`)

```yaml
absolute_move_node:
  ros__parameters:
    # --- Existing params (same) ---
    max_linear_speed: 0.22
    max_angular_speed: 2.84
    kp_linear: 1.5
    kp_angular: 4.0
    control_rate: 30.0

    # --- Tighter tolerances (ArUco is more accurate than odom) ---
    position_tolerance: 0.01    # 1 cm (was 2 cm)
    heading_tolerance:  0.01    # ~0.57° (was 1.15°)

    # --- ArUco-specific ---
    use_aruco: true
    aruco_topic: '/aruco_pose'
    aruco_timeout: 0.5          # seconds before fallback to odom
    aruco_min_confidence: 0.5   # reject high-covariance detections
```

---

### 12.11 Accuracy Comparison

| Method | Typical Position Error | Typical Heading Error | Drift |
|--------|----------------------|----------------------|-------|
| Odom only | 1–5 cm (< 2 m) | 0.5–2° | Accumulates |
| ArUco (Strategy A) | 3–8 mm | 0.2–0.5° | None (when visible) |
| ArUco + EKF (Strategy B) | 5–15 mm | 0.3–1° | None, smooth |
| Nav2 + AMCL + LiDAR | 2–5 cm | 0.5–1° | None, global |

ArUco gives **sub-centimetre accuracy** when the marker fills at least ~15% of the camera frame. Accuracy degrades at large distances and steep angles (> 45° from marker normal).

---

### 12.12 Known ArUco Limitations

| Limitation | Impact | Mitigation |
|------------|--------|------------|
| **Line-of-sight required** | No pose update when marker is occluded | Multiple markers, strategic placement |
| **Lighting sensitivity** | Detection fails in poor/harsh light | Use IR markers + IR camera indoors |
| **Distance limit** | Pose error grows past ~2 m for 10 cm markers | Use larger markers or fisheye lens |
| **Angle sensitivity** | > 60° from marker normal → unreliable | Place markers facing robot path |
| **Single-frame noise** | Individual detections can be noisy | EKF smoothing or median filter |
| **Camera calibration drift** | Thermal changes can shift intrinsics | Re-calibrate periodically |

---

### 12.13 Full Data Flow with ArUco

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    ArUco-Enhanced Absolute Move System                   │
│                                                                          │
│  /camera/image_raw ──► aruco_localizer_node                              │
│       (Pi Camera)            │                                           │
│                              │  ArUco marker detected                    │
│                              │  → compute robot world pose               │
│                              ▼                                           │
│                       /aruco_pose ──────────────────────┐                │
│                    (PoseWithCovarianceStamped)           │                │
│                                                         ▼                │
│  /odom ──────────────────────────────────► absolute_move_node            │
│  (wheel encoders, fallback)               │                              │
│                                           │  _get_pose():                │
│                                           │   aruco fresh? → use aruco   │
│                                           │   else         → use odom    │
│                                           │                              │
│                                     3-phase P-controller                 │
│                                           │                              │
│                                      /cmd_vel ──► TurtleBot3             │
└──────────────────────────────────────────────────────────────────────────┘

      ┌─────────────────────── Optional: EKF Fusion ────────────────────┐
      │  /odom + /aruco_pose → robot_localization EKF                   │
      │  → /odometry/filtered (smooth, fused pose)                      │
      │  → absolute_move_node subscribes to /odometry/filtered instead  │
      └─────────────────────────────────────────────────────────────────┘
```
