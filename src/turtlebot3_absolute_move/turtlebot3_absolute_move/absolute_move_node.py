"""
Absolute Move Node for TurtleBot3.

Three-phase proportional controller to absolute (x, y, heading) pose.
SLAM Toolbox (slam:=true default) keeps the odom frame drift-corrected.

Reactive obstacle avoidance:
  When an obstacle blocks the direct path to the goal, the node analyses
  the full LiDAR scan to find the best open gap whose direction is closest
  to the goal heading.  The robot steers through the gap at reduced speed
  and resumes direct control once the path clears.  If stuck (no progress
  for stuck_timeout seconds), it backs up, rotates toward the best gap,
  and retries up to max_recovery_attempts times.

Best-effort reaching:
  If the goal is unreachable (all recovery attempts exhausted), the node
  does NOT simply abort.  Instead it:
    1. Records the closest position achieved during Phase 2
    2. Still performs Phase 3 (heading alignment) at the closest position
    3. Reports success=False with a clear message and the actual
       position/heading errors, so the caller can decide what to do next.

Tilt detection:
  Monitors the odom quaternion for chassis tipping.  In simulation, calls
  /gazebo/set_entity_state to restore the robot upright and continues.
"""

import math
import signal
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from turtlebot3_absolute_move_interfaces.action import AbsoluteMove

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    _GAZEBO_AVAILABLE = True
except ImportError:
    _GAZEBO_AVAILABLE = False


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def tilt_from_quaternion(q) -> float:
    """Return combined roll+pitch magnitude in radians (0 = upright)."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    return math.sqrt(roll * roll + pitch * pitch)


class AbsoluteMoveNode(Node):

    def __init__(self):
        super().__init__('absolute_move_node')

        # --- Parameters ---
        self.declare_parameter('max_linear_speed',    0.12)
        self.declare_parameter('max_angular_speed',   1.0)
        self.declare_parameter('position_tolerance',  0.05)
        self.declare_parameter('heading_tolerance',   0.05)
        self.declare_parameter('kp_linear',           1.0)
        self.declare_parameter('kp_angular',          1.5)
        self.declare_parameter('control_rate',        30.0)
        self.declare_parameter('stuck_timeout',       5.0)
        self.declare_parameter('stuck_distance',      0.03)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('recovery_backup_speed', 0.05)
        self.declare_parameter('recovery_backup_time',  1.5)
        self.declare_parameter('obstacle_distance',   0.25)
        self.declare_parameter('obstacle_angle_deg',  30.0)
        self.declare_parameter('tilt_threshold_deg',  40.0)
        self.declare_parameter('use_gazebo_reset',    True)

        self._read_params()

        # --- Shared state ---
        self._lock = threading.Lock()
        self._odom_received = False
        self._x = self._y = self._yaw = 0.0
        self._odom_q = None
        self._scan = None

        # --- ROS interfaces ---
        self._cb_group = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10,
            callback_group=self._cb_group)

        self.create_subscription(
            LaserScan, 'scan', self._scan_cb, 10,
            callback_group=self._cb_group)

        self._reset_client = None
        if _GAZEBO_AVAILABLE and self._use_gazebo_reset:
            self._reset_client = self.create_client(
                SetEntityState, '/gazebo/set_entity_state')

        self._action_server = ActionServer(
            self, AbsoluteMove, '~/absolute_move',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)

        self.get_logger().info('Absolute Move node ready.')
        self.get_logger().info(
            f'pos_tol={self._pos_tol:.3f} m  '
            f'head_tol={math.degrees(self._head_tol):.1f} deg  '
            f'obstacle_dist={self._obstacle_dist:.2f} m')

    def _read_params(self):
        self._max_lin  = self.get_parameter('max_linear_speed').value
        self._max_ang  = self.get_parameter('max_angular_speed').value
        self._pos_tol  = self.get_parameter('position_tolerance').value
        self._head_tol = self.get_parameter('heading_tolerance').value
        self._kp_lin   = self.get_parameter('kp_linear').value
        self._kp_ang   = self.get_parameter('kp_angular').value
        self._rate_hz  = self.get_parameter('control_rate').value
        self._stuck_timeout  = self.get_parameter('stuck_timeout').value
        self._stuck_distance = self.get_parameter('stuck_distance').value
        self._max_recovery   = self.get_parameter('max_recovery_attempts').value
        self._backup_speed   = self.get_parameter('recovery_backup_speed').value
        self._backup_time    = self.get_parameter('recovery_backup_time').value
        self._obstacle_dist  = self.get_parameter('obstacle_distance').value
        self._obstacle_angle = math.radians(
            self.get_parameter('obstacle_angle_deg').value)
        self._tilt_threshold = math.radians(
            self.get_parameter('tilt_threshold_deg').value)
        self._use_gazebo_reset = self.get_parameter('use_gazebo_reset').value

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._x   = msg.pose.pose.position.x
            self._y   = msg.pose.pose.position.y
            self._yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self._odom_q = msg.pose.pose.orientation
            self._odom_received = True

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg

    def _get_pose(self):
        with self._lock:
            return self._x, self._y, self._yaw, self._odom_received

    def _get_tilt(self):
        with self._lock:
            if self._odom_q is None:
                return 0.0
            return tilt_from_quaternion(self._odom_q)

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f'Goal: x={goal_request.target_x:.3f} '
            f'y={goal_request.target_y:.3f} '
            f'h={math.degrees(goal_request.target_heading):.1f} deg')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('Cancel requested.')
        self._stop()
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Execute — with best-effort reaching
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        self.get_logger().info('Executing Absolute Move...')
        tx = goal_handle.request.target_x
        ty = goal_handle.request.target_y
        th = normalize_angle(goal_handle.request.target_heading)

        result   = AbsoluteMove.Result()
        feedback = AbsoluteMove.Feedback()
        rate     = self.create_rate(self._rate_hz)

        if not self._wait_for_odom(goal_handle, rate):
            return self._abort(goal_handle, 'No odometry received.')

        # --- Phase 1: rotate to face target ---
        ok = self._phase_rotate_to_target(goal_handle, tx, ty, feedback, rate)
        if ok is None:
            return self._abort(goal_handle, 'Stuck in Phase 1.')
        if not ok:
            return self._cancelled(goal_handle)

        # --- Phase 2: translate (with obstacle avoidance) ---
        phase2_result = self._phase_translate(
            goal_handle, tx, ty, th, feedback, rate)

        if phase2_result == 'cancelled':
            return self._cancelled(goal_handle)

        # phase2_result is True (reached) or 'partial' (closest approach)
        is_partial = (phase2_result == 'partial')

        if is_partial:
            cx, cy, cyaw, _ = self._get_pose()
            remaining = math.hypot(tx - cx, ty - cy)
            self.get_logger().warn(
                f'Target unreachable. Best approach: {remaining:.3f} m '
                f'from goal. Aligning heading at current position.')

        # --- Phase 3: final heading alignment ---
        ok = self._phase_final_heading(goal_handle, th, feedback, rate)
        if ok is None:
            # Even Phase 3 stuck — still report what we achieved
            pass
        if ok is False:
            return self._cancelled(goal_handle)

        # --- Build result ---
        self._stop()
        cx, cy, cyaw, _ = self._get_pose()
        result.final_x       = cx
        result.final_y       = cy
        result.final_heading = cyaw
        result.position_error = math.hypot(tx - cx, ty - cy)
        result.heading_error  = abs(normalize_angle(th - cyaw))

        if is_partial:
            result.success = False
            result.message = (
                f'Partial: closest approach {result.position_error:.3f} m '
                f'from goal (heading err {math.degrees(result.heading_error):.1f} deg). '
                f'Target may be unreachable.')
            self.get_logger().warn(
                f'Partial result: ({cx:.3f}, {cy:.3f}, '
                f'{math.degrees(cyaw):.1f} deg) — '
                f'{result.position_error:.3f} m from goal')
            goal_handle.abort()
        else:
            result.success = True
            result.message = 'Goal reached.'
            self.get_logger().info(
                f'Goal reached: ({cx:.3f}, {cy:.3f}, '
                f'{math.degrees(cyaw):.1f} deg) — '
                f'err={result.position_error:.3f} m')
            goal_handle.succeed()

        return result

    # ------------------------------------------------------------------
    # Phase 1: rotate to face target
    # ------------------------------------------------------------------

    def _wait_for_odom(self, goal_handle, rate, timeout_sec=5.0):
        elapsed, dt = 0.0, 1.0 / self._rate_hz
        while not self._odom_received and elapsed < timeout_sec:
            if goal_handle.is_cancel_requested:
                return False
            rate.sleep()
            elapsed += dt
        return self._odom_received

    def _phase_rotate_to_target(self, goal_handle, tx, ty, fb, rate):
        """Phase 1 — rotate to face target. Returns True/False/None."""
        cx, cy, cyaw, _ = self._get_pose()
        if math.hypot(tx - cx, ty - cy) < self._pos_tol:
            return True

        heading_err   = normalize_angle(math.atan2(ty - cy, tx - cx) - cyaw)
        best_abs_err  = abs(heading_err)
        last_progress = time.monotonic()

        self.get_logger().info(
            f'Phase 1: rotate to face target '
            f'(err={math.degrees(heading_err):.1f} deg)')

        while abs(heading_err) > self._head_tol:
            if goal_handle.is_cancel_requested:
                self._stop(); return False
            if self._get_tilt() > self._tilt_threshold:
                if not self._try_tilt_recovery():
                    return None

            cx, cy, cyaw, _ = self._get_pose()
            heading_err = normalize_angle(
                math.atan2(ty - cy, tx - cx) - cyaw)

            abs_err = abs(heading_err)
            if abs_err < best_abs_err - self._head_tol:
                best_abs_err  = abs_err
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > self._stuck_timeout:
                self._stop()
                self.get_logger().warn('Phase 1 stuck.')
                return None

            cmd = Twist()
            cmd.angular.z = self._clamp(
                self._kp_ang * heading_err, self._max_ang)
            self._cmd_pub.publish(cmd)

            fb.current_x = cx; fb.current_y = cy
            fb.current_heading = cyaw
            fb.distance_remaining = math.hypot(tx - cx, ty - cy)
            fb.heading_error = heading_err
            fb.phase = 'rotate_to_target'
            goal_handle.publish_feedback(fb)
            rate.sleep()

        self._stop()
        self.get_logger().info('Phase 1 complete.')
        return True

    # ------------------------------------------------------------------
    # Phase 2: translate with reactive gap-based obstacle avoidance
    # ------------------------------------------------------------------

    def _phase_translate(self, goal_handle, tx, ty, th, fb, rate):
        """Drive toward (tx, ty) with reactive obstacle avoidance.

        Returns:
            True        — reached the goal position
            'cancelled' — goal was cancelled
            'partial'   — could not reach; stopped at closest approach
        """
        cx, cy, cyaw, _ = self._get_pose()
        dist = math.hypot(tx - cx, ty - cy)

        self.get_logger().info(
            f'Phase 2: translate to target (dist={dist:.3f} m)')

        best_dist     = dist
        last_progress = time.monotonic()
        recovery_attempts = 0

        while dist > self._pos_tol:
            if goal_handle.is_cancel_requested:
                self._stop(); return 'cancelled'

            # ── Tilt recovery ─────────────────────────────────────
            if self._get_tilt() > self._tilt_threshold:
                if not self._try_tilt_recovery():
                    self._stop(); return 'partial'
                cx, cy, cyaw, _ = self._get_pose()
                dist = math.hypot(tx - cx, ty - cy)
                best_dist     = dist
                last_progress = time.monotonic()
                continue

            cx, cy, cyaw, _ = self._get_pose()
            dist = math.hypot(tx - cx, ty - cy)
            angle_to_target = math.atan2(ty - cy, tx - cx)
            heading_err     = normalize_angle(angle_to_target - cyaw)

            # ── Stuck detection + recovery ────────────────────────
            if dist < best_dist - self._stuck_distance:
                best_dist     = dist
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > self._stuck_timeout:
                if recovery_attempts >= self._max_recovery:
                    self._stop()
                    self.get_logger().error(
                        f'Phase 2: {self._max_recovery} recovery '
                        f'attempts exhausted (dist={dist:.3f} m).')
                    return 'partial'
                recovery_attempts += 1
                self.get_logger().warn(
                    f'Phase 2 stuck - recovery {recovery_attempts}/'
                    f'{self._max_recovery}')
                self._recovery_maneuver(angle_to_target, rate)
                cx, cy, cyaw, _ = self._get_pose()
                dist = math.hypot(tx - cx, ty - cy)
                best_dist     = dist
                last_progress = time.monotonic()
                continue

            # ── Reactive gap-based steering ───────────────────────
            gap_dir = self._find_best_gap(angle_to_target)

            if gap_dir is None:
                # Direct path is clear — normal P-control
                linear_speed = self._clamp(
                    self._kp_lin * dist, self._max_lin)
                linear_speed *= max(0.0, math.cos(heading_err))
                angular_speed = self._clamp(
                    self._kp_ang * heading_err, self._max_ang)
            else:
                # Obstacle ahead — steer toward the best gap
                steer_err = normalize_angle(gap_dir - cyaw)
                linear_speed = self._max_lin * 0.5
                linear_speed *= max(0.0, math.cos(steer_err))
                angular_speed = self._clamp(
                    self._kp_ang * steer_err, self._max_ang)

            cmd = Twist()
            cmd.linear.x  = linear_speed
            cmd.angular.z = angular_speed
            self._cmd_pub.publish(cmd)

            # Feedback — show heading-to-target error (what controller uses)
            fb.current_x = cx; fb.current_y = cy
            fb.current_heading = cyaw
            fb.distance_remaining = dist
            fb.heading_error = heading_err
            fb.phase = 'translate'
            goal_handle.publish_feedback(fb)
            rate.sleep()

        self._stop()
        self.get_logger().info('Phase 2 complete.')
        return True

    # ------------------------------------------------------------------
    # Phase 3: final heading
    # ------------------------------------------------------------------

    def _phase_final_heading(self, goal_handle, th, fb, rate):
        """Phase 3 — final heading. Returns True/False/None."""
        cx, cy, cyaw, _ = self._get_pose()
        heading_err   = normalize_angle(th - cyaw)
        best_abs_err  = abs(heading_err)
        last_progress = time.monotonic()

        self.get_logger().info(
            f'Phase 3: align heading '
            f'{math.degrees(th):.1f} deg '
            f'(err={math.degrees(heading_err):.1f} deg)')

        while abs(heading_err) > self._head_tol:
            if goal_handle.is_cancel_requested:
                self._stop(); return False
            if self._get_tilt() > self._tilt_threshold:
                if not self._try_tilt_recovery():
                    return None

            cx, cy, cyaw, _ = self._get_pose()
            heading_err = normalize_angle(th - cyaw)

            abs_err = abs(heading_err)
            if abs_err < best_abs_err - self._head_tol:
                best_abs_err  = abs_err
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > self._stuck_timeout:
                self._stop()
                self.get_logger().warn('Phase 3 stuck.')
                return None

            cmd = Twist()
            cmd.angular.z = self._clamp(
                self._kp_ang * heading_err, self._max_ang)
            self._cmd_pub.publish(cmd)

            fb.current_x = cx; fb.current_y = cy
            fb.current_heading = cyaw
            fb.distance_remaining = 0.0
            fb.heading_error = heading_err
            fb.phase = 'final_heading'
            goal_handle.publish_feedback(fb)
            rate.sleep()

        self._stop()
        self.get_logger().info('Phase 3 complete.')
        return True

    # ------------------------------------------------------------------
    # LiDAR gap-finding (reactive obstacle avoidance)
    # ------------------------------------------------------------------

    def _find_best_gap(self, goal_angle_world: float):
        """Analyse the LiDAR scan and find the best open gap to steer into.

        Returns:
            None                - direct path is clear (no obstacle).
            float (world angle) - direction of the best gap closest to goal.
        """
        with self._lock:
            scan = self._scan
            cyaw = self._yaw
        if scan is None or scan.angle_increment == 0.0:
            return None

        ranges = scan.ranges
        n      = len(ranges)
        inc    = scan.angle_increment
        a_min  = scan.angle_min
        safe   = self._obstacle_dist

        # --- Check if the direct forward cone is clear ---
        fwd_idx = int(round(-a_min / inc))
        half    = int(round(self._obstacle_angle / inc))
        blocked = False
        for i in range(fwd_idx - half, fwd_idx + half + 1):
            r = ranges[i % n]
            if scan.range_min < r < safe:
                blocked = True
                break
        if not blocked:
            return None  # direct path clear

        # --- Find open gaps in the full scan ---
        open_mask = []
        for i in range(n):
            r = ranges[i]
            open_mask.append(r > safe or r < scan.range_min)

        # Group consecutive open indices into gap runs
        gaps = []
        start = None
        for i in range(n):
            if open_mask[i]:
                if start is None:
                    start = i
            else:
                if start is not None:
                    gaps.append((start, i - start))
                    start = None
        # Handle wrap-around
        if start is not None:
            if gaps and gaps[0][0] == 0:
                gaps[0] = (start, (n - start) + gaps[0][1])
            else:
                gaps.append((start, n - start))

        if not gaps:
            return None  # everything blocked

        # Minimum gap width: ~20 cm robot body at obstacle_distance
        min_gap = max(3, int(0.3 / (safe * inc + 1e-9)))

        best_dir   = None
        best_score = float('inf')

        for g_start, g_len in gaps:
            if g_len < min_gap:
                continue
            mid_idx   = (g_start + g_len // 2) % n
            gap_angle = a_min + mid_idx * inc  # robot frame
            gap_world = normalize_angle(cyaw + gap_angle)
            score     = abs(normalize_angle(gap_world - goal_angle_world))
            if score < best_score:
                best_score = score
                best_dir   = gap_world

        return best_dir

    # ------------------------------------------------------------------
    # Tilt recovery
    # ------------------------------------------------------------------

    def _try_tilt_recovery(self) -> bool:
        tilt = self._get_tilt()
        self.get_logger().warn(
            f'Tilt detected: {math.degrees(tilt):.1f} deg')

        if not (_GAZEBO_AVAILABLE and self._reset_client is not None):
            self.get_logger().error('Gazebo reset not available.')
            return False

        if not self._reset_client.service_is_ready():
            if not self._reset_client.wait_for_service(timeout_sec=3.0):
                return False

        with self._lock:
            x, y, yaw = self._x, self._y, self._yaw

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'burger'
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.02
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'

        future = self._reset_client.call_async(req)
        deadline = time.monotonic() + 3.0
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)

        self.get_logger().info('Gazebo: robot restored upright.')
        time.sleep(0.3)
        return True

    # ------------------------------------------------------------------
    # Recovery maneuver: back up + rotate toward best gap
    # ------------------------------------------------------------------

    def _recovery_maneuver(self, goal_angle_world, rate):
        """Back up, then rotate toward the best open gap."""
        # Step 1: back up
        self.get_logger().info(
            f'Recovery: backing up for {self._backup_time}s')
        cmd = Twist()
        cmd.linear.x = -self._backup_speed
        end = time.monotonic() + self._backup_time
        while time.monotonic() < end:
            self._cmd_pub.publish(cmd)
            rate.sleep()
        self._stop()
        time.sleep(0.2)

        # Step 2: rotate toward the best gap (or 90 deg if no gap found)
        gap_dir = self._find_best_gap(goal_angle_world)
        cx, cy, cyaw, _ = self._get_pose()
        if gap_dir is not None:
            target_yaw = gap_dir
        else:
            target_yaw = normalize_angle(cyaw + math.pi / 2.0)

        self.get_logger().info(
            f'Recovery: rotating toward '
            f'{math.degrees(target_yaw):.1f} deg')

        rotate_end = time.monotonic() + 3.0
        while time.monotonic() < rotate_end:
            cx, cy, cyaw, _ = self._get_pose()
            err = normalize_angle(target_yaw - cyaw)
            if abs(err) < self._head_tol:
                break
            cmd = Twist()
            cmd.angular.z = self._clamp(self._kp_ang * err, self._max_ang)
            self._cmd_pub.publish(cmd)
            rate.sleep()
        self._stop()
        time.sleep(0.2)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _stop(self):
        self._cmd_pub.publish(Twist())

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _abort(self, goal_handle, reason: str):
        self._stop()
        cx, cy, cyaw, _ = self._get_pose()
        result = AbsoluteMove.Result()
        result.success = False
        result.message = reason
        result.final_x = cx; result.final_y = cy
        result.final_heading = cyaw
        self.get_logger().error(f'Goal aborted: {reason}')
        goal_handle.abort()
        return result

    def _cancelled(self, goal_handle):
        self._stop()
        cx, cy, cyaw, _ = self._get_pose()
        result = AbsoluteMove.Result()
        result.success = False
        result.message = 'Goal cancelled.'
        result.final_x = cx; result.final_y = cy
        result.final_heading = cyaw
        goal_handle.canceled()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AbsoluteMoveNode()

    def _shutdown_handler(signum, frame):
        node.get_logger().warn('Shutdown signal received.')
        node._stop()
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
