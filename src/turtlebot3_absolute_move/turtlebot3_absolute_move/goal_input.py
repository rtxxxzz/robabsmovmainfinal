"""
Interactive Goal Input — map-aware absolute move commands from the terminal.

Run this tool in a SEPARATE terminal while the simulation/robot is running:

    ros2 run turtlebot3_absolute_move goal_input

The tool connects to the running absolute_move action server, subscribes
to /map and /odom, and uses A* path planning on the SLAM occupancy grid
to find the optimal collision-free path to each goal.

Workflow:
  Terminal 1:  ros2 launch turtlebot3_absolute_move pipeline.launch.py ...
               (exploration runs, map is saved, pipeline node exits —
                but gazebo, SLAM, and absolute_move_node keep running)

  Terminal 2:  ros2 run turtlebot3_absolute_move goal_input
               (enter goals interactively; robot plans optimal path & moves)

  When done:   Ctrl+C Terminal 1 to stop the simulation
"""

import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from nav_msgs.msg import OccupancyGrid
from turtlebot3_absolute_move_interfaces.action import AbsoluteMove
from turtlebot3_absolute_move.path_planner import PathPlanner


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalInputNode(Node):
    """Map-aware interactive goal input node.

    Subscribes to /map and /odom, uses A* path planning on the SLAM
    occupancy grid, and sends optimal waypoints to absolute_move.
    """

    def __init__(self):
        super().__init__('goal_input_node')

        self._cb_group = ReentrantCallbackGroup()

        # --- State ---
        self._lock = threading.Lock()
        self._latest_map = None
        self._map_received = False

        # --- TF2 for map-frame localization ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self)

        # --- Path planner (same as pipeline) ---
        self._planner = PathPlanner(
            inflation_radius=0.18,
            waypoint_spacing=0.3,
            unknown_as_free=True,
            simplify=True)

        # --- ROS interfaces ---
        self._action_client = ActionClient(
            self, AbsoluteMove, '/absolute_move_node/absolute_move')

        self.create_subscription(
            OccupancyGrid, 'map', self._map_cb, 10,
            callback_group=self._cb_group)

    def _map_cb(self, msg):
        with self._lock:
            self._latest_map = msg
            self._map_received = True

    def _get_pose(self):
        """Get robot pose in the MAP frame via TF2."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = _yaw_from_quaternion(t.transform.rotation)
            return x, y, yaw
        except Exception:
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                x = t.transform.translation.x
                y = t.transform.translation.y
                yaw = _yaw_from_quaternion(t.transform.rotation)
                return x, y, yaw
            except Exception:
                return 0.0, 0.0, 0.0

    def _map_to_odom(self, map_x, map_y, map_yaw):
        """Transform a map-frame pose to the odom frame."""
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'map',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tyaw = _yaw_from_quaternion(t.transform.rotation)
            cos_t = math.cos(tyaw)
            sin_t = math.sin(tyaw)
            odom_x = cos_t * map_x - sin_t * map_y + tx
            odom_y = sin_t * map_x + cos_t * map_y + ty
            odom_yaw = _normalize_angle(map_yaw + tyaw)
            return odom_x, odom_y, odom_yaw
        except Exception:
            return map_x, map_y, map_yaw

    def _get_map(self):
        with self._lock:
            return self._latest_map

    def plan_and_execute(self, goal_x, goal_y, heading_deg):
        """Plan an A* path on the map and execute via waypoints.

        Returns:
            (success, final_x, final_y, message)
        """
        grid = self._get_map()
        cx, cy, cyaw = self._get_pose()

        if grid is None:
            print('  ⚠ No /map received — falling back to direct move')
            return self._send_direct_goal(goal_x, goal_y, heading_deg)

        # Plan path using A*
        path = self._planner.plan(grid, cx, cy, goal_x, goal_y)

        if path is None:
            print('  ⚠ A* found no path — falling back to direct move')
            return self._send_direct_goal(goal_x, goal_y, heading_deg)

        print(f'  ✓ A* path planned: {len(path)} waypoints')

        # Execute waypoints sequentially
        for i, (wx, wy) in enumerate(path):
            is_last = (i == len(path) - 1)

            if is_last:
                h_rad = math.radians(heading_deg)
            elif i < len(path) - 1:
                nx, ny = path[i + 1]
                h_rad = math.atan2(ny - wy, nx - wx)
            else:
                h_rad = 0.0

            # Transform map→odom before sending to action server
            ox, oy, oh = self._map_to_odom(wx, wy, h_rad)

            print(f'    Waypoint {i+1}/{len(path)}: '
                  f'({wx:.2f}, {wy:.2f}, {math.degrees(h_rad):.1f}°)')

            success, fx, fy, msg = self._send_single_goal(ox, oy, oh)

            if not success and is_last:
                return False, fx, fy, msg

        return success, fx, fy, msg

    def _send_direct_goal(self, x, y, heading_deg):
        """Send a single direct goal (no path planning)."""
        h_rad = math.radians(heading_deg)
        return self._send_single_goal(x, y, h_rad)

    def _send_single_goal(self, x, y, heading_rad):
        """Send a single goal to the action server and wait for result.

        Returns:
            (success, final_x, final_y, message)
        """
        goal_msg = AbsoluteMove.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_heading = _normalize_angle(heading_rad)

        done_event = threading.Event()
        result_holder = [None]

        def result_cb(future):
            result_holder[0] = future.result().result
            done_event.set()

        future = self._action_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            return False, 0.0, 0.0, 'Timeout waiting for goal acceptance'

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False, 0.0, 0.0, 'Goal rejected by action server'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_cb)

        # Wait for result (up to 120 s per waypoint)
        done_event.wait(timeout=120.0)

        if result_holder[0] is None:
            return False, 0.0, 0.0, 'Timeout waiting for result'

        result = result_holder[0]
        return result.success, result.final_x, result.final_y, result.message


def main(args=None):
    rclpy.init(args=args)
    node = GoalInputNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Graceful shutdown on Ctrl+C
    def shutdown_handler(signum, frame):
        print('\nShutting down...')
        rclpy.try_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    # Wait for the action server
    print('Connecting to absolute_move action server...')
    if not node._action_client.wait_for_server(timeout_sec=15.0):
        print('ERROR: absolute_move action server not available!')
        print('Make sure the simulation/robot is running.')
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        return
    print('  ✓ Action server connected')

    # Wait for TF
    print('Waiting for TF (map → base_footprint)...')
    deadline = time.monotonic() + 10.0
    tf_ok = False
    while time.monotonic() < deadline:
        try:
            node._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            tf_ok = True
            break
        except Exception:
            try:
                node._tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                tf_ok = True
                break
            except Exception:
                time.sleep(0.2)
    if tf_ok:
        cx, cy, cyaw = node._get_pose()
        print(f'  ✓ TF received — robot at ({cx:.2f}, {cy:.2f}, '
              f'{math.degrees(cyaw):.1f}°) in map frame')
    else:
        print('  ⚠ No TF yet — will retry when sending goals')

    # Wait for map
    print('Waiting for /map...')
    deadline = time.monotonic() + 10.0
    while not node._map_received and time.monotonic() < deadline:
        time.sleep(0.2)
    if node._map_received:
        grid = node._get_map()
        print(f'  ✓ Map received — {grid.info.width}×{grid.info.height} '
              f'cells @ {grid.info.resolution:.2f} m/cell')
    else:
        print('  ⚠ No /map received — will use direct moves (no A* planning)')

    print()
    print('=' * 60)
    print('  Interactive Absolute Move (A* Path Planning)')
    print('=' * 60)
    print('Enter goal coordinates to move the robot.')
    print('The robot will plan an optimal path using A* on the SLAM map.')
    print()
    print('Format:  x y heading_degrees')
    print('  Example: 1.0 0.5 90')
    print('  Example: -2.0 1.0 0')
    print()
    print('Type "quit" or press Ctrl+C to exit.')
    print('=' * 60)
    print()

    goal_count = 0

    try:
        while True:
            try:
                line = input(f'Goal {goal_count + 1} (x y heading_deg): ').strip()
            except EOFError:
                break

            if not line:
                continue
            if line.lower() in ('quit', 'exit', 'q'):
                break

            parts = line.split()
            if len(parts) == 2:
                parts.append('0')
            if len(parts) != 3:
                print('  ⚠ Enter 2 or 3 values: x y [heading_degrees]')
                print('  Example: 1.0 0.5 90')
                continue

            try:
                x, y, h = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                print('  ⚠ Invalid numbers. Example: 1.0 0.5 90')
                continue

            goal_count += 1
            print(f'  → Planning path to ({x:.2f}, {y:.2f}, {h:.1f}°)...')

            success, fx, fy, msg = node.plan_and_execute(x, y, h)

            if success:
                print(f'  ✓ Goal reached ({fx:.3f}, {fy:.3f})')
            else:
                print(f'  ✗ Failed: {msg}')
            print()

    except KeyboardInterrupt:
        print('\nExiting.')

    print(f'Total goals executed: {goal_count}')

    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.try_shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
