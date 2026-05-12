"""
Pipeline Orchestrator — Autonomous navigation pipeline for TurtleBot3.

Orchestrates the full workflow: SLAM exploration → A* path planning →
absolute move execution. Works in both Gazebo simulation and real hardware.

Usage:
  # Full pipeline: explore then navigate to goals from file
  ros2 run turtlebot3_absolute_move pipeline -- --goals goals.yaml

  # Full pipeline with interactive goals after exploration
  ros2 run turtlebot3_absolute_move pipeline

  # Skip exploration, use existing map
  ros2 run turtlebot3_absolute_move pipeline -- --map ~/my_map.yaml --goals goals.yaml

  # Explore only (build and save the map)
  ros2 run turtlebot3_absolute_move pipeline -- --explore-only

  # Explore then navigate with custom save path
  ros2 run turtlebot3_absolute_move pipeline -- --save-map ~/my_map
"""

import math
import os
import signal
import subprocess
import sys
import threading
import time

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

from turtlebot3_absolute_move_interfaces.action import AbsoluteMove
from turtlebot3_absolute_move.frontier_explorer import FrontierExplorer
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


class PipelineOrchestrator(Node):
    """Master controller for the autonomous navigation pipeline."""

    def __init__(self):
        super().__init__('pipeline_node')

        # --- Parameters ---
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_timeout', 300.0)
        self.declare_parameter('frontier_cost_weight', 0.5)
        self.declare_parameter('no_frontier_patience', 3)
        self.declare_parameter('inflation_radius', 0.18)
        self.declare_parameter('path_simplification', True)
        self.declare_parameter('waypoint_spacing', 0.3)
        self.declare_parameter('unknown_as_free', True)
        self.declare_parameter('max_replan_attempts', 3)
        self.declare_parameter('waypoint_reached_tolerance', 0.10)
        self.declare_parameter('max_frontier_skip', 5)
        self.declare_parameter('frontier_blacklist_radius', 0.3)

        self._read_params()

        # --- State ---
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = 0.0
        self._odom_received = False
        self._latest_map = None
        self._map_received = False

        # Frontier blacklist — unreachable frontiers we should skip
        self._frontier_blacklist = set()

        # RViz subprocess handle
        self._rviz_proc = None

        # --- Modules ---
        self._explorer = FrontierExplorer(
            min_frontier_size=self._min_frontier_size,
            cost_weight=self._frontier_cost_weight)
        self._planner = PathPlanner(
            inflation_radius=self._inflation_radius,
            waypoint_spacing=self._waypoint_spacing,
            unknown_as_free=self._unknown_as_free,
            simplify=self._path_simplification)

        # --- ROS interfaces ---
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            OccupancyGrid, 'map', self._map_cb, 10,
            callback_group=self._cb_group)

        self._action_client = ActionClient(
            self, AbsoluteMove, '/absolute_move_node/absolute_move')

        # Visualisation publishers
        self._path_pub = self.create_publisher(Path, 'planned_path', 10)
        self._frontier_pub = self.create_publisher(
            MarkerArray, 'frontiers', 10)
        self._goal_marker_pub = self.create_publisher(
            Marker, 'exploration_goal', 10)

        self.get_logger().info('Pipeline orchestrator initialised.')

    def _read_params(self):
        self._min_frontier_size = self.get_parameter('min_frontier_size').value
        self._exploration_timeout = self.get_parameter('exploration_timeout').value
        self._frontier_cost_weight = self.get_parameter('frontier_cost_weight').value
        self._no_frontier_patience = self.get_parameter('no_frontier_patience').value
        self._inflation_radius = self.get_parameter('inflation_radius').value
        self._path_simplification = self.get_parameter('path_simplification').value
        self._waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self._unknown_as_free = self.get_parameter('unknown_as_free').value
        self._max_replan = self.get_parameter('max_replan_attempts').value
        self._wp_tol = self.get_parameter('waypoint_reached_tolerance').value
        self._max_frontier_skip = self.get_parameter('max_frontier_skip').value
        self._blacklist_radius = self.get_parameter('frontier_blacklist_radius').value

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        with self._lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
            self._odom_received = True

    def _map_cb(self, msg):
        with self._lock:
            self._latest_map = msg
            self._map_received = True

    def _get_pose(self):
        with self._lock:
            return self._x, self._y, self._yaw

    def _get_map(self):
        with self._lock:
            return self._latest_map

    # ------------------------------------------------------------------
    # Pipeline entry point
    # ------------------------------------------------------------------

    def run_pipeline(self, goals_file=None, map_file=None,
                     explore_only=False, save_map_path=None,
                     launch_rviz=True):
        """Run the full autonomous pipeline.

        Args:
            goals_file: Path to YAML file with predetermined goals.
            map_file: Path to saved map (skip exploration).
            explore_only: If True, only explore and save the map.
            save_map_path: Path prefix to save the map (e.g. ~/my_map).
            launch_rviz: If True, auto-launch RViz2 for real-time visualization.
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('  TurtleBot3 Autonomous Navigation Pipeline')
        self.get_logger().info('=' * 60)

        # --- Wait for systems ---
        if not self._wait_for_systems():
            return

        # RViz is launched by pipeline.launch.py with proper use_sim_time

        # --- Phase A: Exploration ---
        if map_file:
            self.get_logger().info(f'Skipping exploration — using map: {map_file}')
            self.get_logger().info('Waiting for SLAM to load map data...')
            # Wait for map topic to start publishing
            timeout = time.monotonic() + 30.0
            while not self._map_received and time.monotonic() < timeout:
                time.sleep(0.5)
            if not self._map_received:
                self.get_logger().warn(
                    'No /map received. Path planning may not work without a map.')
        else:
            self.get_logger().info('')
            self.get_logger().info('-' * 50)
            self.get_logger().info('  Phase A: Autonomous Exploration')
            self.get_logger().info('-' * 50)
            self._run_exploration()

            # Save the map
            save_path = save_map_path or os.path.expanduser('~/pipeline_map')
            self._save_map(save_path)

        if explore_only:
            self.get_logger().info('Explore-only mode — pipeline complete.')
            return

        # --- Phase B+C: Load goals → Plan → Execute ---
        goals = self._load_goals(goals_file)
        if not goals:
            self.get_logger().warn('No goals to execute. Pipeline complete.')
            return

        self.get_logger().info('')
        self.get_logger().info('-' * 50)
        self.get_logger().info(f'  Phase B+C: Navigate to {len(goals)} goals')
        self.get_logger().info('-' * 50)

        results = []
        for i, (gx, gy, gh_deg) in enumerate(goals):
            self.get_logger().info('')
            self.get_logger().info(
                f'=== Goal {i+1}/{len(goals)}: '
                f'({gx:.2f}, {gy:.2f}, {gh_deg:.1f}°) ===')
            success = self._navigate_to_goal(gx, gy, gh_deg)
            results.append((gx, gy, gh_deg, success))

        # --- Summary ---
        self._print_summary(results)

    # ------------------------------------------------------------------
    # Phase A: Exploration
    # ------------------------------------------------------------------

    def _run_exploration(self):
        """Autonomous frontier-based exploration with frontier blacklisting."""
        start_time = time.monotonic()
        no_frontier_count = 0
        consecutive_skips = 0

        self.get_logger().info('Starting frontier-based exploration...')
        self.get_logger().info(
            f'Timeout: {self._exploration_timeout:.0f}s, '
            f'Patience: {self._no_frontier_patience} empty cycles, '
            f'Max skip: {self._max_frontier_skip}')

        exploration_goal_count = 0

        while True:
            elapsed = time.monotonic() - start_time
            if elapsed > self._exploration_timeout:
                self.get_logger().info(
                    f'Exploration timeout ({self._exploration_timeout:.0f}s) reached.')
                break

            grid = self._get_map()
            if grid is None:
                self.get_logger().info('Waiting for /map...')
                time.sleep(2.0)
                continue

            cx, cy, _ = self._get_pose()
            target = self._explorer.find_best_frontier(
                grid, cx, cy,
                blacklist=self._frontier_blacklist,
                blacklist_radius=self._blacklist_radius)

            # Visualise frontiers
            self._publish_frontier_markers(grid)

            if target is None:
                # Check if we have blacklisted frontiers — maybe clear and retry
                if self._frontier_blacklist:
                    self.get_logger().info(
                        f'No non-blacklisted frontiers. '
                        f'Clearing {len(self._frontier_blacklist)} '
                        f'blacklisted entries and retrying...')
                    self._frontier_blacklist.clear()
                    consecutive_skips = 0
                    continue

                no_frontier_count += 1
                progress = self._explorer.get_exploration_progress(grid)
                self.get_logger().info(
                    f'No frontiers found ({no_frontier_count}/'
                    f'{self._no_frontier_patience}). '
                    f'Map coverage: {progress*100:.1f}%')
                if no_frontier_count >= self._no_frontier_patience:
                    self.get_logger().info(
                        'Exploration complete — no more frontiers.')
                    break
                time.sleep(3.0)
                continue

            no_frontier_count = 0
            exploration_goal_count += 1
            gx, gy = target

            self.get_logger().info(
                f'Exploration goal #{exploration_goal_count}: '
                f'({gx:.2f}, {gy:.2f}) '
                f'[blacklist: {len(self._frontier_blacklist)}]')

            # Publish goal marker for RViz
            self._publish_goal_marker(gx, gy)

            # Plan path to frontier
            path = self._plan_path(gx, gy)
            if path is None:
                # Blacklist this unreachable frontier
                self._frontier_blacklist.add((round(gx, 2), round(gy, 2)))
                consecutive_skips += 1
                self.get_logger().warn(
                    f'No path to frontier ({gx:.2f}, {gy:.2f}), '
                    f'blacklisted. ({consecutive_skips}/'
                    f'{self._max_frontier_skip} skips)')

                if consecutive_skips >= self._max_frontier_skip:
                    # Too many consecutive skips — clear blacklist and retry
                    self.get_logger().info(
                        f'Hit max skip limit ({self._max_frontier_skip}). '
                        f'Clearing blacklist and retrying...')
                    self._frontier_blacklist.clear()
                    consecutive_skips = 0
                    # Let SLAM update with a longer pause
                    time.sleep(3.0)

                time.sleep(1.0)
                continue

            # Successful path found — reset skip counter
            consecutive_skips = 0

            # Execute path via waypoints
            self._execute_waypoints(path, heading_deg=None)

            # Brief pause to let SLAM update
            time.sleep(1.0)

        progress = self._explorer.get_exploration_progress(
            self._get_map()) if self._get_map() else 0.0
        self.get_logger().info(
            f'Exploration finished. Map coverage: {progress*100:.1f}%. '
            f'Goals executed: {exploration_goal_count}. '
            f'Blacklisted: {len(self._frontier_blacklist)}.')

    # ------------------------------------------------------------------
    # Phase B: Path Planning
    # ------------------------------------------------------------------

    def _plan_path(self, goal_x, goal_y):
        """Plan a path from current pose to (goal_x, goal_y)."""
        grid = self._get_map()
        if grid is None:
            self.get_logger().warn('No map available for planning.')
            return None

        cx, cy, _ = self._get_pose()
        path = self._planner.plan(grid, cx, cy, goal_x, goal_y)

        if path:
            self._publish_path(path)
            self.get_logger().info(
                f'Path planned: {len(path)} waypoints')
        else:
            self.get_logger().warn(
                f'A* found no path to ({goal_x:.2f}, {goal_y:.2f})')
        return path

    # ------------------------------------------------------------------
    # Phase C: Execution
    # ------------------------------------------------------------------

    def _navigate_to_goal(self, goal_x, goal_y, heading_deg):
        """Plan and execute navigation to a goal with replanning."""
        for attempt in range(1, self._max_replan + 1):
            path = self._plan_path(goal_x, goal_y)
            if path is None:
                self.get_logger().warn(
                    f'Planning failed (attempt {attempt}/{self._max_replan})')
                time.sleep(2.0)
                continue

            # Last waypoint heading = desired heading
            success = self._execute_waypoints(path, heading_deg)
            if success:
                return True

            self.get_logger().warn(
                f'Execution failed (attempt {attempt}/{self._max_replan}), '
                f'replanning...')
            time.sleep(1.0)

        self.get_logger().error(
            f'Goal ({goal_x:.2f}, {goal_y:.2f}, {heading_deg:.1f}°) '
            f'failed after {self._max_replan} attempts.')
        return False

    def _execute_waypoints(self, path, heading_deg=None):
        """Send waypoints one-by-one to the absolute_move action server.

        For intermediate waypoints, the heading is set to point toward
        the next waypoint.  For the final waypoint, heading_deg is used
        (or 0 if None).
        """
        if not path:
            return False

        for i, (wx, wy) in enumerate(path):
            is_last = (i == len(path) - 1)

            if is_last and heading_deg is not None:
                h_rad = math.radians(heading_deg)
            elif not is_last:
                # Point toward the next waypoint
                nx, ny = path[i + 1]
                h_rad = math.atan2(ny - wy, nx - wx)
            else:
                h_rad = 0.0

            self.get_logger().info(
                f'  Waypoint {i+1}/{len(path)}: '
                f'({wx:.2f}, {wy:.2f}, {math.degrees(h_rad):.1f}°)')

            success = self._send_absolute_move(wx, wy, h_rad)
            if not success and is_last:
                return False
            # For intermediate waypoints, partial success is acceptable

        return True

    def _send_absolute_move(self, x, y, heading_rad):
        """Send a single goal to the absolute_move action server."""
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
            self.get_logger().error('Timeout waiting for goal acceptance.')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(result_cb)

        # Wait for result (up to 120s per waypoint)
        done_event.wait(timeout=120.0)

        if result_holder[0] is None:
            self.get_logger().warn('Timeout waiting for result.')
            return False

        result = result_holder[0]
        if result.success:
            self.get_logger().info(
                f'  ✓ Reached ({result.final_x:.3f}, {result.final_y:.3f})')
        else:
            self.get_logger().warn(f'  ✗ {result.message}')

        return result.success

    # ------------------------------------------------------------------
    # System checks
    # ------------------------------------------------------------------

    def _wait_for_systems(self):
        """Wait for odom and action server to be ready."""
        self.get_logger().info('Waiting for systems...')

        # Wait for odom
        timeout = time.monotonic() + 15.0
        while not self._odom_received and time.monotonic() < timeout:
            time.sleep(0.2)
        if not self._odom_received:
            self.get_logger().error('No /odom received — is the robot running?')
            return False
        self.get_logger().info('  ✓ Odometry received')

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'absolute_move action server not available — '
                'is absolute_move_node running?')
            return False
        self.get_logger().info('  ✓ Action server connected')

        self.get_logger().info('  All systems ready.')
        return True

    # ------------------------------------------------------------------
    # Goal loading
    # ------------------------------------------------------------------

    def _load_goals(self, goals_file):
        """Load goals from YAML file or interactive prompt."""
        if goals_file:
            return self._load_goals_from_file(goals_file)
        return self._interactive_goals()

    def _load_goals_from_file(self, filepath):
        """Load goals from a YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            goals = data.get('goals', [])
            parsed = []
            for g in goals:
                if len(g) == 3:
                    parsed.append((float(g[0]), float(g[1]), float(g[2])))
                else:
                    self.get_logger().warn(f'Invalid goal entry: {g}')
            self.get_logger().info(
                f'Loaded {len(parsed)} goals from {filepath}')
            return parsed
        except Exception as e:
            self.get_logger().error(f'Failed to load goals: {e}')
            return []

    def _interactive_goals(self):
        """Prompt user for goals interactively."""
        goals = []
        print('\n' + '=' * 60)
        print('  Enter navigation goals')
        print('=' * 60)
        print('Format: x y heading_degrees')
        print('  Example: 1.0 0.5 90')
        print('Type "done" to start navigation, "quit" to exit.')
        print('=' * 60 + '\n')

        try:
            while True:
                try:
                    line = input(f'Goal {len(goals)+1} (x y heading_deg): ').strip()
                except EOFError:
                    break

                if not line or line.lower() in ('done', 'd'):
                    break
                if line.lower() in ('quit', 'exit', 'q'):
                    return []

                parts = line.split()
                if len(parts) != 3:
                    print('  Enter exactly 3 values: x y heading_degrees')
                    continue
                try:
                    x, y, h = float(parts[0]), float(parts[1]), float(parts[2])
                    goals.append((x, y, h))
                    print(f'  Added goal {len(goals)}: ({x:.2f}, {y:.2f}, {h:.1f}°)')
                except ValueError:
                    print('  Invalid numbers. Example: 1.0 0.5 90')
        except KeyboardInterrupt:
            print('\nCancelled.')
            return []

        return goals

    # ------------------------------------------------------------------
    # RViz auto-launch
    # ------------------------------------------------------------------

    def _launch_rviz(self):
        """Launch RViz2 with the pipeline config for real-time map visualization."""
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('turtlebot3_absolute_move')
            rviz_config = os.path.join(pkg_share, 'config', 'pipeline.rviz')

            if not os.path.exists(rviz_config):
                self.get_logger().warn(
                    f'RViz config not found: {rviz_config}')
                return

            self.get_logger().info('Launching RViz2 for real-time visualization...')
            self._rviz_proc = subprocess.Popen(
                ['rviz2', '-d', rviz_config,
                 '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            self.get_logger().info('  ✓ RViz2 launched — map, scan, path, '
                                  'frontiers all visible')
        except Exception as e:
            self.get_logger().warn(f'Could not launch RViz2: {e}')

    def _kill_rviz(self):
        """Terminate the auto-launched RViz2 subprocess."""
        if self._rviz_proc is not None:
            try:
                self._rviz_proc.terminate()
                self._rviz_proc.wait(timeout=5)
            except Exception:
                try:
                    self._rviz_proc.kill()
                except Exception:
                    pass
            self._rviz_proc = None

    # ------------------------------------------------------------------
    # Map saving
    # ------------------------------------------------------------------

    def _save_map(self, path_prefix):
        """Save the current SLAM map using nav2_map_server."""
        self.get_logger().info(f'Saving map to {path_prefix}...')
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', path_prefix],
                capture_output=True, text=True, timeout=30)
            if result.returncode == 0:
                self.get_logger().info(
                    f'Map saved: {path_prefix}.pgm + {path_prefix}.yaml')
            else:
                self.get_logger().warn(
                    f'map_saver_cli returned {result.returncode}: '
                    f'{result.stderr.strip()}')
                self.get_logger().warn(
                    'Map saving failed — nav2_map_server may not be installed. '
                    'Install with: sudo apt install ros-humble-nav2-map-server')
        except FileNotFoundError:
            self.get_logger().warn(
                'nav2_map_server not found. '
                'Install: sudo apt install ros-humble-nav2-map-server')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Map save timed out.')

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def _publish_path(self, path):
        """Publish the planned path for RViz visualisation."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _publish_frontier_markers(self, grid_msg):
        """Publish frontier centroids as RViz markers."""
        centroids = self._explorer.get_all_frontiers(grid_msg)
        marker_array = MarkerArray()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (fx, fy) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.8
            m.lifetime.sec = 5
            marker_array.markers.append(m)

        self._frontier_pub.publish(marker_array)

    def _publish_goal_marker(self, gx, gy):
        """Publish the current exploration goal as an RViz marker."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'exploration_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.3
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime.sec = 30
        self._goal_marker_pub.publish(m)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    def _print_summary(self, results):
        """Print a final summary of all goal results."""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  Pipeline Summary')
        self.get_logger().info('=' * 60)

        succeeded = sum(1 for _, _, _, s in results if s)
        total = len(results)

        for i, (gx, gy, gh, success) in enumerate(results):
            status = '✓' if success else '✗'
            self.get_logger().info(
                f'  {status} Goal {i+1}: ({gx:.2f}, {gy:.2f}, {gh:.1f}°)')

        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'  Result: {succeeded}/{total} goals reached')
        self.get_logger().info('=' * 60)


def _parse_args(argv):
    """Parse CLI arguments."""
    args = {
        'goals_file': None,
        'map_file': None,
        'explore_only': False,
        'save_map': None,
    }
    i = 0
    while i < len(argv):
        if argv[i] == '--goals' and i + 1 < len(argv):
            args['goals_file'] = argv[i + 1]
            i += 2
        elif argv[i] == '--map' and i + 1 < len(argv):
            args['map_file'] = argv[i + 1]
            i += 2
        elif argv[i] == '--explore-only':
            args['explore_only'] = True
            i += 1
        elif argv[i] == '--save-map' and i + 1 < len(argv):
            args['save_map'] = argv[i + 1]
            i += 2
        else:
            i += 1
    return args


def main(args=None):
    rclpy.init(args=args)
    node = PipelineOrchestrator()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Spin in background
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Parse CLI args
    cli_args = _parse_args(sys.argv)

    def shutdown_handler(signum, frame):
        try:
            node.get_logger().warn('Shutdown signal.')
        except Exception:
            pass
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        node.run_pipeline(
            goals_file=cli_args['goals_file'],
            map_file=cli_args['map_file'],
            explore_only=cli_args['explore_only'],
            save_map_path=cli_args['save_map'],
            launch_rviz=cli_args.get('rviz', True))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Pipeline error: {e}')
    finally:
        try:
            node._kill_rviz()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
