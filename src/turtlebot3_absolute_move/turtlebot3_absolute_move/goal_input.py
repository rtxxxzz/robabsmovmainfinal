"""
Interactive Goal Input — send absolute move commands from the terminal.

Run this tool in a SEPARATE terminal while the simulation/robot is running:

    ros2 run turtlebot3_absolute_move goal_input

The tool connects to the already-running absolute_move action server
and lets you type in goal coordinates interactively.

Workflow:
  Terminal 1:  ros2 launch turtlebot3_absolute_move pipeline.launch.py ...
               (exploration runs, map is saved, pipeline node exits —
                but gazebo, SLAM, and absolute_move_node keep running)

  Terminal 2:  ros2 run turtlebot3_absolute_move goal_input
               (enter goals interactively; robot moves)

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
from rclpy.executors import MultiThreadedExecutor

from turtlebot3_absolute_move_interfaces.action import AbsoluteMove


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoalInputNode(Node):
    """Lightweight node that sends goals to the absolute_move action server."""

    def __init__(self):
        super().__init__('goal_input_node')
        self._action_client = ActionClient(
            self, AbsoluteMove, '/absolute_move_node/absolute_move')

    def send_goal(self, x, y, heading_deg):
        """Send a goal and block until result.

        Returns:
            (success, final_x, final_y, message)
        """
        goal_msg = AbsoluteMove.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_heading = _normalize_angle(math.radians(heading_deg))

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

        # Wait for result (up to 120 s per goal)
        done_event.wait(timeout=120.0)

        if result_holder[0] is None:
            return False, 0.0, 0.0, 'Timeout waiting for result'

        result = result_holder[0]
        return result.success, result.final_x, result.final_y, result.message


def main(args=None):
    rclpy.init(args=args)
    node = GoalInputNode()

    executor = MultiThreadedExecutor(num_threads=2)
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
        print('Make sure the simulation/robot is running:')
        print('  ros2 launch turtlebot3_absolute_move pipeline.launch.py '
              'world:=turtlebot3_world ...')
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        return
    print('Connected!\n')

    print('=' * 60)
    print('  Interactive Absolute Move')
    print('=' * 60)
    print('Enter goal coordinates to move the robot.')
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
                # Allow omitting heading (default 0)
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
            print(f'  → Moving to ({x:.2f}, {y:.2f}, {h:.1f}°)...')

            success, fx, fy, msg = node.send_goal(x, y, h)

            if success:
                print(f'  ✓ Reached ({fx:.3f}, {fy:.3f})')
            else:
                print(f'  ✗ Failed: {msg}')
            print()

    except KeyboardInterrupt:
        print('\nExiting.')

    print(f'Total goals sent: {goal_count}')

    try:
        node.destroy_node()
    except Exception:
        pass
    rclpy.try_shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
