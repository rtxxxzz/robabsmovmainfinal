"""
Absolute Move Client — interactive CLI and command-line goal sender.

Usage:
  # Interactive mode (prompts for goals):
  ros2 run turtlebot3_absolute_move absolute_move_client

  # Single-shot from command line:
  ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 1.0 0.5 90

  The heading argument is in DEGREES for user convenience.
  Internally it is converted to radians before sending.
"""

import math
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot3_absolute_move_interfaces.action import AbsoluteMove


class AbsoluteMoveClient(Node):
    """Simple action client with CLI interface."""

    def __init__(self):
        super().__init__('absolute_move_client')
        self._client = ActionClient(
            self, AbsoluteMove, '/absolute_move_node/absolute_move')
        self._goal_handle = None
        self._done_event = threading.Event()

    def send_goal(self, x: float, y: float, heading_deg: float):
        """Send an absolute move goal. Heading is in degrees."""
        self.get_logger().info('Waiting for action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. '
                'Is absolute_move_node running?')
            return False

        heading_rad = math.radians(heading_deg)

        goal_msg = AbsoluteMove.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y
        goal_msg.target_heading = heading_rad

        self.get_logger().info(
            f'Sending goal: x={x:.3f}, y={y:.3f}, '
            f'heading={heading_deg:.1f} deg ({heading_rad:.3f} rad)')

        self._done_event.clear()
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        # Wait for completion
        self._done_event.wait()
        return True

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            self._done_event.set()
            return

        self.get_logger().info('Goal accepted.')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'Goal reached: ({result.final_x:.3f}, '
                f'{result.final_y:.3f}, '
                f'{math.degrees(result.final_heading):.1f} deg)')
            self.get_logger().info(
                f'  Position error: {result.position_error:.4f} m, '
                f'Heading error: {math.degrees(result.heading_error):.2f} deg')
        else:
            self.get_logger().warn(f'Goal result: {result.message}')
            self.get_logger().info(
                f'  Final pose: ({result.final_x:.3f}, '
                f'{result.final_y:.3f}, '
                f'{math.degrees(result.final_heading):.1f} deg)')
            if result.position_error > 0:
                self.get_logger().info(
                    f'  Remaining distance: {result.position_error:.3f} m')
        self._done_event.set()

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  [{fb.phase}] pos=({fb.current_x:.3f}, {fb.current_y:.3f}) '
            f'heading={math.degrees(fb.current_heading):.1f} deg '
            f'dist_rem={fb.distance_remaining:.3f} m '
            f'head_err={math.degrees(fb.heading_error):.1f} deg')

    def cancel_goal(self):
        """Cancel the current goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            self._goal_handle.cancel_goal_async()


def _parse_cli_goal(args):
    """Parse --goal X Y HEADING_DEG from command line args."""
    if '--goal' in args:
        idx = args.index('--goal')
        try:
            x = float(args[idx + 1])
            y = float(args[idx + 2])
            h = float(args[idx + 3])
            return x, y, h
        except (IndexError, ValueError):
            print('Usage: --goal <x> <y> <heading_degrees>')
            sys.exit(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    client = AbsoluteMoveClient()

    # Spin in background so callbacks are processed
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    # Check for single-shot CLI goal
    cli_goal = _parse_cli_goal(sys.argv)
    if cli_goal:
        x, y, h = cli_goal
        client.send_goal(x, y, h)
        # Clean shutdown: destroy node first, then shutdown rclpy,
        # then join the spin thread (which will exit once shutdown).
        client.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        return

    # Interactive mode
    print('\n' + '=' * 60)
    print('  TurtleBot3 Absolute Move — Interactive Client')
    print('=' * 60)
    print('Enter goals as: x y heading_degrees')
    print('  Example: 1.0 0.5 90')
    print('Type "quit" or Ctrl+C to exit.')
    print('=' * 60 + '\n')

    try:
        while True:
            try:
                line = input('Goal (x y heading_deg): ').strip()
            except EOFError:
                break

            if not line or line.lower() in ('quit', 'exit', 'q'):
                break

            parts = line.split()
            if len(parts) != 3:
                print('  Enter exactly 3 values: x y heading_degrees')
                continue

            try:
                x, y, h = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                print('  Invalid numbers. Example: 1.0 0.5 90')
                continue

            client.send_goal(x, y, h)
            print()  # blank line between goals

    except KeyboardInterrupt:
        print('\nCancelling...')
        client.cancel_goal()

    client.destroy_node()
    rclpy.try_shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
