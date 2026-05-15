"""
Scan Relay — Bridges Best Effort LiDAR scans to Reliable QoS.

The TurtleBot3 LDS driver publishes /scan with Best Effort QoS.
SLAM Toolbox subscribes with Reliable QoS (hardcoded, no parameter).
This tiny relay subscribes to /scan with Best Effort and re-publishes
on /scan_reliable with Reliable QoS so SLAM Toolbox receives data.

Usage:
  ros2 run turtlebot3_absolute_move scan_relay
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):

    def __init__(self):
        super().__init__('scan_relay')

        # Subscribe with Best Effort (matches LDS driver)
        self.create_subscription(
            LaserScan, '/scan', self._cb, qos_profile_sensor_data)

        # Publish with Reliable (matches SLAM Toolbox expectation)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self._pub = self.create_publisher(
            LaserScan, '/scan_reliable', reliable_qos)

        self.get_logger().info(
            'Scan relay: /scan (Best Effort) → /scan_reliable (Reliable)')

    def _cb(self, msg):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
