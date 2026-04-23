"""
RViz2 Launch File — TurtleBot3 Absolute Move.

Launches RViz2 with a configuration that shows the robot model,
odometry, TF frames, and laser scan.

Usage:
  ros2 launch turtlebot3_absolute_move rviz.launch.py
  ros2 launch turtlebot3_absolute_move rviz.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_absolute_move')
    rviz_config = os.path.join(pkg_share, 'config', 'absolute_move.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
