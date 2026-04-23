"""
Hardware Launch File — TurtleBot3 Absolute Move.

Launches only the Absolute Move node on the Remote PC.
The TurtleBot3 bringup must already be running on the robot's SBC.

SLAM for hardware:
  By default SLAM is disabled here (slam:=false) because on a real
  TurtleBot3 the recommended workflow is to run SLAM Toolbox on the
  Remote PC *separately* (or on the SBC if it has enough compute).
  Enable it with slam:=true if you want SLAM Toolbox to start here.

  When slam:=true, this launch starts SLAM Toolbox online_async which
  subscribes to /scan coming from the real robot's LiDAR and publishes
  the map→odom TF correction.  The absolute_move_node benefits from
  this automatically — no other changes needed.

Usage (on Remote PC):
  # Odom-only (default):
  ros2 launch turtlebot3_absolute_move hardware.launch.py

  # With SLAM Toolbox (Remote PC handles SLAM):
  ros2 launch turtlebot3_absolute_move hardware.launch.py slam:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_absolute_move')
    params_file = os.path.join(pkg_share, 'config', 'params_hw.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'params_slam.yaml')

    try:
        slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    except Exception:
        slam_toolbox_pkg = None

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false for real hardware)'))

    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='false',
        description=(
            'Run SLAM Toolbox (online async) on the Remote PC. '
            'Set true if you want the Remote PC to handle SLAM. '
            'Requires /scan from the robot to be visible on this machine.'
        )))

    # ------------------------------------------------------------------
    # Absolute Move Node
    # ------------------------------------------------------------------
    ld.add_action(Node(
        package='turtlebot3_absolute_move',
        executable='absolute_move_node',
        name='absolute_move_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    ))

    # ------------------------------------------------------------------
    # SLAM Toolbox — Optional (slam:=true to enable)
    # ------------------------------------------------------------------
    if slam_toolbox_pkg:
        slam_toolbox_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    slam_toolbox_pkg, 'launch',
                    'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_params_file,
            }.items(),
            condition=IfCondition(LaunchConfiguration('slam')),
        )
        ld.add_action(slam_toolbox_cmd)
    else:
        ld.add_action(LogInfo(
            msg='[INFO] slam_toolbox not found (slam:=true will have no effect). '
                'Install with: sudo apt install ros-humble-slam-toolbox'))

    return ld
