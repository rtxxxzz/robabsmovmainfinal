"""
Hardware Launch File — TurtleBot3 Absolute Move.

Launches the full stack on the Remote PC for real hardware:
  1. SLAM Toolbox (online async) — enabled by default (slam:=true)
  2. Absolute Move action server node
  3. Pipeline Orchestrator (optional — explore + navigate)
  4. RViz2 for real-time map visualization

The TurtleBot3 bringup must already be running on the robot's SBC.

This launch mirrors the simulation launch (simulation.launch.py / pipeline.launch.py)
but without Gazebo and with hardware-specific parameters.

Usage (on Remote PC):
  export TURTLEBOT3_MODEL=burger

  # Full pipeline (explore + interactive goals):
  ros2 launch turtlebot3_absolute_move hardware.launch.py

  # Pipeline with goals from a file:
  ros2 launch turtlebot3_absolute_move hardware.launch.py goals_file:=goals.yaml

  # Pipeline — explore only (build and save the map):
  ros2 launch turtlebot3_absolute_move hardware.launch.py explore_only:=true

  # No pipeline (just absolute_move_node + SLAM):
  ros2 launch turtlebot3_absolute_move hardware.launch.py pipeline:=false

  # Without SLAM (raw odometry only):
  ros2 launch turtlebot3_absolute_move hardware.launch.py slam:=false

  # Without RViz:
  ros2 launch turtlebot3_absolute_move hardware.launch.py rviz:=false

  # Skip exploration, use saved map:
  ros2 launch turtlebot3_absolute_move hardware.launch.py skip_exploration:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_absolute_move')
    params_hw = os.path.join(pkg_share, 'config', 'params_hw.yaml')
    params_pipeline = os.path.join(pkg_share, 'config', 'params_pipeline.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'params_slam.yaml')
    slam_params_loc = os.path.join(pkg_share, 'config',
                                    'params_slam_localization.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'pipeline.rviz')

    try:
        slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    except Exception:
        slam_toolbox_pkg = None

    ld = LaunchDescription()

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false for real hardware)'))

    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='true',
        description=(
            'Run SLAM Toolbox (online async) on the Remote PC. '
            'Required for pipeline exploration and map-aware navigation. '
            'Set false for raw-odom-only mode.')))

    ld.add_action(DeclareLaunchArgument(
        'pipeline', default_value='true',
        description='Run the pipeline orchestrator (exploration + goals). '
                    'Set false to only launch absolute_move_node + SLAM.'))

    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for real-time visualization'))

    ld.add_action(DeclareLaunchArgument(
        'goals_file', default_value='',
        description='Path to YAML goals file (empty = interactive via goal_input)'))

    ld.add_action(DeclareLaunchArgument(
        'map_file', default_value='',
        description='Path to saved map YAML (empty = explore)'))

    ld.add_action(DeclareLaunchArgument(
        'explore_only', default_value='false',
        description='Only explore and save map, do not navigate'))

    ld.add_action(DeclareLaunchArgument(
        'skip_exploration', default_value='auto',
        description='Skip exploration: auto (detect existing map), '
                    'true (always skip), false (always explore)'))

    # ------------------------------------------------------------------
    # Scan Relay — Bridge Best Effort /scan → Reliable /scan_reliable
    # The TurtleBot3 LDS driver publishes /scan with Best Effort QoS.
    # SLAM Toolbox subscribes with Reliable QoS (hardcoded).
    # This relay bridges the QoS gap.
    # ------------------------------------------------------------------
    ld.add_action(Node(
        package='turtlebot3_absolute_move',
        executable='scan_relay',
        name='scan_relay',
        output='log',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('slam')),
    ))

    # ------------------------------------------------------------------
    # SLAM Toolbox — Online Async (default: enabled)
    # ------------------------------------------------------------------
    if slam_toolbox_pkg:
        # Use localization config when map_file is provided
        slam_config = PythonExpression([
            "'", slam_params_loc, "' if '",
            LaunchConfiguration('map_file'), "' else '",
            slam_params, "'"
        ])

        slam_toolbox_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    slam_toolbox_pkg, 'launch',
                    'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration('slam')),
        )
        ld.add_action(slam_toolbox_cmd)
    else:
        ld.add_action(LogInfo(
            msg='[WARN] slam_toolbox not found. '
                'Install: sudo apt install ros-humble-slam-toolbox'))

    # ------------------------------------------------------------------
    # Absolute Move Node
    # ------------------------------------------------------------------
    ld.add_action(Node(
        package='turtlebot3_absolute_move',
        executable='absolute_move_node',
        name='absolute_move_node',
        output='screen',
        parameters=[
            params_hw,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    ))

    # ------------------------------------------------------------------
    # RViz2 — Real-time map visualization
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    ld.add_action(rviz_node)

    # ------------------------------------------------------------------
    # Pipeline Orchestrator Node
    # ------------------------------------------------------------------
    pipeline_node = Node(
        package='turtlebot3_absolute_move',
        executable='pipeline',
        name='pipeline_node',
        output='screen',
        parameters=[
            params_pipeline,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=[
            PythonExpression([
                "'--goals' if '", LaunchConfiguration('goals_file'),
                "' else ''"
            ]),
            PythonExpression([
                "'", LaunchConfiguration('goals_file'),
                "' if '", LaunchConfiguration('goals_file'), "' else ''"
            ]),
            PythonExpression([
                "'--map' if '", LaunchConfiguration('map_file'),
                "' else ''"
            ]),
            PythonExpression([
                "'", LaunchConfiguration('map_file'),
                "' if '", LaunchConfiguration('map_file'), "' else ''"
            ]),
            PythonExpression([
                "'--explore-only' if '",
                LaunchConfiguration('explore_only'), "' == 'true' else ''"
            ]),
            '--skip-exploration',
            LaunchConfiguration('skip_exploration'),
        ],
        condition=IfCondition(LaunchConfiguration('pipeline')),
    )
    ld.add_action(pipeline_node)

    return ld
