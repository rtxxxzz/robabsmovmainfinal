"""
Pipeline Launch File — TurtleBot3 Autonomous Navigation.

Single-command launch for the full pipeline: Gazebo/HW + SLAM + absolute_move_node + pipeline_orchestrator.

Usage:
  export TURTLEBOT3_MODEL=burger

  # Simulation — full pipeline (explore + navigate):
  ros2 launch turtlebot3_absolute_move pipeline.launch.py

  # Simulation with goals file:
  ros2 launch turtlebot3_absolute_move pipeline.launch.py goals_file:=goals_example.yaml

  # Simulation with a specific world:
  ros2 launch turtlebot3_absolute_move pipeline.launch.py world:=turtlebot3_world x_pose:=-2.0 y_pose:=-0.5

  # Hardware mode (bringup must already run on the SBC):
  ros2 launch turtlebot3_absolute_move pipeline.launch.py mode:=hardware

  # Skip exploration, use saved map:
  ros2 launch turtlebot3_absolute_move pipeline.launch.py map_file:=/home/user/my_map.yaml

  # Explore only:
  ros2 launch turtlebot3_absolute_move pipeline.launch.py explore_only:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_absolute_move')
    params_sim = os.path.join(pkg_share, 'config', 'params_sim.yaml')
    params_hw = os.path.join(pkg_share, 'config', 'params_hw.yaml')
    params_pipeline = os.path.join(pkg_share, 'config', 'params_pipeline.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'params_slam.yaml')

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    try:
        gazebo_tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    except Exception:
        gazebo_tb3_pkg = None

    try:
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    except Exception:
        pkg_gazebo_ros = None

    try:
        slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    except Exception:
        slam_toolbox_pkg = None

    ld = LaunchDescription()

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        'mode', default_value='simulation',
        description='Pipeline mode: simulation or hardware'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'))

    ld.add_action(DeclareLaunchArgument(
        'world', default_value='empty_world',
        description='Gazebo world name'))

    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot spawn X'))

    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Robot spawn Y'))

    ld.add_action(DeclareLaunchArgument(
        'goals_file', default_value='',
        description='Path to YAML goals file (empty = interactive)'))

    ld.add_action(DeclareLaunchArgument(
        'map_file', default_value='',
        description='Path to saved map YAML (empty = explore)'))

    ld.add_action(DeclareLaunchArgument(
        'explore_only', default_value='false',
        description='Only explore and save map, do not navigate'))

    ld.add_action(SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value=tb3_model))

    # ------------------------------------------------------------------
    # Determine if simulation mode
    # ------------------------------------------------------------------
    is_sim = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'simulation'"])

    # ------------------------------------------------------------------
    # Gazebo (simulation only)
    # ------------------------------------------------------------------
    if gazebo_tb3_pkg and pkg_gazebo_ros:
        world_file = PathJoinSubstitution([
            gazebo_tb3_pkg, 'worlds',
            PythonExpression(["'", LaunchConfiguration('world'), ".world'"])
        ])

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items(),
            condition=IfCondition(is_sim))
        ld.add_action(gzserver)

        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(is_sim))
        ld.add_action(gzclient)

        tb3_launch_dir = os.path.join(gazebo_tb3_pkg, 'launch')

        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(is_sim))
        ld.add_action(rsp)

        spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
            }.items(),
            condition=IfCondition(is_sim))
        ld.add_action(spawn)

    # ------------------------------------------------------------------
    # SLAM Toolbox (always on — needed for exploration)
    # ------------------------------------------------------------------
    if slam_toolbox_pkg:
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_pkg, 'launch',
                             'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_params,
            }.items())
        ld.add_action(slam)
    else:
        ld.add_action(LogInfo(
            msg='[WARN] slam_toolbox not found. Pipeline requires SLAM. '
                'Install: sudo apt install ros-humble-slam-toolbox'))

    # ------------------------------------------------------------------
    # Absolute Move Node (existing, unchanged)
    # ------------------------------------------------------------------
    # Choose params based on mode
    abs_move_sim = Node(
        package='turtlebot3_absolute_move',
        executable='absolute_move_node',
        name='absolute_move_node',
        output='screen',
        parameters=[
            params_sim,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(is_sim))
    ld.add_action(abs_move_sim)

    is_hw = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'hardware'"])
    abs_move_hw = Node(
        package='turtlebot3_absolute_move',
        executable='absolute_move_node',
        name='absolute_move_node',
        output='screen',
        parameters=[
            params_hw,
            {'use_sim_time': False},
        ],
        condition=IfCondition(is_hw))
    ld.add_action(abs_move_hw)

    # ------------------------------------------------------------------
    # RViz2 — Real-time map visualization
    # ------------------------------------------------------------------
    rviz_config = os.path.join(pkg_share, 'config', 'pipeline.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    ld.add_action(rviz_node)

    # ------------------------------------------------------------------
    # Pipeline Orchestrator Node
    # ------------------------------------------------------------------
    # Build the arguments list for the pipeline node
    pipeline_args = []

    pipeline_node = Node(
        package='turtlebot3_absolute_move',
        executable='pipeline',
        name='pipeline_node',
        output='screen',
        parameters=[
            params_pipeline,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # Pass goals_file, map_file, explore_only as CLI args
        arguments=[
            PythonExpression([
                "'--goals ' + '", LaunchConfiguration('goals_file'),
                "' if '", LaunchConfiguration('goals_file'), "' else ''"
            ]),
            PythonExpression([
                "'--map ' + '", LaunchConfiguration('map_file'),
                "' if '", LaunchConfiguration('map_file'), "' else ''"
            ]),
            PythonExpression([
                "'--explore-only' if '",
                LaunchConfiguration('explore_only'), "' == 'true' else ''"
            ]),
        ],
    )
    ld.add_action(pipeline_node)

    return ld

