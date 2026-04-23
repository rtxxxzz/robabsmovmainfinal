"""
Simulation Launch File — TurtleBot3 Absolute Move.

Launches:
  1. Gazebo (gzserver + gzclient) with a selectable world
  2. Robot State Publisher (URDF → TF)
  3. TurtleBot3 spawn into Gazebo
  4. SLAM Toolbox (online async) — enabled by default (slam:=true)
  5. Absolute Move action server node

SLAM is ON by default. The SLAM Toolbox publishes the map → odom TF
correction that keeps the odometry frame accurate across the entire run.
The absolute_move_node continues to read /odom — SLAM keeps that frame
calibrated so absolute goals remain consistent in the map frame.

Usage:
  export TURTLEBOT3_MODEL=burger

  # Empty world (default, SLAM on):
  ros2 launch turtlebot3_absolute_move simulation.launch.py

  # SLAM disabled (raw odometry only):
  ros2 launch turtlebot3_absolute_move simulation.launch.py slam:=false

  # TurtleBot3 World (hexagonal obstacles):
  ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_world

  # House environment:
  ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_house

  # DQN training stages:
  ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_dqn_stage1

  # Custom spawn position:
  ros2 launch turtlebot3_absolute_move simulation.launch.py world:=turtlebot3_house x_pose:=-2.0 y_pose:=-0.5

  # No Gazebo (Gazebo already running separately):
  ros2 launch turtlebot3_absolute_move simulation.launch.py launch_gazebo:=false

Available worlds (from turtlebot3_gazebo):
  empty_world, turtlebot3_world, turtlebot3_house,
  turtlebot3_dqn_stage1, turtlebot3_dqn_stage2,
  turtlebot3_dqn_stage3, turtlebot3_dqn_stage4
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


# Default spawn positions per world.
# turtlebot3_world and turtlebot3_house place the robot at (-2, -0.5)
# to avoid spawning inside a wall.
WORLD_SPAWN_DEFAULTS = {
    'empty_world':           ('0.0',  '0.0'),
    'turtlebot3_world':      ('-2.0', '-0.5'),
    'turtlebot3_house':      ('-2.0', '-0.5'),
    'turtlebot3_dqn_stage1': ('0.0',  '0.0'),
    'turtlebot3_dqn_stage2': ('0.0',  '0.0'),
    'turtlebot3_dqn_stage3': ('0.0',  '0.0'),
    'turtlebot3_dqn_stage4': ('0.0',  '0.0'),
}


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_absolute_move')
    params_file = os.path.join(pkg_share, 'config', 'params_sim.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'params_slam.yaml')

    # Ensure TURTLEBOT3_MODEL is set
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Locate required packages
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
        'use_sim_time', default_value='true',
        description='Use simulation clock'))

    ld.add_action(DeclareLaunchArgument(
        'launch_gazebo', default_value='true',
        description='Launch Gazebo simulation'))

    ld.add_action(DeclareLaunchArgument(
        'world', default_value='empty_world',
        description=(
            'Gazebo world name (without .world extension). '
            'Available: empty_world, turtlebot3_world, turtlebot3_house, '
            'turtlebot3_dqn_stage1, turtlebot3_dqn_stage2, '
            'turtlebot3_dqn_stage3, turtlebot3_dqn_stage4'
        )))

    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot spawn X position (meters). '
                    'Tip: use -2.0 for house/world to avoid walls.'))

    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Robot spawn Y position (meters). '
                    'Tip: use -0.5 for house/world to avoid walls.'))

    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='true',
        description=(
            'Run SLAM Toolbox (online async) alongside the absolute move node. '
            'When true, SLAM publishes the map->odom TF correction that keeps '
            'odometry accurate over long runs. Set to false for raw-odom mode.'
        )))

    # Set TURTLEBOT3_MODEL env var for child processes
    ld.add_action(SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value=tb3_model))

    # ------------------------------------------------------------------
    # Gazebo (gzserver + gzclient) — with world selection
    # ------------------------------------------------------------------
    if gazebo_tb3_pkg and pkg_gazebo_ros:
        # Build the full path to the .world file:
        #   <turtlebot3_gazebo>/worlds/<world_name>.world
        world_file = PathJoinSubstitution([
            gazebo_tb3_pkg, 'worlds',
            PythonExpression(["'", LaunchConfiguration('world'), ".world'"])
        ])

        # --- Gazebo Server (loads the world) ---
        gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )
        ld.add_action(gzserver_cmd)

        # --- Gazebo Client (GUI) ---
        gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )
        ld.add_action(gzclient_cmd)

        # --- Robot State Publisher (URDF → /robot_description + TF) ---
        tb3_launch_dir = os.path.join(gazebo_tb3_pkg, 'launch')

        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )
        ld.add_action(robot_state_publisher_cmd)

        # --- Spawn TurtleBot3 into Gazebo ---
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        )
        ld.add_action(spawn_turtlebot_cmd)
    else:
        ld.add_action(LogInfo(
            msg='[WARN] turtlebot3_gazebo or gazebo_ros not found. '
                'Skipping Gazebo launch. Install with: '
                'sudo apt install ros-humble-turtlebot3-gazebo '
                'ros-humble-gazebo-ros-pkgs'))

    # ------------------------------------------------------------------
    # SLAM Toolbox — Online Async (default: enabled)
    # ------------------------------------------------------------------
    # When slam:=true (default), SLAM Toolbox runs online_async mode.
    # It subscribes to /scan (TurtleBot3 LiDAR) and publishes:
    #   • /map                  (OccupancyGrid)
    #   • /map_metadata         (MapMetaData)
    #   • TF: map → odom        (continuously corrects odom drift)
    #
    # The absolute_move_node reads /odom.  With the map→odom TF published
    # by SLAM, the odom frame stays drift-corrected, so absolute goals
    # remain accurate over the full session.
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
            msg='[WARN] slam_toolbox not found — SLAM disabled. '
                'Install with: sudo apt install ros-humble-slam-toolbox'))

    # ------------------------------------------------------------------
    # Absolute Move Node
    # ------------------------------------------------------------------
    absolute_move_node = Node(
        package='turtlebot3_absolute_move',
        executable='absolute_move_node',
        name='absolute_move_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    ld.add_action(absolute_move_node)

    return ld
