prompt:

You are to create a fully working ROS 2 Humble TurtleBot3 project for the “Absolute Move” behavior, with production-quality documentation and support for BOTH:

1) Gazebo simulation mode
2) Real TurtleBot3 hardware mode over SSH/remote-PC workflow

Target robot/setup:
- TurtleBot3 Burger
- LDS2
- Ubuntu 22.x on the TurtleBot3 SBC (Raspberry Pi)
- Remote PC running Ubuntu + ROS 2 Humble
- Assume standard TurtleBot3 bringup / network setup from the official TurtleBot3 manual is already completed
- Use the official TurtleBot3 Absolute Move semantics: move to an ABSOLUTE (x, y, heading) in the odom frame, not relative motion

Primary goal:
Create a clean ROS 2 workspace/package set that lets me run the same logic in simulation and on real hardware with minimal changes. The project must include source code, launch files, configuration files, and a detailed README.

Functional requirements:
- Implement an Absolute Move node that:
  - subscribes to /odom
  - publishes /cmd_vel
  - accepts goal input as absolute x, y, and heading (degrees or radians, but document clearly)
  - computes position and orientation control using odometry
  - moves to the goal position first, then aligns heading
  - uses tolerances for position and heading
  - handles angle wrapping robustly
  - stops the robot safely on success or interruption
- Make the node reusable in both simulation and hardware with a parameter for:
  - use_sim_time
  - control gains
  - position tolerance
  - heading tolerance
  - max linear speed
  - max angular speed
  - odom frame / base frame names if needed
- Add a clean CLI interaction mode:
  - user can type goal x, goal y, goal heading
  - and/or a simple ROS 2 service or action interface for non-interactive control
- Include safety behavior:
  - emergency stop on Ctrl+C
  - zero cmd_vel published on exit
  - optional obstacle-awareness hook if easy, but do not block the core feature on it

Simulation requirements:
- Create a Gazebo-ready launch setup for TurtleBot3 Burger
- Include an easy command path to start an empty-world simulation
- Ensure the node works with simulated odom
- Use use_sim_time correctly
- Add an RViz2 launch or optional RViz instructions
- The simulation launch must be compatible with the TurtleBot3 Gazebo workflow
- Make sure the README clearly separates “simulation mode” from “hardware mode”

Hardware requirements:
- Create a hardware launch/run workflow that assumes:
  - TurtleBot3 bringup runs on the SBC/RPi
  - the control node runs on the Remote PC
- Provide exact shell commands for:
  - SSH to the RPi
  - setting TURTLEBOT3_MODEL=burger
  - launching turtlebot3_bringup on the robot
  - running the Absolute Move node on the remote PC
- The hardware workflow must be written so it matches the TurtleBot3 remote-PC setup pattern

Project structure requirements:
- Use a standard ROS 2 package layout
- Include source code, launch files, config YAML, and README
- Prefer a simple, maintainable architecture
- Organize the code so simulation and hardware differences are handled by launch/config, not by duplicating logic
- Include comments where needed, but do not over-comment trivial lines

Documentation requirements:
- Write a detailed README that includes:
  - project overview
  - architecture
  - prerequisites
  - build instructions
  - simulation run instructions
  - hardware run instructions
  - troubleshooting
  - explanation of coordinate frames and odom reset assumptions
  - explanation of how absolute move differs from relative move
  - examples of goal inputs
- Include a “known limitations” section
- Include a “tuning” section for gains and tolerances
- Include a short note that odom must be initialized consistently before using absolute positioning

Implementation guidance:
- Prefer ROS 2 Humble APIs and idioms
- You may implement in Python (rclpy) or C++ (rclcpp), but keep the code robust and clean
- Include launch files for:
  - simulation
  - hardware
  - optional RViz
- Include parameters in YAML for easy tuning
- Add package metadata and dependencies correctly
- Make sure all imports/build dependencies are complete
- If you use a custom action/service, include the interface definition and document it
- Add a minimal test or validation script if practical

Deliverables:
1) Full ROS 2 workspace/package source tree
2) All code files
3) Launch files
4) Config files
5) README.md
6) Step-by-step terminal commands for building and running
7) A brief explanation of how to validate the node in simulation before using hardware

Important constrai / @nts:
- Keep the project focused on TurtleBot3 Burger on ROS 2 Humble
- Do not include unrelated features
- Do not assume extra packages beyond standard TurtleBot3 + ROS 2 Humble dependencies unless you justify them
- Make the final result copy-paste runnable
- If any assumption is ambiguous, choose the most standard TurtleBot3 Humble approach and document that choice clearly

Output format:
- First give the final project tree
- Then provide each file in full
- Then provide the exact build/run commands for simulation and hardware
- Then provide troubleshooting notes
- Make sure everything is internally consistent
Task





response:

Deliverables — 27 files across 2 ROS 2 packages
Project Structure
robabsmov/
├── .gitignore
├── README.md
└── src/
    ├── turtlebot3_absolute_move_interfaces/   ← CMake package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── action/AbsoluteMove.action
    └── turtlebot3_absolute_move/              ← Python package
        ├── package.xml, setup.py, setup.cfg
        ├── config/
        │   ├── params_sim.yaml                ← sim gains + use_sim_time:true
        │   ├── params_hw.yaml                 ← conservative HW gains
        │   └── absolute_move.rviz
        ├── launch/
        │   ├── simulation.launch.py           ← Gazebo + node
        │   ├── hardware.launch.py             ← node only
        │   └── rviz.launch.py
        └── turtlebot3_absolute_move/
            ├── absolute_move_node.py          ← 3-phase controller + action server
            └── absolute_move_client.py        ← interactive CLI + single-shot
Key Design Points
Three-phase controller: rotate → translate → align heading
Action interface: goals in radians (odom frame), CLI client accepts degrees
Safety: zero cmd_vel on SIGINT, SIGTERM, cancel, and exit
Sim/HW parity: same node binary, different YAML configs and launch files
Heading-dependent speed reduction: linear_speed *= cos(heading_error) prevents arcing
Quick Start
bash
# Build:
cd robabsmov && colcon build --symlink-install && source install/setup.bash
# Simulation:
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_absolute_move simulation.launch.py
# Send goal (new terminal):
ros2 run turtlebot3_absolute_move absolute_move_client -- --goal 1.0 0.5 90
See the README.md for full hardware workflow, tuning guide, troubleshooting, and validation checklist.