# Mobile Robot Control and Navigation Assignment



## Introduction



This assignment is designed to provide hands-on experience with fundamental control and navigation techniques using ROS 2 and Webots. The tasks are structured progressively:



1. Set up the simulation environment and control the robot manually

2. Implement PID control for point-to-point navigation

3. Extend to trajectory following based on a sequence of waypoints

4. Develop a target-tracking system where one robot follows another using camera localization



---



## Prerequisites



Ensure the following dependencies are installed:



- **Programming Language:** Python or C++

- **Operating System:** Ubuntu 22.04 (recommended)

- **Middleware:** ROS2 Humble (recommended)

- **Simulator:** Webots 2025a, [webots_ros2_robomaster](https://github.com/MohismLab/webots_ros2_robomaster)



---



## Part 1 — Environment Setup



- Install Ubuntu (22.04 recommended), ROS 2 (Humble recommended), Webots, and all other necessary dependencies.

- Setup [webots_ros2_robomaster](https://github.com/MohismLab/webots_ros2_robomaster). Launch the provided ROS2 launch file:



```bash

ros2 launch webots_ros2_robomaster robot_launch.py

```



- Use the keyboard teleoperation node to manually control the mobile robot and verify basic movement functionality:



```bash

python3 keyboard_twist.py --topic /agent0/cmd_vel

```



---



## Part 2 — Point-to-Point Navigation with PID Control



- Implement a PID controller for the mobile robot.

- The robot should autonomously move from its starting position to a specified target point in the environment. Consider 2D planar navigation (x, y, θ).



---



## Part 3 — Trajectory Following



- Define a custom set of trajectory waypoints for the robot to follow.

- Using PID control, implement a navigation strategy that enables the robot to follow the given trajectory smoothly and accurately.



---



## Part 4 — Target Tracking



- Set up two mobile robots in the simulation: one as the leader and the other as the follower.

- Equip the follower with a monocular camera and integrate ArUco tag (or AprilTag) detection to estimate the relative pose of the leader.

- Manually control the leader robot using the keyboard. Meanwhile, implement a control strategy so that the follower robot tracks and follows the leader’s movements based on the relative pose information.



---



## Part 5 Bonus — LLM for function calling



Deploy [ROS MCP server](https://github.com/robotmcp/ros-mcp-server) to integrate natural language commands with robotic control functions.



Implement the following functionalities for the follower robot:



- Point-to-Point Navigation – enable the robot to move between specified waypoints using ROS MCP calls.



- Trajectory Following – configure the robot to follow a predefined trajectory based on inputs provided through ROS MCP.



- Target Tracking – implement continuous tracking of a moving target (the leader robot).



The leader robot will be manually controlled using natural language input (e.g., “move forward 2 meters,” “turn left 90 degrees”), which will be parsed and executed through ROS MCP.



The follower robot must maintain the required behavior by dynamically responding to the leader’s movements.



## Code Submission



Submit your code via a private GitHub repository. Ensure the following:



- Add **Qingbiao LI** (`qingbiao.qli@gmail.com`) and **Shiyuan Yang** (`shiyuanyang0628@hotmail.com`) as collaborators.

- Include a `README.md` file with clear instructions on how to set up, run the code, and replicate the results.

