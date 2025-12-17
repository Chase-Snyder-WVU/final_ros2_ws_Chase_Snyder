# Square-Path Mobile Robot Simulation (ROS 2 Humble)
**Author:** Chase Snyder
**Course:** ROBE 313 - West Virginia University
**Assignment:** Homework 3 - ROS 2 Multi-Package System

## Project Overview
This project implements a ROS 2 Humble simulation for a mobile robot that drives in a **2m x 2m square** in an Ubuntu 22.04 WSL environment.
> **Note:** 
> This implementation uses **only linear x and y velocity** with no turning or angular models.

## Main Features
- C++ odometry node that integrates velocity to estimate robot pose and publishes robot pose with `/tf` broadcast.
- Python controller node using TF2 feeback and a state machine to drive the square path by publishing velocity commands.
- Python teleop keyboard controller using ROS 2 twist keyboard.
- Custom `.srv` service to reset robot pose.
- URDF robot model and visualization in RViz.

## Workspace Structure
ros2_ws_Chase Snyder/
    src/
        custom_interfaces/
        robot_simulator_cpp/
        robot_simulator_py/
        robot_bringup/

## Package Summary
- **custom_interfaces** holds `ResetPosition.srv`
- **robot_simulator_cpp** publishes `/tf`, integrates odometry, and provides reset service
- **robot_simulator_py** TF2 controller state machine that publishes `/cmd_vel` for square path
- **robot_bringup** holds the launch files, URDF, and RViz config

## Requirements
- ROS 2 Humble  (Ubuntu 22.04 / WSL)
- `colcon`
- `ament`
- `rviz2`
- `Python3`
- `rclpy`
- `C++17`
- `rclcpp`

## Build Instructions

### 1. Clone Workspace
Open a terminal and input the following command.
```bash
git clone https://github.com/Chase-Snyder-WVU/ros2_ws_Chase_Snyder.git
```
### 2. Source ROS 2
Source Ros 2 on your system.
```bash
source /opt/ros/humble/setup.bash
```
### 3. Build
Source the project folder directory and build it.
```bash
cd ros2_ws_Chase_Snyder2
colcon build
source install/setup.bash
```
### 4. Run Simulation 
From ros2_ws_Chase_Snyder2 run the simulation.
```bash
ros2 launch robot_bringup robot_simulation.launch.py
```
>**WSL Note:**
>WSL fails to render the graphics with the base settings, the following commands will force software rendering though RViz remains unstable. Run them before sending the launch command to the terminal if you have this issue.
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```
>**RViz Note:** 
> When using RViz, the Topic and Parameter options did not function properly. You may have to directly reference the URDF file path (robot_bringup->urdf) under robot model->robot description in RViz.

## Keyboard Teleop
After building, sourcing, and making sure the package works, proceed with the following steps.

### 1. Launch Teleop Simulation
RViz will appear with a idle robot.
```bash
 ros2 launch robot_bringup teleop.launch.py
```
### 2. Run the Keyboard
In another terminal, initiate the Ros 2 twist keyboard.
```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### 3. Begin Controlling the Robot
With the keyboard terminal selected and RViz in view. The following commands control the robot's position.
**The Key Strokes**
`shift + i = +x motion`
`shift + < = -x motion`
`shift + j = +y motion`
`shift + l = -y motion`
`w/x = increase/decrease linear speed`
The other commands can also be used to manipulate the robot, but these are the primary strokes. 
>**Note:**
>In cases where the robot starts off the screen or has moved off the enviroment use the **Reset Robot Pose Service** command in a terminal to reset it and get the robot back into view on the frame.

>**Stopping Note:**
>Be careful when stopping, as the robot's next movement scales with the amount of time the robot has spent not moving. This is denoted as the "jumping" bug.
## Useful Commands

### Run Odometry Node
```bash
ros2 run robot_simulator_cpp odometry_node
```
### Run Only Controller
```bash
ros2 run robot_simulator_py controller_node
```
### Inspect Graph
In a seperate terminal while the project is running. Display the node communication as an rqt_graph.
```bash
rqt_graph
```
### View Transforms
In a seperate terminal while the odometry node, controller node, or the project is running.
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
### Reset Robot Pose Service
In a seperate terminal while the odometry node, controller node, or the project is running.
```bash
ros2 service call /ResetPosition custom_interfaces/srv/ResetPosition
```
## Known Bugs

### "No Frames" Intialization
This bug uncommonly occurs when the project is started, often displaying a failed transform message when it occurs. This is likely because on start-up the odometry node and controller node fail to see eachother's initialization commands resulting in an idle white robot in RViz,

### Teleop "Jumping"
This bug influences teleop control specifically, while not issuing any movement the robot appears idle. The moment a
command is inputted though the robot "jumps" a signifcant distance scaling with the amount of time since the last
input. It is unkown why this occurs.








# final_ros2_ws_Chase_Snyder
