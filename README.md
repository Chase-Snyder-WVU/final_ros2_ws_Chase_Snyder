# Autonomous Warehouse Inspection Bot — ROS 2 Final Project

**Author:** Chase Snyder  
**Course:** ROBE 313 (Robot Operating Systems) - West Virginia University  
**Platform:** ROS 2 Humble - Ubuntu 22.04 (WSL supported)

## Project Overview
This repository contains a ROS 2 Humble simulation of a **differential-drive mobile robot** that explores a warehouse environment, detects **ArUco markers**, navigates to requested markers, and performs an **image inspection** task. The system is designed around a simple mission-driven **state machine** and uses TF2 to reason about robot and marker poses.

> **Dependency Note**  
> This project relies on the WVU class workspace being present and built (packages/tools provided by the course).  
> Clone/build the class workspace first: https://github.com/WVU-ROBE313-class

---

## Main Features
- **C++ Odometry Node**
  - Integrates velocity to estimate robot pose
  - Publishes TF (`world -> base_link`)
  - Provides a reset service to re-zero pose
- **Python Mission Controller**
  - State machine for missions: initialization → search → navigate → inspection → return
  - Uses TF2 feedback and proportional control
  - Uses OpenCV (cv2) for ArUco and image inspection steps
- **Gazebo + RViz Visualization**
  - Launch files, URDF, and RViz config included

---

## Package Summary
- **custom_interfaces**
  - `ResetPosition.srv`
- **robot_simulator_cpp**
  - Odometry and TF publisher
  - Reset position service
- **robot_simulator_py**
  - Controller / mission state machine
  - Publishes `/cmd_vel`, `/robot_status`, `/robot_report`
- **robot_bringup**
  - Launch files, URDF, RViz config

---

## State Machine / Mission Flow
The controller listens for mission commands on `/missions` and responds via `/robot_status` and `/robot_report`.

### Phase 1 — Initialization
- Waits for system start and node readiness
- Publishes: `ready` to `/robot_status`

### Phase 2 — Search (`search_aruco`)
- Robot rotates/drives to scan the environment
- When a marker is detected, publish to `/robot_report`:
  - `aruco <ID> in position x: <X>, y: <Y>, z: <Z>`

### Phase 3 — Navigation (`move to <ID>`)
- Drives to the stored position for the requested marker ID
- Publishes upon arrival:
  - `arrived to <ID>` to `/robot_report`

### Phase 4 — Inspection (`image_analysis`)
- Perform image motion analysis
- Publishes:
  - `analyze image` to `/robot_status`
- Subscribes:
  - `image1` and `image2`
- Detect coordinates where movement is detected before publishing:
  - `movement at x: <val>, y: <val>` to `/robot_report`

### Phase 5 — Return (`to origin`)
- Drive robot back to the initial position
- Publishes on call:
  - `returning` to `/robot_status`
- Publishes on arrival:
  - `returned` to `/robot_report`

### Phase 6 (Bonus) — Feature Counting
- Count features using OpenCV feature matching
- Publishes:
  - `analyze image2` to `/robot_status`
- Subscribes:
  - `image1` and `image2`
- Detects features between them with SIFT/feature matching before publishing:
  - `<N> features detected` to `/robot_report`

---

## Requirements
- Ubuntu 22.04 + ROS 2 Humble
- `colcon`
- `rviz2`
- `gazebo` (as provided by course environment)
- Python 3 + `rclpy`
- C++17 + `rclcpp`
- OpenCV (`cv2`)

---

## Build Instructions

### 0) One-Time Setup: Install/Build the Class Workspace
This project depends on the WVU class packages (Gazebo launcher, evaluator, worlds, etc.).  
Follow the class instructions and ensure both workspaces below build successfully:

- `~/ROBE313_ROS_ws/Simulation_ws`
- `~/ROBE313_ROS_ws/Evaluator_SM_ws`

### 1) Clone and Build This Repository
```bash
git clone https://github.com/Chase-Snyder-WVU/final_ros2_ws_Chase_Snyder.git
cd final_ros2_ws_Chase_Snyder
```
# Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```
# Build and source overlay
```bash
colcon build
source install/setup.bash
```
### 2) Move the robot  model SDF from
- `~\final_ros2_ws_Chase_Snyder\src\SDF`
- To:
- `~\ROBE313_ROS_ws\Simulation_ws\models\`

## Running the Project

### Terminal 1
```bash
cd ~/ROBE313_ROS_ws/Simulation_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
ros2 launch gazebo_sim gazebo_ros.launch.py world:=rect_room_aruco.world sdf_path:='/home/chasedrew03/ROBE313_ROS_ws/Simulation_ws/models/square_bot/model.sdf' model_name:=square_bot
```
### Terminal 2
```bash
cd ~/ROBE313_ROS_ws/Evaluator_SM_ws
source install/setup.bash
ros2 run evaluator_package evaluator_node
```

### Terminal 3
```bash
cd ~/final_ros2_ws_Chase_Snyder
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
ros2 launch robot_bringup final_mission.launch.py
```



