<div align="center">

# Pharmacy Assistant Robot

### Autonomous Mobile Manipulator | ROS 2 · MoveIt 2 · Nav2 · Gazebo Sim

**A four-wheel differential-drive robot equipped with a custom 5-DOF manipulator, RGB-D perception, and an autonomous colour-based pick-and-place workflow.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![MoveIt 2](https://img.shields.io/badge/Manipulation-MoveIt%202-3776AB)](https://moveit.picknik.ai/)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo%20Fortress-646CFF)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)

</div>

> **Development status:** Simulation-focused research and development. Autonomous pickup, navigation, and colour-specific placement are implemented; repeatable physically verified placement across all three colours is an ongoing validation goal. Passing unit tests alone does not certify successful placement.

<!-- Add an actual robot screenshot or demonstration GIF here when available:
![Pharmacy Assistant Robot in Gazebo](docs/images/robot_demo.gif)
-->

## Overview

The **Pharmacy Assistant Robot**, also referred to as the **Autonomous Mobile Manipulator System (AMMS)**, combines autonomous mobile navigation with robotic manipulation. The robot navigates to a pickup station, detects a coloured object, uses its 5-DOF arm and gripper to pick it up, transports it to a central placing station, and attempts to deposit it onto the matching elevated platform.

The current design replaces an earlier block-style arm with a more realistic articulated robot arm. The updated software integrates **MoveIt 2 motion planning**, **RGB-D colour detection**, and a dedicated **purple placing station with red, green, and blue targets**.

### Features

- **Custom 5-DOF arm:** base yaw, shoulder pitch, elbow pitch, wrist pitch, and wrist roll, plus a parallel-jaw gripper.
- **MoveIt 2 manipulation:** planning scene, inverse kinematics, collision-aware trajectory planning, and joint trajectory execution.
- **Autonomous navigation:** Nav2, AMCL localization, global/local costmaps, and precise station approach logic.
- **Perception:** RGB-D camera-based colour segmentation and 3D object localization for picking.
- **Sensor fusion:** wheel odometry and IMU measurements fused by `robot_localization` EKF.
- **Gazebo Sim:** robot description, sensors, actuators, arena, pickup stations, and colour-specific placement platforms.
- **Mission control:** ROS 2 services and an interactive command-line interface.
- **Placement validation:** Gazebo-based observation and support-checking logic under development and testing.

## System architecture

```text
                        PHARMACY ASSISTANT ROBOT
                                   |
         +-------------------------+-------------------------+
         |                         |                         |
    RGB-D Camera               2D LiDAR               Wheel Encoders
         |                         |                       + IMU
  Colour Detection           Nav2 / AMCL                    |
  3D Object Position      Path Planning / Docking      EKF Odometry
         |                         |                         |
         +-------------------------+-------------------------+
                                   |
                            AMMS Task Manager
                        (mission state machine)
                                   |
                           MoveIt 2 Commander
                     IK / planning / collision checks
                                   |
                     5-DOF Arm + Parallel Gripper
                                   |
                        Pick -> Carry -> Place
                                   |
                        Gazebo Pose Observation
                      Placement Support Verification
```

### Main technologies

| Layer | Technologies |
|---|---|
| Operating system | Ubuntu 22.04 |
| Robotics middleware | ROS 2 Humble |
| Simulation | Gazebo Sim / Ignition Gazebo Fortress |
| Arm manipulation | MoveIt 2, OMPL, KDL, analytical IK helpers |
| Navigation | Nav2, AMCL, DWB controller |
| Perception | RGB-D camera, OpenCV colour segmentation |
| State estimation | `robot_localization` EKF, IMU, wheel odometry |
| Controllers | `ros2_control`, differential-drive and joint-trajectory controllers |
| Languages | Python, C++, Arduino firmware |

## Robot design

**Mobile base:** four-wheel differential-drive platform with LiDAR, wheel encoders, IMU, and an RGB-D camera.

**Manipulator:** five revolute degrees of freedom with a parallel-jaw gripper. Its geometry and joints are defined in URDF/Xacro, with separate MoveIt 2 configuration for planning and execution. Arm motion is sent to `/arm_position_controller/follow_joint_trajectory`.

**Manipulation workflow:** the camera provides an object estimate; the task manager selects the target; the arm plans an approach and grasp; the robot carries the object; then the placer computes an approach, descent, release, and retreat. Collision checking and feedback are used to reduce unsafe or inconsistent motions.

## Simulation environment

The `robotics_arena` world is defined in `src/articubot_one_with_arm/worlds/og.world`. It includes three pickup stations and a central **purple placement station** with three elevated, colour-coded platforms.

| Mission | Pickup object | Destination |
|---|---|---|
| A | `cube_red` | `placing_platform_red` |
| B | `cube_green` | `placing_platform_green` |
| C | `cube_blue` | `placing_platform_blue` |
| D | Place-only station demonstration | Placement station |

### Nominal placement geometry

| Platform | World XY centre (m) | Top Z (m) |
|---|---|---|
| Red | `(-0.080, -2.640)` | `0.115` |
| Green | `(0.000, -2.640)` | `0.115` |
| Blue | `(0.080, -2.640)` | `0.115` |

Each coloured platform is nominally **60 × 60 mm**; the target cubes are **25 mm** wide. A correctly supported upright cube has a nominal centre height of **0.1275 m**. These are world-model values, not substitutes for observed poses during a mission.

## Autonomous mission sequence

```text
Start / select mission
        |
Navigate to pickup station (Nav2)
        |
Detect target object (RGB-D colour detection)
        |
Compute grasp and plan arm motion (MoveIt 2)
        |
Approach -> close gripper -> lift
        |
Move arm to carry posture
        |
Navigate to purple placement station
        |
Dock / align at placing station
        |
Identify matching coloured platform
        |
Plan placement -> descend -> release
        |
Verify support and stability -> retreat
        |
Report verified result / failure / unverified state
```

**Important:** mission completion must be based on observed physical support, not simply a successful gripper-open command. The placement verification and docking logic are areas of active testing.

## Repository structure

```text
src/
├── articubot_one_with_arm/     # Robot Xacro/URDF, Gazebo world, controllers and bridge
├── k12_pick_place/             # Mission state machine, perception, docking and manipulation
├── k12_moveit_config/          # SRDF, OMPL, kinematics and MoveIt controllers
├── robot_arm_description/      # Arm meshes and geometry/kinematics tools
└── ROSArduinoBridge_BTS7960/   # Embedded firmware for the physical mobile base
```

### Key software components

| Component | Responsibility |
|---|---|
| `amms_task_manager.py` | Orchestrates navigation, perception, picking, placing, and mission states |
| `moveit_pick_place_commander.py` | Executes planned arm/gripper actions and handles placement logic |
| `object_detector.py` | Performs colour-based object detection and position estimation |
| `gazebo_observer.py` | Observes simulator entity poses for physical placement checks |
| `station_docking.py` | Supports final station approach and alignment |
| `ik.py` | Analytical inverse-kinematics utilities |
| `readiness_gate.py` | Checks system readiness before mission execution |
| `amms_command.py` | Provides interactive mission selection |

## Getting started

### Prerequisites

- Ubuntu **22.04** with **ROS 2 Humble**.
- Gazebo Sim / Ignition Gazebo **Fortress**, plus the appropriate ROS–Gazebo bridge and control packages.
- **MoveIt 2**, **Nav2**, `ros2_control`, `robot_localization`, `colcon`, and the Python dependencies used by the workspace.

> This project includes workspace-specific ROS/Gazebo packages. Review package manifests and launch files for the exact dependencies; it is not yet packaged as a one-command universal installer.

### 1. Clone and build

```bash
git clone https://github.com/naban559/pharmacy_assistant_robot.git
cd pharmacy_assistant_robot

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If dependency resolution is needed, install/configure ROS dependencies for your Ubuntu/ROS distribution before rebuilding.

### 2. Launch the autonomous simulation

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77

ros2 launch k12_pick_place amms_moveit_nav_pick_place.launch.py
```

### 3. Run the operator interface

In a **second terminal**, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77

ros2 run k12_pick_place amms_command
```

Choose **A**, **B**, **C**, or **D** to run the corresponding mission.

Alternatively, trigger a mission directly:

```bash
ros2 service call /amms/goto_a std_srvs/srv/Trigger "{}"
ros2 service call /amms/goto_b std_srvs/srv/Trigger "{}"
ros2 service call /amms/goto_c std_srvs/srv/Trigger "{}"
```

> **Domain ID:** `77` is the documented local default; use a different ID consistently across terminals when running isolated tests or sharing a network with other ROS systems.

## Configuration

| Configuration file | Purpose |
|---|---|
| `src/k12_pick_place/config/amms_waypoints.yaml` | Pickup and placing waypoints, docking and arrival tolerances |
| `src/k12_pick_place/config/nav2_amms.yaml` | Navigation controller, planners, localization and costmaps |
| `src/k12_pick_place/config/moveit_nav_pick_place.yaml` | Arm planning, grasp/place parameters, collision and release settings |
| `src/k12_pick_place/config/ekf_odom.yaml` | Wheel-odometry and IMU fusion |
| `src/articubot_one_with_arm/config/my_controllers.yaml` | Base, arm and gripper controllers |
| `src/articubot_one_with_arm/config/gz_bridge.yaml` | ROS–Gazebo message bridging |

Adjust station and placement parameters only after checking the active world geometry, robot frames, and controller feedback.

## Testing and validation

The repository includes Python tests and mission-validation tooling. From a properly configured workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/k12_pick_place/test/ -v
```

For a complete simulation run, inspect the validation runner in `validation/run_mission_test.py` and ensure required simulator/ROS services are available. **Unit tests and successful launch do not by themselves prove a cube landed on the correct platform.**

The placement validation target is to measure real simulator state throughout pickup, docking, pre-release alignment, release, settling and retreat. Success should require matching colour/platform identity, footprint containment, acceptable cube/platform height difference, and sustained stability after release.

### Areas under active refinement

- Reliable, repeatable platform-centre localization for all colours.
- Closed-loop alignment of the **held object**, rather than the arm TCP alone.
- Consistent final docking without repeated heading correction.
- Pre-release and post-retreat physical verification.
- End-to-end success rates across varied starting poses.

## Development history

The current `try_18` development iteration introduces a more realistic **5-DOF articulated arm**, MoveIt 2 integration, colour-based perception, and an expanded placement workflow, building upon earlier mobile robot and manipulation prototypes.

## Maintainer

**Sayyid Muhammad Nabhan**  
GitHub: [@naban559](https://github.com/naban559)

---

<div align="center">

**ROS 2 · Autonomous Navigation · Robot Manipulation · Computer Vision**

*From perception and motion planning to coordinated autonomous pick-and-place.*

</div>
