# K12 AMMS — Autonomous Mobile Manipulator System
## ROS 2 Humble + Ignition Gazebo + MoveIt 2 Pick-and-Place Pipeline

> **Maintainer**: nabhan (naban559@gmail.com)  
> **Robot Domain**: `ROS_DOMAIN_ID=77`  
> **Simulation**: Ignition Gazebo (Gazebo Sim Fortress / Sim 6)  
> **ROS Distribution**: ROS 2 Humble Hawksbill  
> **Planning & Navigation**: MoveIt 2 (OMPL / KDL) + Nav2 (AMCL / DWB)

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Workspace Architecture](#2-workspace-architecture)
3. [Package Breakdown](#3-package-breakdown)
   - [3.1 k12_pick_place](#31-k12_pick_place)
   - [3.2 articubot_one_with_arm](#32-articubot_one_with_arm)
   - [3.3 k12_moveit_config](#33-k12_moveit_config)
   - [3.4 robot_arm_description](#34-robot_arm_description)
   - [3.5 ROSArduinoBridge_BTS7960](#35-rosarduinobridge_bts7960)
4. [Simulation Environment (og.world)](#4-simulation-environment-ogworld)
5. [Autonomous Missions & Operator CLI](#5-autonomous-missions--operator-cli)
6. [Manipulation, MoveIt 2 & Physical Verification](#6-manipulation-moveit-2--physical-verification)
7. [Navigation & Precise Docking Architecture](#7-navigation--precise-docking-architecture)
8. [System Interconnection & Data Flow](#8-system-interconnection--data-flow)
9. [How to Build and Run](#9-how-to-build-and-run)
10. [Tuning & Configuration Reference](#10-tuning--configuration-reference)

---

## 1. System Overview

The **K12 AMMS** (Autonomous Mobile Manipulator System) is an integrated mobile manipulation platform designed for automated sorting and pick-and-place operations in robotics arenas and pharmacy environments.

```
       [ 2D LiDAR ]           [ RGB-D Depth Camera ]
            │                           │
  ┌─────────┴───────────────────────────┴─────────┐
  │         5-DOF Articulated Robotic Arm         │
  │        with Parallel Jaw Gripper (MoveIt 2)   │
  ├───────────────────────────────────────────────┤
  │                                               │
  │       4-Wheeled Differential Drive Base       │
  │          Nav2 Stack + EKF Sensor Fusion       │
  └───────────────────────┬───────────────────────┘
                          │
              [ Wheel Encoders + IMU ]
```

### Key Capabilities
- **Multi-Station Autonomous Pick**: Navigates autonomously to one of three pickup stations across the arena (Red, Green, Blue) using Nav2 costmaps and AMCL localization.
- **Perception & Grasping**: Detects colored target blocks using an RGB-D depth camera, computes analytical or MoveIt collision-free grasp trajectories, grasps the block with a parallel gripper, and transitions to a compact carry posture.
- **Two-Stage Precise Docking**: Approaches the central placing station via a staged staging waypoint `(0.00, -1.90)`, aligns its yaw heading perpendicular to the table, and rolls forward into the docking position `(0.00, -2.25)`.
- **Targeted Surface Placement**: Deposits the picked block onto its designated elevated platform (Red, Green, or Blue) mounted on a purple placement table.
- **Strict Ground-Truth Physical Verification**: Enforces physics-based settlement checks via Gazebo Sim pose streaming before releasing the object and before retracting the arm, preventing unguided drops or premature departures.
- **Interactive Operator Control**: Exposes ROS 2 Trigger services (`/amms/goto_a`, `/amms/goto_b`, `/amms/goto_c`, `/amms/goto_d`) and an interactive CLI (`ros2 run k12_pick_place amms_command`).

---

## 2. Workspace Architecture

All software packages reside directly in `/home/nabhan/try_18/src/`:

```
/home/nabhan/try_18/
├── src/
│   ├── articubot_one_with_arm/     → Robot URDF/Xacro, Gazebo plugins, og.world
│   ├── k12_pick_place/             → Mission FSM, MoveIt commander, perception, docking
│   ├── k12_moveit_config/          → MoveIt 2 SRDF, OMPL kinematics & collision setup
│   ├── robot_arm_description/      → 5-DOF arm kinematics, STL meshes & offline tools
│   └── ROSArduinoBridge_BTS7960/   → Physical robot Arduino firmware (BTS7960 + encoders)
├── build/                          → (Excluded from git) Colcon build workspace
├── install/                        → (Excluded from git) Colcon install overlay
├── log/                            → (Excluded from git) Build and runtime logs
├── .gitignore                      → Git exclusions for build, logs, and archives
└── README.md                       → This comprehensive documentation
```

### Environment Setup
Before launching or building, source both ROS 2 Humble and the workspace overlay:
```bash
source /opt/ros/humble/setup.bash
source /home/nabhan/try_18/install/setup.bash
export ROS_DOMAIN_ID=77
```

---

## 3. Package Breakdown

### 3.1 `k12_pick_place`

**Type**: Python package (`ament_python`)  
**Maintainer**: vijay (`vijaykumargorli2003@gmail.com`) / nabhan  
**Role**: Master orchestration, perception pipeline, MoveIt 2 manipulation interface, station docking, and physical verification.

#### Key Modules & Scripts
| Module | Executable Name | Role |
|---|---|---|
| `amms_task_manager.py` | `amms_task_manager` | Master Finite State Machine (FSM) coordinating navigation, docking, perception, and manipulation. |
| `moveit_pick_place_commander.py` | `moveit_pick_place_commander` | MoveIt 2 motion planning client for arm trajectories, collision scene management, and gripper control. |
| `pick_place_commander.py` | `pick_place_commander` | Analytical IK fallback manipulation commander. |
| `gazebo_observer.py` | *(Library)* | Gazebo Sim ground-truth pose listener; enforces pre-release lateral alignment and post-release support verification. |
| `station_docking.py` | *(Library)* | Specialized docking controller for two-stage approach and camera-assisted purple table alignment. |
| `object_detector.py` | `object_detector` | OpenCV RGB-D color segmentation and 3D centroid estimator. |
| `ik.py` | *(Library)* | Pure Python analytical inverse kinematics solver for the 5-DOF arm. |
| `readiness_gate.py` | `readiness_gate` | Startup synchronization barrier checking topics, actions, transforms, and Nav2 lifecycle nodes. |
| `cloud_to_scan.py` | `cloud_to_scan` | Converts 3D PointCloud2 from depth camera to 2D LaserScan for Nav2 obstacle avoidance with self-filter. |
| `waypoint_manager.py` | `waypoint_manager` | Visualizes navigation waypoints and station markers in RViz2. |
| `amms_command.py` | `amms_command` | Interactive CLI menu for commanding Missions A, B, C, or D. |

#### Configuration Files (`config/`)
- `amms_waypoints.yaml`: Map coordinates for all pickup stations, docking waypoints, arrival tolerances, and backup distances.
- `moveit_nav_pick_place.yaml`: MoveIt planning parameters, table collision boxes, platform offsets, and grasp/release tolerances.
- `nav2_amms.yaml`: Nav2 stack configuration (DWB local planner, AMCL particle filter, global & local costmaps).
- `ekf_odom.yaml`: Extended Kalman Filter parameters fusing wheel odometry (`/diff_cont/odom`) and IMU (`/imu`).
- `pointcloud_to_laserscan.yaml`: Height filtering and angular bounds for synthetic laser scan generation.

---

### 3.2 `articubot_one_with_arm`

**Type**: CMake package (`ament_cmake`), registered in ROS as `articubot_one`  
**Role**: Complete robot description (URDF/Xacro), Gazebo simulation world, ros2_control configuration, and ROS-Gazebo bridge.

#### Key Subdirectories & Files
- `description/four_wheel.xacro`: Main robot URDF entry point. Combines differential drive mobile chassis, sensors, and arm mount.
- `description/arm.xacro`: 5-DOF manipulator links, joints, inertias, and limits.
- `description/gripper.xacro`: Parallel jaw gripper with two prismatic fingers (`finger_travel = 0.0394875 m`).
- `description/arm_ros2_control.xacro`: `ign_ros2_control` hardware interface configuration.
- `worlds/og.world`: Ignition Gazebo world containing the robotics arena, 3 pickup tables, cubes, and placing station.
- `config/my_controllers.yaml`: `ros2_control` controller definitions:
  - `diff_cont` (`diff_drive_controller/DiffDriveController`)
  - `joint_broad` (`joint_state_broadcaster/JointStateBroadcaster`)
  - `arm_position_controller` (`joint_trajectory_controller/JointTrajectoryController`)
  - `gripper_position_controller` (`forward_command_controller/ForwardCommandController`)
- `config/gz_bridge.yaml`: Parameter mapping between Ignition Gazebo transport and ROS 2 topics.

---

### 3.3 `k12_moveit_config`

**Type**: CMake package (`ament_cmake`)  
**Role**: MoveIt 2 configuration package generated and customized for the K12 arm.

- `config/k12.srdf`: Semantic Robot Description Format. Defines the `arm` planning group (`arm_base_yaw_joint` through `arm_wrist_roll_joint`), `gripper` end-effector group, and disables self-collision checking for adjacent links.
- `config/ompl_planning.yaml`: OMPL motion planner configurations (RRTConnect, RRT*, PRM*, etc.).
- `config/kinematics.yaml`: Kinematic solver plugin configuration (`kdl_kinematics_plugin/KDLKinematicsPlugin` with `position_only_ik: true`).
- `config/joint_limits.yaml`: Velocity and acceleration limits for trajectory generation.
- `config/controllers.yaml`: Connects MoveIt 2 to the `/arm_position_controller/follow_joint_trajectory` action server.

---

### 3.4 `robot_arm_description`

**Type**: CMake package (`ament_cmake`)  
**Role**: Standalone description and kinematic verification tools for the 5-DOF robotic arm.
- Contains high-resolution STL meshes (`base.stl`, `shoulder.stl`, `upper_arm.stl`, `fore_arm.stl`, `wrist.stl`).
- Provides offline inspection tools:
  - `tools/analyze_meshes.py`: Computes center of mass, bounding boxes, and inertias directly from STL geometry.
  - `tools/verify_kinematics.py`: Validates URDF joint transformations and workspace limits without launching ROS.

---

### 3.5 `ROSArduinoBridge_BTS7960`

**Type**: Embedded Arduino Firmware  
**Role**: Real-robot hardware control for the differential drive base.
- Implements closed-loop PID velocity control using dual BTS7960 43A High-Power H-bridge motor drivers.
- Reads quadrature optical encoders for real-time wheel odometry.
- Communicates with the onboard ROS computer via serial commands (`commands.h`, `diff_controller.h`).

---

## 4. Simulation Environment (`og.world`)

The simulation takes place inside `robotics_arena` ($9.0\text{ m} \times 6.0\text{ m}$ bounded arena):

```
 Y (meters)
 ▲
 │   [-3.0, 2.75]              [0.0, 2.75]              [3.0, 2.75]
 │   ┌──────────┐              ┌──────────┐             ┌──────────┐
 │   │ Pickup 1 │              │ Pickup 2 │             │ Pickup 3 │
 │   │ (Red)    │              │ (Green)  │             │ (Blue)   │
 │   └──────────┘              └──────────┘             └──────────┘
 │
 │                             (0.0, 0.0)
 │                             [  Home  ]
 │
 │                             [0.0, -1.90] Staging Approach
 │                                  │
 │                                  ▼
 │                             [0.0, -2.25] Docked Position
 │                       ┌─────────────────────┐
 │                       │   Placing Station   │
 │                       │ [Red] [Grn] [Blue]  │
 │                       └─────────────────────┘
 └─────────────────────────────────────────────────────────────────► X (meters)
```

### Exact Coordinates & Dimensions

#### 1. Pickup Stations (Tables: $0.30 \times 0.30 \times 0.10\text{ m}$, Top $Z = 0.10\text{ m}$)
| Station | Table World Pose $(x, y, z)$ | Target Object | Object Starting Pose $(x, y, z)$ | Nav Waypoint $(x, y, \theta)$ |
|---|---|---|---|---|
| **Pickup 1 (Red)** | `(-3.00, 2.75, 0.05)` | `cube_red` ($25\text{ mm}$ cube) | `(-3.00, 2.75, 0.1125)` | `(-3.00, 1.95, +1.5708)` |
| **Pickup 2 (Green)** | `(0.00, 2.75, 0.05)` | `cube_green` ($25\text{ mm}$ cube) | `(0.00, 2.75, 0.1125)` | `(0.00, 1.95, +1.5708)` |
| **Pickup 3 (Blue)** | `(3.00, 2.75, 0.05)` | `cube_blue` ($25\text{ mm}$ cube) | `(3.00, 2.75, 0.1125)` | `(3.00, 1.95, +1.5708)` |

#### 2. Placing Station (Purple Table & Elevated Platforms)
- **Base Table**: Purple table at `(0.00, -2.75, 0.05)`, size $0.30 \times 0.30 \times 0.10\text{ m}$ (surface at $Z = 0.10\text{ m}$).
- **Elevated Platforms**: Three colored platforms ($0.06 \times 0.06 \times 0.015\text{ m}$, top surface at $Z = 0.115\text{ m}$):
  - **Red Platform**: Center at `(-0.080, -2.640, 0.1075)`, Top $Z = 0.115\text{ m}$
  - **Green Platform**: Center at `(0.000, -2.640, 0.1075)`, Top $Z = 0.115\text{ m}$
  - **Blue Platform**: Center at `(+0.080, -2.640, 0.1075)`, Top $Z = 0.115\text{ m}$
- **Cube Settled Position**: When a $25\text{ mm}$ cube ($0.025\text{ m}$) rests flat on any platform, its center is at:
  $$Z_{\text{settled}} = 0.115 + \frac{0.025}{2} = 0.1275\text{ m}$$

---

## 5. Autonomous Missions & Operator CLI

The system supports four distinct automated missions:

```
Mission A: Pick RED Cube    ──► Carry ──► Place on RED Platform   ──► Return Home
Mission B: Pick GREEN Cube  ──► Carry ──► Place on GREEN Platform ──► Return Home
Mission C: Pick BLUE Cube   ──► Carry ──► Place on BLUE Platform  ──► Return Home
Mission D: Place-Only Demo  ──► Direct approach to Placing Station
```

### Triggering via Interactive CLI
Run the AMMS Operator CLI in a separate terminal:
```bash
ros2 run k12_pick_place amms_command
```
The CLI provides an interactive prompt displaying live robot coordinates and FSM state:
```
============================================================
              AMMS OPERATOR COMMAND INTERFACE
============================================================
  A  ->  Mission A : Pick Point 1 - RED   -> Place Red Platform
  B  ->  Mission B : Pick Point 2 - GREEN -> Place Green Platform
  C  ->  Mission C : Pick Point 3 - BLUE  -> Place Blue Platform
  D  ->  Mission D : Place Station (Place-only sequence)
  Q  ->  Quit CLI
============================================================
Current State: WAIT_FOR_COMMAND | Robot Pose: (0.00, 0.00, 0.00)
Enter command [A/B/C/D/Q]: 
```

### Triggering via ROS 2 Services
You can also trigger missions directly using ROS 2 standard Trigger service calls:
```bash
ros2 service call /amms/goto_a std_srvs/srv/Trigger "{}" # Mission A (Red)
ros2 service call /amms/goto_b std_srvs/srv/Trigger "{}" # Mission B (Green)
ros2 service call /amms/goto_c std_srvs/srv/Trigger "{}" # Mission C (Blue)
ros2 service call /amms/goto_d std_srvs/srv/Trigger "{}" # Mission D (Place only)
```

---

## 6. Manipulation, MoveIt 2 & Physical Verification

Manipulation is managed by `moveit_pick_place_commander.py` collaborating with MoveIt 2's `move_group`.

```
  Task Manager (FSM)
         │  /amms/pick_object or /amms/place_object
         ▼
  MoveItPickPlaceCommander
   ├── 1. Read Gazebo ground-truth & camera detection
   ├── 2. Add Planning Scene Obstacles (tables, platforms, arena walls)
   ├── 3. Plan collision-free trajectory via MoveIt 2 (OMPL RRTConnect)
   ├── 4. Execute via /arm_position_controller/follow_joint_trajectory
   ├── 5. Actuate parallel gripper (/gripper_position_controller/commands)
   └── 6. Verify physical settlement via GazeboObserver
```

### Collision Geometry & Planning Scene
MoveIt dynamically maintains accurate collision models so the arm never strikes the table edges:
- `pickup_tables`: $0.30 \times 0.30 \times 0.10\text{ m}$ at each pickup station.
- `placing_station`: $0.30 \times 0.30 \times 0.10\text{ m}$ purple base table.
- `platforms`: Three $0.06 \times 0.06 \times 0.015\text{ m}$ elevated platforms at $Z=0.1075\text{ m}$.
- `arena_walls`: Outer boundary walls preventing out-of-workspace maneuvers.

### Closed-Loop Physical Verification (`gazebo_observer.py`)
To prevent "ghost placements" where the robot opens the gripper in mid-air or drops the cube outside the platform, the system enforces two verification gates:

1. **Pre-Release Containment Gate**:
   - Compares the cube's live pose against the target platform center before opening the gripper.
   - Requires horizontal error $\le 14.5\text{ mm}$ (guaranteeing the $25\text{ mm}$ cube footprint is fully supported by the $60\text{ mm}$ platform).
   - Requires vertical release gap $\le 5\text{ mm}$ above the platform surface.
   - If a minor base docking error exists, the manipulator performs an online IK offset adjustment to center the cube exactly over the platform.

2. **Post-Release Settlement Gate**:
   - Waits for physics contact and settlement ($0.6\text{ s}$).
   - Verifies the cube is physically supported:
     $$|Z_{\text{cube\_bottom}} - Z_{\text{platform\_top}}| \le 3.0\text{ mm}$$
   - Confirms linear velocity $< 10\text{ mm/s}$ (no residual bounce or slide).
   - Only when verified does the arm retreat upwards and backwards to carry pose.

---

## 7. Navigation & Precise Docking Architecture

Navigating a mobile manipulator to a pickup/placing table requires millimeter-level heading and standoff precision.

```
       [ Stage 1: Global Navigation ]
         Navigate to place_station_approach (0.00, -1.90, -1.57)
         AMCL + DWB Planner (tolerant of obstacles)
                      │
                      ▼
       [ Stage 2: In-Place Heading Alignment ]
         Rotate purely in-place until yaw == -1.5708 rad (tolerance 0.03 rad)
                      │
                      ▼
       [ Stage 3: Straight-Line Forward Docking ]
         Drive straight 35 cm forward to place_station (0.00, -2.25)
         Tolerances: XY <= 0.035 m, Yaw <= 0.03 rad
                      │
                      ▼
       [ Stage 4: Station Docking Verification ]
         Confirm visual contact with purple table before calling /amms/place_object
```

### Staging Waypoint Parameters
```yaml
waypoint_place_station_approach: [0.0, -1.90, -1.570796] # 35 cm pre-dock staging
waypoint_place_station:          [0.0, -2.25, -1.570796] # Final placing stance
docking_arrival_xy_tolerance_m:   0.035                  # Strict XY arrival gate
docking_arrival_yaw_tolerance_rad: 0.03                  # Strict yaw arrival gate
```

---

## 8. System Interconnection & Data Flow

### 8.1 Launch Tree Hierarchy
```
amms_moveit_nav_pick_place.launch.py (Convenience Entry Point)
 └── amms_nav_pick_place.launch.py (Master Launch Description)
      ├── amms_navigation.launch.py
      │    ├── amms_gazebo.launch.py (articubot_one)
      │    │    ├── rsp.launch.py (robot_state_publisher)
      │    │    ├── Ignition Gazebo Sim (og.world)
      │    │    ├── ros_gz_bridge (gz_bridge.yaml)
      │    │    └── controller_manager (my_controllers.yaml)
      │    ├── cloud_to_scan (pointcloud_to_laserscan.yaml)
      │    ├── ekf_node (ekf_odom.yaml)
      │    ├── localization_launch.py (Nav2 AMCL)
      │    ├── navigation_launch.py (Nav2 DWB / BT Navigator)
      │    ├── waypoint_manager (amms_waypoints.yaml)
      │    └── rviz2 (amms_nav_pick_place.rviz)
      ├── object_detector (pick_place_service.yaml)
      ├── move_group node (MoveIt 2 OMPL / KDL / SRDF)
      ├── moveit_pick_place_commander (moveit_nav_pick_place.yaml)
      ├── readiness_gate (synchronization barrier)
      └── amms_task_manager (amms_waypoints.yaml)
```

### 8.2 ROS 2 Topic and Service Map
```
Ignition Gazebo (og.world)
  │ (ros_gz_bridge)
  ├── /scan                     ──► Nav2 Costmaps & Task Manager
  ├── /imu                      ──► robot_localization EKF
  ├── /diff_cont/odom           ──► robot_localization EKF ──► /odometry/filtered
  ├── /camera/depth_camera/...  ──► object_detector ──► /amms/detections
  ├── /joint_states             ──► MoveIt move_group & Commanders
  └── /gazebo/pose_info         ──► gazebo_observer (ground truth)

amms_task_manager
  ├── [Action] navigate_to_pose ──► Nav2 bt_navigator ──► /cmd_vel
  ├── [Action] backup           ──► Nav2 behavior_server ──► /cmd_vel
  ├── [Service] /amms/pick_object  ──► moveit_pick_place_commander
  ├── [Service] /amms/place_object ──► moveit_pick_place_commander
  ├── [Service] /amms/fold_arm     ──► moveit_pick_place_commander
  └── [Pub]     /amms/task_state   ──► amms_command CLI & RViz

moveit_pick_place_commander
  ├── [Action] /arm_position_controller/follow_joint_trajectory ──► ros2_control
  ├── [Pub]    /gripper_position_controller/commands            ──► ros2_control
  └── [MoveIt] Planning Scene Updates & OMPL Motion Planning
```

---

## 9. How to Build and Run

### 9.1 Build the Workspace
Always build from `/home/nabhan/try_18`:
```bash
cd /home/nabhan/try_18
source /opt/ros/humble/setup.bash

# Build all packages with symlink install:
colcon build --symlink-install

# Source overlay:
source install/setup.bash
```

### 9.2 Launch the Primary Autonomous System
Run the complete simulation stack (Gazebo + Navigation + MoveIt + Task Manager):
```bash
source /opt/ros/humble/setup.bash
source /home/nabhan/try_18/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 launch k12_pick_place amms_moveit_nav_pick_place.launch.py
```

### 9.3 Command a Mission (Operator CLI)
Open a second terminal, source the overlay, set domain, and run:
```bash
source /opt/ros/humble/setup.bash
source /home/nabhan/try_18/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 run k12_pick_place amms_command
```
Press `A`, `B`, `C`, or `D` and hit Enter.

### 9.4 Running Automated Verification Tests
The repository includes a comprehensive unit and integration test suite:
```bash
cd /home/nabhan/try_18
pytest src/k12_pick_place/test/ -v
```

---

## 10. Tuning & Configuration Reference

### Mobile Base Speeds (`config/nav2_amms.yaml`)
To adjust navigation velocity and acceleration:
```yaml
FollowPath:                     # DWB local planner
  max_vel_x: 0.25               # Forward speed (m/s)
  max_vel_theta: 0.50           # Angular speed (rad/s)
  acc_lim_x: 0.35               # Linear acceleration (m/s²)
  acc_lim_theta: 0.75           # Angular acceleration (rad/s²)
```

### Arm Planning & Speed (`config/moveit_nav_pick_place.yaml`)
To adjust MoveIt velocity scaling and planning attempts:
```yaml
moveit_pick_place_commander:
  ros__parameters:
    move_group_velocity_scale: 0.5        # 0.1 to 1.0 (50% speed)
    move_group_acceleration_scale: 0.5    # 0.1 to 1.0 (50% accel)
    move_group_planning_time: 10.0        # Max planning budget (sec)
    move_group_num_planning_attempts: 10  # OMPL sample attempts
```

### Gripper Stroke Hardware Limits (`description/gripper.xacro`)
> [!CAUTION]
> The physical prismatic joint limit in `gripper.xacro` is `0.0394875 m`. Never set `open_gripper_width` higher than this value in configuration files:
> - `open_gripper_width: 0.0115` (Sufficient for $25\text{ mm}$ cube)
> - `closed_gripper_width: 0.000` (Fully closed on target)

### Docking & Standoff Tolerances (`config/amms_waypoints.yaml`)
```yaml
amms_task_manager:
  ros__parameters:
    docking_arrival_xy_tolerance_m: 0.035    # Strict final translation tolerance
    docking_arrival_yaw_tolerance_rad: 0.03  # Strict final orientation tolerance
    post_pick_backup_distance: 0.50          # Reversing distance after grasp
    post_place_backup_distance: 0.35         # Reversing distance after placement
```
