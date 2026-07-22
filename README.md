# K12 AMMS — Autonomous Mobile Manipulator System
## ROS 2 Humble + Ignition Gazebo Pick-and-Place Pipeline

> **Maintainer**: nabhan (naban559@gmail.com)
> **Robot Domain**: `ROS_DOMAIN_ID=77`
> **Platform**: ROS 2 Humble | Ignition Gazebo (Gazebo Sim 6)

---

## Table of Contents
1. [What This System Does](#1-what-this-system-does)
2. [Workspace Layout](#2-workspace-layout)
3. [Package 1 — k12_pick_place](#3-package-1--k12_pick_place)
4. [Package 2 — articubot_one_with_arm](#4-package-2--articubot_one_with_arm)
5. [How All Files Are Connected](#5-how-all-files-are-connected)
6. [Mission State Machine](#6-mission-state-machine)
7. [How to Build and Run](#7-how-to-build-and-run)
8. [Tuning Parameters Reference](#8-tuning-parameters-reference)

---

## 1. What This System Does

The K12 AMMS is a fully autonomous mobile robot that:
1. **Navigates** to a pickup station (Nav2 + AMCL + DWB local planner)
2. **Detects** a coloured block using an RGB-D camera (OpenCV color segmentation)
3. **Picks** the block using a 5-DOF arm and parallel gripper
4. **Carries** it across the room to a pharmacy shelf
5. **Places** it precisely at the correct shelf slot
6. **Returns home** — mission complete

Everything is automatic. No human intervention required after launch.

---

## 2. Workspace Layout

```
~/try_12/
├── src/                         → symlink to src_1/src/
├── src_1/src/
│   ├── articubot_one_with_arm/  → Robot hardware description + Gazebo sim
│   └── k12_pick_place/          → Mission logic + manipulation + perception
├── build/                       → colcon build output
├── install/                     → installed packages (sourced at runtime)
├── log/                         → build and launch logs
└── README.md                    → This file
```

> Before running anything, always source both overlays:
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/try_12/install/setup.bash
> ```

---

## 3. Package 1 — `k12_pick_place`

**Type**: Python (`ament_python`)
**Location**: `src_1/src/k12_pick_place/`

This package owns the entire mission — navigation goals, manipulation commands, object detection, and the state machine that ties them together.

### 3.1 Directory Tree

```
k12_pick_place/
├── k12_pick_place/               ← Python source (ROS 2 nodes)
│   ├── __init__.py
│   ├── ik.py                     ← Arm IK solver (pure Python library)
│   ├── object_detector.py        ← RGB-D camera detection node
│   ├── pick_place_commander.py   ← Arm & gripper execution node
│   ├── amms_task_manager.py      ← Master mission FSM node
│   ├── readiness_gate.py         ← Startup synchronization node
│   ├── scene_spawner.py          ← Gazebo object spawner node
│   ├── cloud_to_scan.py          ← PointCloud → LaserScan node
│   └── waypoint_manager.py       ← Waypoint visualizer node
├── config/
│   ├── amms_waypoints.yaml       ← Waypoints + task manager params
│   ├── nav2_amms.yaml            ← Nav2 stack (DWB, costmaps, AMCL)
│   ├── pick_place_service.yaml   ← Detector + arm commander params
│   ├── pick_place.yaml           ← Same (standalone use)
│   ├── pick_place_standalone.yaml← Same (demo without task manager)
│   ├── ekf_odom.yaml             ← EKF sensor fusion params
│   ├── pointcloud_to_laserscan.yaml ← CloudToScan params
│   └── slam_toolbox_amms.yaml    ← SLAM mapping params
├── launch/
│   ├── amms_nav_pick_place.launch.py  ← PRIMARY: full autonomous mission
│   ├── amms_navigation.launch.py      ← Navigation only
│   ├── amms_mapping.launch.py         ← SLAM mapping mode
│   └── pick_place_demo.launch.py      ← Arm demo only
├── maps/
│   └── pharma.yaml               ← Pre-built map of pharmacy world
├── models/                        ← SDF models (tables, blocks)
├── rviz/
│   └── amms_nav_pick_place.rviz  ← RViz2 display config
└── setup.py                       ← Entry points: node executables
```

### 3.2 How Nodes Are Registered (setup.py)

`setup.py` maps each Python `main()` function to a CLI command:

```python
'console_scripts': [
    'object_detector      = k12_pick_place.object_detector:main',
    'pick_place_commander = k12_pick_place.pick_place_commander:main',
    'cloud_to_scan        = k12_pick_place.cloud_to_scan:main',
    'waypoint_manager     = k12_pick_place.waypoint_manager:main',
    'amms_task_manager    = k12_pick_place.amms_task_manager:main',
    'scene_spawner        = k12_pick_place.scene_spawner:main',
    'readiness_gate       = k12_pick_place.readiness_gate:main',
]
```

After `colcon build`, each becomes a CLI command:
```bash
ros2 run k12_pick_place <command_name>
```

---

### 3.3 Python Nodes — Detailed Explanation

#### `ik.py` — Inverse Kinematics Solver

**Type**: Pure Python library (NOT a ROS node)
**Imported by**: `pick_place_commander.py`, `amms_task_manager.py`

```python
from k12_pick_place.ik import PlanarArmIK
```

Analytical geometry IK solver for the 5-DOF arm. No topics, no subscribers — just math.

| Class/Function | Purpose |
|---------------|---------|
| `ArmGeometry` | Frozen dataclass with link lengths — MUST match `arm.xacro` |
| `JointLimits` | Min/max joint angle bounds |
| `PlanarArmIK.solve(x, y, z)` | Returns `(yaw, shoulder, elbow, wrist, roll)` or `None` if unreachable |
| `PlanarArmIK.is_reachable(x, y, z)` | Returns True/False |

**Arm reach**: Between ~0.14 m and ~0.55 m from `arm_base_link`. Targets outside this return `None`.

**Joints solved** (order matches `arm_position_controller`):
1. `arm_base_yaw_joint`
2. `arm_shoulder_pitch_joint`
3. `arm_elbow_pitch_joint`
4. `arm_wrist_pitch_joint`
5. `arm_wrist_roll_joint`

> **Critical**: If you resize the arm in `arm.xacro`, you MUST update the link lengths in `ArmGeometry` to match.

---

#### `object_detector.py` — RGB-D Object Detector

**ROS Node Name**: `k12_object_detector`
**Config**: `pick_place_service.yaml` → section `k12_object_detector`
**Executable**: `ros2 run k12_pick_place object_detector`

Uses OpenCV color segmentation on the RGB image to find the blue block. Reads depth at the detected pixel to compute 3D position in camera frame. Publishes result as a JSON string.

**Subscribes to**:
| Topic | Type | Source |
|-------|------|--------|
| `/camera/depth_camera/image` | `sensor_msgs/Image` | GZ Bridge (from Gazebo camera) |
| `/camera/depth_camera/depth_image` | `sensor_msgs/Image` | GZ Bridge |
| `/camera/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | GZ Bridge |

**Publishes to**:
| Topic | Type | Content |
|-------|------|---------|
| `/amms/detections` | `std_msgs/String` | JSON: `{"label":"blue_rectangular_block","x":0.628,"y":-0.089,"z":-0.058,"confidence":0.91}` |
| `/amms/detected_object_poses_camera` | `geometry_msgs/PoseArray` | 3D poses for RViz |
| `/amms/detection_debug` | `sensor_msgs/Image` | Annotated image for debugging |

**Who consumes `/amms/detections`**:
- `pick_place_commander.py` — reads pose to compute IK for grasp
- `amms_task_manager.py` — monitors freshness to decide when to pick

---

#### `pick_place_commander.py` — Arm & Gripper Execution Engine

**ROS Node Name**: `k12_pick_place_commander`
**Config**: `pick_place_service.yaml` → section `k12_pick_place_commander`
**Executable**: `ros2 run k12_pick_place pick_place_commander`
**Imports**: `from k12_pick_place.ik import PlanarArmIK`

This is the manipulation executor. It receives commands (via ROS services) from the task manager and physically moves the arm and gripper.

**Subscribes to**:
| Topic | Type | Purpose |
|-------|------|---------|
| `/amms/detections` | `std_msgs/String` | Read detected object pose |
| `/amms/pick_target` | `std_msgs/String` | External pick target override |
| `/joint_states` | `sensor_msgs/JointState` | Verify gripper reached position |

**Action Clients** (sends commands to):
| Action | Type | Moves |
|--------|------|-------|
| `/arm_position_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 5 arm joints |

**Publishers**:
| Topic | Type | Purpose |
|-------|------|---------|
| `/arm_position_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Topic fallback (if action unavailable) |
| `/gripper_position_controller/commands` | `std_msgs/Float64MultiArray` | `[left_finger_pos, right_finger_pos]` |

**Service Servers** (called by `amms_task_manager`):
| Service | Type | Action Taken |
|---------|------|-------------|
| `/amms/pick_object` | `std_srvs/Trigger` | Full pick sequence |
| `/amms/place_object` | `std_srvs/Trigger` | Full place sequence |
| `/amms/fold_arm` | `std_srvs/Trigger` | Move to home/carry pose |

**Pick sequence** (on `/amms/pick_object` call):
1. Read latest detection → get 3D position
2. TF2 transform → `arm_base_link` frame
3. `PlanarArmIK.solve()` → joint angles
4. Open gripper (0.039 m)
5. Move to pre-grasp pose
6. Move to grasp pose
7. Close gripper (0.00146 m)
8. Verify via `/joint_states` that both fingers closed
9. Lift arm to carry pose

---

#### `amms_task_manager.py` — Mission State Machine

**ROS Node Name**: `amms_task_manager`
**Config**: `amms_waypoints.yaml` → section `amms_task_manager`
**Executable**: `ros2 run k12_pick_place amms_task_manager`
**Imports**: `from k12_pick_place.ik import PlanarArmIK`

The brain of the mission. Runs a Finite State Machine that:
- Sends Nav2 navigation goals
- Calls pick/place services
- Monitors sensors
- Handles failures and retries

**Action Clients** (navigation commands):
| Action | Type | Used For |
|--------|------|---------|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Go to waypoints |
| `drive_on_heading` | `nav2_msgs/DriveOnHeading` | Straight-line shelf docking |
| `backup` | `nav2_msgs/BackUp` | Reverse after pick/place |

**Service Clients** (manipulation commands):
| Service | Target Node | Purpose |
|---------|------------|---------|
| `/amms/pick_object` | `pick_place_commander` | Pick the block |
| `/amms/place_object` | `pick_place_commander` | Place the block |
| `/amms/fold_arm` | `pick_place_commander` | Safe carry pose |
| `/local_costmap/clear_entirely_local_costmap` | Nav2 | Unstuck recovery |
| `/global_costmap/clear_entirely_global_costmap` | Nav2 | Unstuck recovery |

**Subscribes to**:
| Topic | Type | Purpose |
|-------|------|---------|
| `/amms/detections` | `std_msgs/String` | Object detection freshness check |
| `/scan` | `sensor_msgs/LaserScan` | Nearest obstacle monitoring |

**Publishes**:
| Topic | Type | Purpose |
|-------|------|---------|
| `/amms/task_state` | `std_msgs/String` | Current FSM state name |
| `/amms/pick_target` | `std_msgs/String` | Confirmed detection for commander |

---

#### `readiness_gate.py` — Startup Synchronization

**ROS Node Name**: `amms_readiness_gate`
**Config**: Inline in `amms_nav_pick_place.launch.py`
**Executable**: `ros2 run k12_pick_place readiness_gate`

Runs a blocking poll loop at startup. The `amms_task_manager` only begins after this gate exits successfully.

**What it waits for**:
- Topics publishing: `/scan`, `/odometry/filtered`, `/amms/detections`
- Services available: `/amms/pick_object`, `/amms/place_object`, `/amms/fold_arm`
- Lifecycle nodes ACTIVE: all 8 Nav2 servers (`/map_server`, `/amcl`, etc.)
- TF available: `map → base_link`
- Action servers ready: `navigate_to_pose`, `drive_on_heading`
- Trajectory action: `/arm_position_controller/follow_joint_trajectory`

If lifecycle nodes are inactive, it tries to activate them automatically.
**Timeout**: 300 seconds. If not ready, exits with error.

---

#### `scene_spawner.py` — Environment Object Spawner

**ROS Node Name**: `amms_scene_spawner`
**Config**: Inline parameters in launch file
**Executable**: `ros2 run k12_pick_place scene_spawner`

Runs once at startup, then terminates. Checks for Gazebo Classic `/get_entity_state` service:
- **Ignition Gazebo** (our setup): service unavailable → waits 2s → logs info → exits with `True`
- **Gazebo Classic** (legacy): would spawn tables programmatically

Objects (`blue_rectangular_block`, Shelf A, tables) are pre-defined in `pharma.world`.

---

#### `cloud_to_scan.py` — Point Cloud → Laser Scan

**ROS Node Name**: `pointcloud_to_laserscan`
**Config**: `pointcloud_to_laserscan.yaml`
**Executable**: `ros2 run k12_pick_place cloud_to_scan`

The depth camera produces a 3D point cloud. Nav2 obstacle avoidance needs a flat 2D laser scan. This node extracts points within a configured height band and converts them.

**Subscribes to**:
| Topic | Type | Source |
|-------|------|--------|
| `/camera/depth_camera/points` | `sensor_msgs/PointCloud2` | GZ Bridge |

**Publishes**:
| Topic | Type | Consumed By |
|-------|------|------------|
| `/scan` | `sensor_msgs/LaserScan` | Nav2 costmaps, `amms_task_manager` |

**Self-filter**: Ignores points within the robot's own body bounding box to prevent the robot from seeing itself as an obstacle.

---

#### `waypoint_manager.py` — Waypoint Publisher

**ROS Node Name**: `amms_waypoint_manager`
**Config**: `amms_waypoints.yaml` → section `amms_waypoint_manager`
**Executable**: `ros2 run k12_pick_place waypoint_manager`

Reads waypoint coordinates from config and publishes them as visual markers in RViz. Does NOT send navigation goals — visualization only.

**Publishes**:
| Topic | Type | Purpose |
|-------|------|---------|
| `/amms/waypoints` | `geometry_msgs/PoseArray` | All waypoints as poses |
| `/amms/waypoint_markers` | `visualization_msgs/MarkerArray` | Blue arrows + text labels in RViz |

**Configured waypoints**:
| Name | (x, y, yaw) | Map Location |
|------|------------|-------------|
| `home` | (0.0, 0.0, 0°) | Robot start position |
| `pick_station` | (0.85, 0.0, 0°) | In front of pickup table |
| `pick_clear` | (0.5, 0.0, 0°) | Clear of pickup table |
| `place_approach` | (-3.0, 2.5, 180°) | Staging before Shelf A |
| `place_station` | (-4.0, 2.5, 180°) | In front of Shelf A |
| `place_clear` | (-3.0, 2.5, 0°) | After placement, clearing zone |

---

### 3.4 Configuration Files

#### `amms_waypoints.yaml` — Most Important Tuning File

**Loaded by**: `amms_task_manager` and `waypoint_manager` nodes
**Path**: `config/amms_waypoints.yaml`

Has TWO sections, each configuring a different node:

```yaml
amms_waypoint_manager:      # configures waypoint_manager.py
  ros__parameters:
    waypoint_pick_station: [0.85, 0.0, 0.0]
    waypoint_place_station: [-4.0, 2.5, 3.14159]

amms_task_manager:           # configures amms_task_manager.py
  ros__parameters:
    align_pick_station_stop_distance: 0.65
    place_final_approach_speed: 0.08
    position_only_min_obstacle_range: 0.25
```

Key parameters:
| Parameter | Value | Effect |
|-----------|-------|--------|
| `waypoint_pick_station` | `[0.85, 0.0, 0.0]` | Robot stop point near pickup table |
| `align_pick_station_stop_distance` | `0.65 m` | Final alignment stop distance from table |
| `object_nav_standoff_distance` | `0.60 m` | Standoff distance for IK reach |
| `place_final_approach_speed` | `0.08 m/s` | Final docking speed |
| `place_final_standoff_xy_tolerance` | `0.30 m` | Acceptable proximity before placing |
| `position_only_min_obstacle_range` | `0.25 m` | Min LiDAR clearance for placement standoff |
| `post_pick_backup_distance` | `0.53 m` | How far to reverse after picking |

---

#### `nav2_amms.yaml` — Navigation Stack Config

**Loaded by**: Nav2 bringup (localization + navigation launch files)
**Path**: `config/nav2_amms.yaml`

| Section | Key Settings |
|---------|-------------|
| `controller_server` | DWB planner: `max_vel_x: 0.21 m/s`, `max_vel_theta: 0.40 rad/s` |
| `velocity_smoother` | `max_velocity: [0.225, 0.0, 0.525]` |
| `planner_server` | NavFn global planner |
| `behavior_server` | Spin, BackUp, DriveOnHeading behaviors |
| `local_costmap` | Footprint `0.42×0.30 m`, `inflation_radius: 0.48` |
| `global_costmap` | 12×10 m full-room map |
| `amcl` | Particle filter localization from `/scan` |

---

#### `pick_place_service.yaml` — Arm & Detector Config

**Loaded by**: `object_detector` and `pick_place_commander` nodes
**Path**: `config/pick_place_service.yaml`

Two YAML sections:
```yaml
k12_object_detector:              # configures object_detector.py
  ros__parameters:
    rgb_topic: /camera/depth_camera/image
    depth_topic: /camera/depth_camera/depth_image
    ...

k12_pick_place_commander:         # configures pick_place_commander.py
  ros__parameters:
    open_gripper_width: 0.039     # max = 0.0394875 (URDF limit)
    closed_gripper_width: 0.0014625
    home_joints: [0.0, -1.35, 2.20, -0.85, 0.0]
    carry_joints: [0.0, -1.10, 1.85, -0.75, 0.0]
    place_retreat_step_duration_sec: 0.50
```

> **Critical**: `open_gripper_width` must NEVER exceed `0.0394875 m` (the URDF joint upper limit defined in `gripper.xacro`).

---

#### `ekf_odom.yaml` — Sensor Fusion Config

**Loaded by**: `robot_localization ekf_node`
**Path**: `config/ekf_odom.yaml`

Fuses two sensor inputs into smooth odometry:
| Input | Topic | Sensor |
|-------|-------|--------|
| Wheel odometry | `/diff_cont/odom` | `diff_drive_controller` |
| IMU | `/imu` | Gazebo IMU sensor |

Outputs:
| Output | Topic | Consumed By |
|--------|-------|------------|
| Filtered odometry | `/odometry/filtered` | Nav2, task manager |
| TF transform | `odom → base_footprint` | TF tree |

---

### 3.5 Launch Files

#### `amms_nav_pick_place.launch.py` — PRIMARY (Run This)

**Path**: `launch/amms_nav_pick_place.launch.py`

The single command that starts the entire system. It:
1. Sets `ROS_DOMAIN_ID=77`
2. Includes → `amms_navigation.launch.py`
3. Starts `scene_spawner`
4. Starts `object_detector` (with `pick_place_service.yaml`)
5. Starts `pick_place_commander` (with `pick_place_service.yaml`)
6. Starts `readiness_gate`
7. Starts `amms_task_manager` (with `amms_waypoints.yaml`)

Launch arguments (all have defaults, override as needed):
| Argument | Default | Override Example |
|----------|---------|-----------------|
| `map` | `maps/k12_warehouse.yaml` | `map:=.../pharma.yaml` |
| `world` | `pharma.world` | (usually default) |
| `gui` | `true` | `gui:=false` |
| `rviz` | `true` | `rviz:=false` |
| `run_task_manager` | `true` | `run_task_manager:=false` |
| `ros_domain_id` | `77` | `ros_domain_id:=0` |

---

#### `amms_navigation.launch.py`

**Path**: `launch/amms_navigation.launch.py`
**Called by**: `amms_nav_pick_place.launch.py`

Starts simulation + navigation stack:
- `amms_gazebo.launch.py` (Gazebo + robot + controllers)
- `cloud_to_scan` node
- `ekf_node` (odometry fusion)
- `nav2_bringup/localization_launch.py` (AMCL)
- `nav2_bringup/navigation_launch.py` (DWB + behaviors)
- `waypoint_manager` node
- `rviz2` node

---

#### `amms_mapping.launch.py`

**Path**: `launch/amms_mapping.launch.py`
**Use**: Create a new map with SLAM Toolbox. Drive robot manually, then save:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/new_map
```

---

#### `pick_place_demo.launch.py`

**Path**: `launch/pick_place_demo.launch.py`
**Use**: Test arm movements in isolation without the full navigation stack.

---

## 4. Package 2 — `articubot_one_with_arm`

**Type**: CMake (`ament_cmake`)
**Registered as**: `articubot_one` (use this name in `ros2 pkg`, `FindPackageShare`, etc.)
**Location**: `src_1/src/articubot_one_with_arm/`

Owns the robot's physical description (URDF), simulation world, and hardware interface.

### 4.1 URDF / Xacro Files (`description/`)

These define every link, joint, sensor, and Gazebo plugin on the robot:

| File | Role |
|------|------|
| `four_wheel.xacro` | Base chassis: 4 wheels, body, IMU, RGB-D camera, LiDAR. URDF entry point. |
| `arm.xacro` | 5-DOF arm links/joints. Link lengths **MUST match** `ik.py:ArmGeometry`. |
| `gripper.xacro` | 2-finger gripper. `finger_travel = 0.0394875 m` = hardware max open width. |
| `arm_ros2_control.xacro` | `ign_ros2_control` plugin — links arm/gripper joints to `controller_manager`. |
| `arm_gazebo.xacro` | Ignition Gazebo physics/sensor plugins for the arm. |
| `four_wheel.gazebo` | Gazebo plugins for base (diff drive, IMU, camera, LiDAR). |
| `materials.xacro` | RViz color definitions. |

**The `arm.xacro` ↔ `ik.py` dependency**:
```
arm.xacro  defines physical link lengths
    ↕ MUST be identical
ik.py ArmGeometry  uses those lengths for IK math
```
If you resize the arm, update BOTH files.

---

### 4.2 Configuration Files

#### `my_controllers.yaml` — Hardware Controllers

**Loaded by**: `ros2_control controller_manager` (via `amms_gazebo.launch.py`)

| Controller | Type | Interface |
|-----------|------|-----------|
| `diff_cont` | `DiffDriveController` | Reads `/cmd_vel`, publishes `/diff_cont/odom` |
| `joint_broad` | `JointStateBroadcaster` | Publishes `/joint_states` for all joints |
| `arm_position_controller` | `JointTrajectoryController` | Serves `/arm_position_controller/follow_joint_trajectory` |
| `gripper_position_controller` | `ForwardCommandController` | Reads `/gripper_position_controller/commands` |

Wheel parameters:
- `wheel_separation: 0.27 m` (measured from URDF joint Y positions)
- `wheel_radius: 0.06 m`
- `enable_odom_tf: false` (EKF publishes the TF instead)

---

#### `gz_bridge.yaml` — Gazebo ↔ ROS Topic Bridge

**Loaded by**: `ros_gz_bridge` node (in `amms_gazebo.launch.py`)

Maps Ignition Gazebo internal topics to ROS 2. Without this bridge, no sensor data reaches ROS nodes.

Key mappings:
| Gazebo Topic | ROS Topic | Type |
|-------------|-----------|------|
| `/clock` | `/clock` | Clock |
| `/scan` | `/scan` | `sensor_msgs/LaserScan` |
| `/imu` | `/imu` | `sensor_msgs/Imu` |
| `/camera/depth_camera/image` | `/camera/depth_camera/image` | `sensor_msgs/Image` |
| `/camera/depth_camera/depth_image` | `/camera/depth_camera/depth_image` | `sensor_msgs/Image` |
| `/camera/depth_camera/camera_info` | `/camera/depth_camera/camera_info` | `sensor_msgs/CameraInfo` |

---

### 4.3 Launch Files

#### `amms_gazebo.launch.py`

**Called by**: `amms_navigation.launch.py`
**Starts**:
1. `rsp.launch.py` — Robot State Publisher (URDF → `/robot_description` + TF)
2. Ignition Gazebo with `pharma.world`
3. `ros_gz_bridge` (using `gz_bridge.yaml`)
4. `controller_manager` (spawning all 4 controllers from `my_controllers.yaml`)

---

## 5. How All Files Are Connected

### 5.1 Launch Chain

```
amms_nav_pick_place.launch.py       ← YOU RUN THIS
│
├── amms_navigation.launch.py        (included)
│   │
│   ├── amms_gazebo.launch.py        (from articubot_one)
│   │   ├── rsp.launch.py            → Robot State Publisher (URDF)
│   │   ├── Ignition Gazebo          → loads pharma.world
│   │   ├── ros_gz_bridge            → uses gz_bridge.yaml
│   │   └── controller_manager       → uses my_controllers.yaml
│   │
│   ├── cloud_to_scan node           → uses pointcloud_to_laserscan.yaml
│   ├── ekf_node                     → uses ekf_odom.yaml
│   ├── localization_launch.py       → (nav2_bringup) uses nav2_amms.yaml
│   ├── navigation_launch.py         → (nav2_bringup) uses nav2_amms.yaml
│   ├── waypoint_manager node        → uses amms_waypoints.yaml
│   └── rviz2 node                   → uses amms_nav_pick_place.rviz
│
├── scene_spawner node               (inline params in launch file)
├── object_detector node             → uses pick_place_service.yaml
├── pick_place_commander node        → uses pick_place_service.yaml
├── readiness_gate node              (inline params in launch file)
└── amms_task_manager node           → uses amms_waypoints.yaml
```

---

### 5.2 ROS Topics & Services Data Flow

```
GAZEBO (pharma.world)
 │  (via ros_gz_bridge using gz_bridge.yaml)
 ├─ /scan           → cloud_to_scan → /scan → [Nav2 costmaps]
 │                                          → [amms_task_manager: obstacle monitor]
 ├─ /imu            → ekf_node
 ├─ /diff_cont/odom → ekf_node
 │      ekf_node → /odometry/filtered → [Nav2 odom source]
 │      ekf_node → TF: odom → base_footprint → [TF tree]
 │
 ├─ /camera/.../image      → object_detector
 ├─ /camera/.../depth      → object_detector
 ├─ /camera/.../info       → object_detector
 │      object_detector → /amms/detections → [pick_place_commander: grasp IK]
 │                                        → [amms_task_manager: freshness check]
 │
 ├─ /joint_states → pick_place_commander [gripper feedback verification]
 │
 └─ arm_position_controller (action server) ← pick_place_commander sends goals

amms_task_manager
 ├─ [ACTION] navigate_to_pose → bt_navigator → DWB planner → /cmd_vel → Gazebo wheels
 ├─ [ACTION] drive_on_heading → behavior_server → /cmd_vel
 ├─ [ACTION] backup           → behavior_server → /cmd_vel
 ├─ [SERVICE] /amms/pick_object  → pick_place_commander
 ├─ [SERVICE] /amms/place_object → pick_place_commander
 └─ [SERVICE] /amms/fold_arm     → pick_place_commander

pick_place_commander
 ├─ [IMPORT]  PlanarArmIK from ik.py
 ├─ [ACTION]  /arm_position_controller/follow_joint_trajectory → ros2_control → Gazebo
 ├─ [PUBLISH] /gripper_position_controller/commands → ros2_control → Gazebo
 └─ [SERVES]  /amms/pick_object, /amms/place_object, /amms/fold_arm
```

---

### 5.3 Python Node Dependency Graph

```
ik.py (pure Python library, no ROS)
 ↑ imported by          ↑ imported by
 │                      │
pick_place_commander   amms_task_manager
 │ serves services       │ calls services
 └──────────────────────►│
                         │ calls Nav2 actions
                         ↓
                    Nav2 (bt_navigator, etc.)
                         │ sends /cmd_vel
                         ↓
                    Gazebo diff drive

object_detector
 │ publishes /amms/detections
 ├──────────────────────► pick_place_commander (grasp target)
 └──────────────────────► amms_task_manager (freshness monitor)

cloud_to_scan
 │ publishes /scan
 ├──────────────────────► Nav2 costmaps (obstacle avoidance)
 └──────────────────────► amms_task_manager (LiDAR check)

waypoint_manager         → RViz2 only (visualization)
scene_spawner            → runs once at startup, then exits
readiness_gate           → runs once at startup, then exits
```

---

### 5.4 Config File ↔ Node Mapping Table

| Config File | Configures | Loaded Via |
|-------------|-----------|-----------|
| `amms_waypoints.yaml` | `amms_waypoint_manager` + `amms_task_manager` | `parameters=[waypoint_params_file]` in launch |
| `pick_place_service.yaml` | `k12_object_detector` + `k12_pick_place_commander` | `parameters=[pick_place_params_file]` in launch |
| `nav2_amms.yaml` | All 8 Nav2 nodes (AMCL, DWB, BT Navigator, costmaps...) | Nav2 bringup `params_file` argument |
| `ekf_odom.yaml` | `ekf_filter_node` (robot_localization) | `parameters=[ekf_params_file]` in launch |
| `pointcloud_to_laserscan.yaml` | `pointcloud_to_laserscan` (cloud_to_scan) | `parameters=[scan_params_file]` in launch |
| `my_controllers.yaml` | `controller_manager` (ros2_control) | `amms_gazebo.launch.py` |
| `gz_bridge.yaml` | `ros_gz_bridge` | `amms_gazebo.launch.py` |

---

## 6. Mission State Machine

`amms_task_manager` executes these states in order:

```
START
 │ → Arm folds to carry pose (/amms/fold_arm)
 ↓
NAVIGATE_TO_PICK_ZONE
 │ → Nav2 goal → pick_station [x=0.85, y=0.0]
 ↓
SEARCH_FOR_OBJECT
 │ → Wait for fresh detection on /amms/detections
 ↓
VALIDATE_OBJECT
 │ → Check IK reachability (PlanarArmIK.solve())
 │ → If unreachable → REPOSITION_FOR_REACH
 ↓
CONFIRM_FRESH_DETECTION
 │ → Ensure detection is < 0.75s old
 ↓
PICK_OBJECT
 │ → Call /amms/pick_object → pick_place_commander executes full pick
 ↓
VERIFY_GRASP
 │ → Confirm gripper joints reached closed position
 ↓
RETRACT_ARM
 │ → /amms/fold_arm → carry pose
 ↓
POST_PICK_BACKUP
 │ → Nav2 BackUp 0.53m (4 segments of 0.16m)
 ↓
NAVIGATE_TO_PLACE_APPROACH
 │ → Nav2 goal → place_approach [x=-3.0, y=2.5]
 ↓
NAVIGATE_TO_PLACE_ZONE
 │ → Nav2 goal → place_station [x=-4.0, y=2.5]
 ↓
ALIGN_TO_PLACE_STATION
 │ → Fine-align heading to face Shelf A
 ↓
STAGE_FOR_PLACE_APPROACH
 │ → Navigate to orientation staging point
 ↓
FINAL_APPROACH_TO_PLACE
 │ → DriveOnHeading straight toward shelf at 0.08 m/s
 ↓
DETECT_PLACEMENT_SURFACE + ESTIMATE_PLACE_POSE
 │ → Verify arm can reach shelf slot
 ↓
PLACE_OBJECT
 │ → Call /amms/place_object → pick_place_commander executes full place
 ↓
VERIFY_RELEASE
 │ → Confirm gripper opened + object released
 ↓
RETRACT_ARM → POST_PLACE_BACKUP → CLEAR_PLACE_ZONE
 ↓
RETURN_TO_SEARCH_AREA
 │ → Nav2 goal → home [x=0.0, y=0.0]
 ↓
IDLE  ← MISSION COMPLETE ✓
```

---

## 7. How to Build and Run

### Build

```bash
cd ~/try_12
source /opt/ros/humble/setup.bash

# Full clean build (needed after any Python, URDF, or C++ change):
rm -rf build install log
colcon build

# Source after building:
source install/setup.bash
```

> YAML config files do NOT require a rebuild — they are read directly from `install/share/` at runtime.

### Launch Full Autonomous Mission

```bash
source /opt/ros/humble/setup.bash
source ~/try_12/install/setup.bash

ros2 launch k12_pick_place amms_nav_pick_place.launch.py \
  map:=/home/nabhan/try_12/src/k12_pick_place/maps/pharma.yaml
```

### Inspect the Running System (separate terminal)

```bash
# MUST set domain ID to see the running nodes:
export ROS_DOMAIN_ID=77

ros2 node list                          # all active nodes
ros2 topic list                         # all active topics
ros2 topic echo /amms/detections        # live object detections
ros2 topic echo /amms/task_state        # current FSM state
ros2 topic echo /odometry/filtered      # filtered odometry
ros2 action list                        # Nav2 action servers
```

### Other Useful Launches

```bash
# Navigation only (no arm):
ros2 launch k12_pick_place amms_navigation.launch.py \
  map:=/home/nabhan/try_12/src/k12_pick_place/maps/pharma.yaml

# SLAM mapping (create a new map):
ros2 launch k12_pick_place amms_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/my_new_map

# Arm demo only:
ros2 launch k12_pick_place pick_place_demo.launch.py
```

---

## 8. Tuning Parameters Reference

### Increase Mobile Base Speed

Edit `config/nav2_amms.yaml`:

```yaml
FollowPath:                    # DWB local planner section
  max_vel_x: 0.35              # forward speed m/s   (default: 0.21)
  max_speed_xy: 0.35           # must match max_vel_x (default: 0.21)
  max_vel_theta: 0.80          # turn speed rad/s    (default: 0.40)
  acc_lim_x: 0.50              # acceleration m/s²   (default: 0.24)
  acc_lim_theta: 1.00          # angular accel        (default: 0.50)

velocity_smoother:
  ros__parameters:
    max_velocity: [0.35, 0.0, 0.80]   # (default: [0.225, 0.0, 0.525])
    max_accel:    [0.50, 0.0, 1.00]   # (default: [0.30,  0.0, 0.45])
```

### Increase Arm Speed

Edit `config/pick_place_service.yaml`:

```yaml
k12_pick_place_commander:
  ros__parameters:
    place_retreat_step_duration_sec: 0.30   # lower = faster (default: 0.50)
```

For pick/grasp/lift trajectory speeds, find the `_move_joints(positions, duration_sec, ...)` calls in `pick_place_commander.py` and reduce the `duration_sec` argument (e.g., `3.0 → 1.5 s`).

### Increase Docking & Backup Speeds

Edit `config/amms_waypoints.yaml`:

```yaml
amms_task_manager:
  ros__parameters:
    place_final_approach_speed: 0.12   # shelf docking m/s     (default: 0.08)
    post_pick_backup_speed:     0.10   # backup after pick m/s  (default: 0.06)
    post_place_backup_speed:    0.15   # backup after place m/s (default: 0.10)
```

### Gripper Width Constraint

The URDF hard limit (in `gripper.xacro`) is `finger_travel = 0.0394875 m`.
**Never** set `open_gripper_width` above this:

```yaml
k12_pick_place_commander:
  ros__parameters:
    open_gripper_width: 0.039        # must be ≤ 0.0394875
    closed_gripper_width: 0.0014625
```

### Waypoint Position Tuning

To change where the robot stops for pick/place, edit the waypoint coordinates in `config/amms_waypoints.yaml`:

```yaml
amms_waypoint_manager:
  ros__parameters:
    waypoint_pick_station: [0.85, 0.0, 0.0]     # [x, y, yaw_radians]
    waypoint_place_station: [-4.0, 2.5, 3.14159] # facing -x direction (π rad)

amms_task_manager:
  ros__parameters:
    waypoint_pick_station: [0.85, 0.0, 0.0]      # same coordinates, both sections
    waypoint_place_station: [-4.0, 2.5, 3.14159]
```

> Both `amms_waypoint_manager` and `amms_task_manager` sections must have matching waypoint values.
