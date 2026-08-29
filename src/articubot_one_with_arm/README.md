# articubot_one — Fixed ROS 2 Package

4-wheel skid-steer robot for ROS 2 Humble + Ignition Gazebo (Fortress/Garden/Harmonic).  
All bugs from the original package have been corrected. The robot can now drive, turn, map, localize, and navigate autonomously.

---

## Table of Contents

1. [Bug Fix Summary](#bug-fix-summary)
2. [Dependencies](#dependencies)
3. [Workspace Setup & Build](#workspace-setup--build)
4. [Launch Simulation](#launch-simulation)
5. [Verify Controllers](#verify-controllers)
6. [Verify TF Tree](#verify-tf-tree)
7. [Verify LiDAR](#verify-lidar)
8. [Run SLAM (Build a Map)](#run-slam-build-a-map)
9. [Save the Map](#save-the-map)
10. [Launch Localization (AMCL)](#launch-localization-amcl)
11. [Launch Nav2](#launch-nav2)
12. [Send Goals from RViz](#send-goals-from-rviz)
13. [File Change Log](#file-change-log)
14. [Troubleshooting Guide](#troubleshooting-guide)

---

## Bug Fix Summary

### Bug 1 — Wheel Friction Direction (Primary Turning Failure)

**File:** `description/four_wheel.gazebo`

| Parameter | Original (broken) | Fixed |
|-----------|-------------------|-------|
| `fdir1` | `0 1 0` (lateral) | `1 0 0` (forward) |
| `mu1` | `1.0` | `10.0` |
| `mu2` | `0.02` | `0.1` |

**Why it broke:** `fdir1` defines the direction of mu1 (primary friction). Setting it to `0 1 0` (Y-axis / lateral) means the *high* friction was resisting lateral motion. For a skid-steer robot, turning *requires* lateral slip. The wheels had to push sideways to turn but the high lateral friction prevented it. The robot could barely rotate.

**Fix:** `fdir1 = 1 0 0` means mu1 acts forward (X-axis, rolling direction). High mu1=10.0 ensures forward grip. Low mu2=0.1 allows enough lateral slip for skid-steer turning. Added `kp=1e6, kd=1.0` for stable contact stiffness.

---

### Bug 2 — Wrong wheel_separation in Controller

**File:** `config/my_controllers.yaml`

| Parameter | Original | Fixed |
|-----------|----------|-------|
| `wheel_separation` | `0.40 m` | `0.18 m` |

**Why it broke:** The physical separation between left and right wheel centres, from the URDF joint origins: left wheels at Y=+0.09 m, right wheels at Y=-0.09 m → separation = 0.18 m. Using 0.40 m made the controller compute angular velocities more than 2× too small, causing the robot to drift badly when turning and producing completely wrong odometry.

---

### Bug 3 — Mesh Wheel Collision Geometry

**File:** `description/four_wheel.xacro`

**Original:** Wheel collision used mesh geometry (same as visual). Mesh collisions create irregular contact normals, jitter, and instability in physics engines.

**Fix:** All four wheel collision blocks replaced with `<cylinder radius="0.04" length="0.03" />` (radius matches wheel radius, length matches wheel width). Cylinder is rotated 90° (`rpy="1.5707963 0 0"`) so its axis aligns with the wheel spin axis (Y). Base link collision also replaced with a box primitive.

---

### Bug 4 — SLAM Configuration (Wrong Mode + Hardcoded Path)

**Files:** `config/mapper_params_online_async.yaml` (fixed), `config/mapper_params_localization.yaml` (new)

**Original issues:**
- `mode: localization` in the SLAM config used for mapping → SLAM would refuse to build a new map
- `map_file_name: /home/nabhan/ros2_ws/mapserial` — machine-specific hardcoded path that breaks on any other machine

**Fix:** Split into two separate YAML files:
- `mapper_params_online_async.yaml` → `mode: mapping`, no `map_file_name`
- `mapper_params_localization.yaml` → `mode: localization`, `map_file_name: ""` (pass at launch)

---

### Bug 5 — EKF / Odometry Frame Conflict

**File:** `config/ekf.yaml`

**Original:** `base_link_frame: base_link`. The diff_drive_controller with `enable_odom_tf: true` would publish `odom → base_link`. The EKF would also try to publish `odom → base_link`. Two nodes publishing the same TF relationship creates a "multiple parents" TF error.

**Fix:** `base_link_frame: base_footprint`. The controller uses `base_frame_id: base_footprint`. The EKF (`enable_odom_tf: true`) publishes `odom → base_footprint`. The controller has `enable_odom_tf: false` so it only publishes odometry data (not TF). Single, consistent TF chain: `map → odom → base_footprint → base_link → wheels`.

---

### Bug 6 — Localization Launch Hardcoded Map Path

**File:** `launch/localization_launch.py`

**Original:** Default map was `turtlebot3_world.yaml` bundled in the package — a TurtleBot3 map, not the robot's own map. Users had no guidance to pass their saved map.

**Fix:** Default map is now `""` (empty string). The user must supply `map:=/path/to/saved_map.yaml` at launch. This forces the correct workflow.

---

## Dependencies

### System packages (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt update && sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-laser-filters \
  ros-humble-rviz2 \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-smoother \
  ros-humble-nav2-waypoint-follower
```

---

## Workspace Setup & Build

```bash
# Create workspace (skip if it already exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy or clone this package into src/
cp -r /path/to/articubot_one .

# Install any missing dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --packages-select articubot_one

# Source
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc` for convenience:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Launch Simulation

```bash
# Terminal 1 — launch everything (Gazebo + ROS 2 control + EKF)
ros2 launch articubot_one launch_sim.launch.py

# Optional: use a specific world
ros2 launch articubot_one launch_sim.launch.py world:=$(ros2 pkg prefix articubot_one)/share/articubot_one/worlds/cafe.world
```

Wait until you see:
```
[spawner-...]: Loaded diff_cont
[spawner-...]: Loaded joint_broad
```

---

## Verify Controllers

```bash
# Should list: diff_cont (active), joint_broad (active)
ros2 control list_controllers

# Test driving forward
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.3}, "angular": {"z": 0.0}}'

# Test turning left
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.0}, "angular": {"z": 0.5}}'

# Test turning right
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.0}, "angular": {"z": -0.5}}'
```

---

## Verify TF Tree

```bash
# Check TF tree — should be continuous with no errors
ros2 run tf2_tools view_frames

# Expected chain:
# map → odom → base_footprint → base_link → [camera_1, lidar_1, ultra_front_1, ultra_rear_1, wheels]

# Live TF echo (odom → base_footprint, published by EKF)
ros2 run tf2_ros tf2_echo odom base_footprint

# Check for TF errors
ros2 topic echo /tf --no-arr 2>&1 | grep -i error
```

---

## Verify LiDAR

```bash
# Check scan is being published
ros2 topic hz /scan
# Expected: ~10 Hz

ros2 topic echo /scan --no-arr

# In RViz2: Add → By Topic → /scan → LaserScan
# Fixed frame: base_footprint or odom
```

---

## Run SLAM (Build a Map)

```bash
# Terminal 1 (already running): Simulation
ros2 launch articubot_one launch_sim.launch.py

# Terminal 2: SLAM Toolbox in mapping mode
ros2 launch articubot_one online_async_launch.py use_sim_time:=true

# Terminal 3: RViz2 with SLAM panel
rviz2 -d $(ros2 pkg prefix articubot_one)/share/articubot_one/config/map.rviz

# Terminal 4: Teleop (keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Drive the robot around to build the map. The map will appear in RViz2 on the `/map` topic.

---

## Save the Map

```bash
# Create a maps directory
mkdir -p ~/ros2_ws/maps

# Save the map (while SLAM is still running)
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/maps/my_map \
  --ros-args -p use_sim_time:=true

# This creates:
#   ~/ros2_ws/maps/my_map.pgm   (grayscale occupancy image)
#   ~/ros2_ws/maps/my_map.yaml  (map metadata)

# Alternatively, use the SLAM Toolbox service to serialize:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{'name': {'data': '/home/$USER/ros2_ws/maps/my_slam_map'}}"
```

---

## Launch Localization (AMCL)

Stop SLAM first. Then:

```bash
# Terminal 1: Simulation (if not already running)
ros2 launch articubot_one launch_sim.launch.py

# Terminal 2: Localization with your saved map
ros2 launch articubot_one localization_launch.py \
  map:=/home/$USER/ros2_ws/maps/my_map.yaml \
  use_sim_time:=true

# Terminal 3: RViz2
rviz2 -d $(ros2 pkg prefix articubot_one)/share/articubot_one/config/map.rviz
```

In RViz2, use **2D Pose Estimate** to set the robot's initial position on the map. AMCL will then localize the robot.

---

## Launch Nav2

```bash
# Terminal 1: Simulation
ros2 launch articubot_one launch_sim.launch.py

# Terminal 2: Localization
ros2 launch articubot_one localization_launch.py \
  map:=/home/$USER/ros2_ws/maps/my_map.yaml \
  use_sim_time:=true

# Terminal 3: Navigation
ros2 launch articubot_one navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true

# Terminal 4: RViz2
rviz2 -d $(ros2 pkg prefix articubot_one)/share/articubot_one/config/map.rviz
```

---

## Send Goals from RViz

1. Open RViz2 with the map config (above).
2. Wait until AMCL particles converge on the robot's position.
3. If needed, click **2D Pose Estimate** and click on the map where the robot is.
4. Click **2D Goal Pose** (Nav2 Goal in Humble) in the toolbar.
5. Click and drag on the map to set a goal position and orientation.
6. Nav2 will plan a path and the robot will navigate autonomously.

**Programmatic goal (terminal):**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## File Change Log

| File | Status | Changes |
|------|--------|---------|
| `description/four_wheel.xacro` | **MODIFIED** | Wheel collisions → cylinder primitives; base_link collision → box; inertia cleanup |
| `description/four_wheel.gazebo` | **MODIFIED** | `fdir1` 0→1 0→0; `mu1` 1.0→10.0; `mu2` 0.02→0.1; added `kp/kd`; DiffDrive wheel_separation corrected |
| `description/four_wheel.trans` | Kept | No changes needed |
| `description/four_wheel_ros2_control.xacro` | **DELETED** | Duplicate of four_wheel.trans — removed to avoid confusion |
| `config/my_controllers.yaml` | **MODIFIED** | `wheel_separation` 0.40→0.18; comments updated |
| `config/ekf.yaml` | **MODIFIED** | `base_link_frame` → `base_footprint`; `world_frame` → `odom` |
| `config/mapper_params_online_async.yaml` | **MODIFIED** | `mode` → `mapping`; removed hardcoded `map_file_name`; removed `map_start_at_dock` |
| `config/mapper_params_localization.yaml` | **NEW** | Localization-mode SLAM config, no hardcoded paths |
| `launch/launch_sim.launch.py` | **MODIFIED** | Default world → obstacles.world; minor cleanup |
| `launch/rsp.launch.py` | Kept | Already correct |
| `launch/online_async_launch.py` | **MODIFIED** | Default params → mapping config |
| `launch/localization_launch.py` | **MODIFIED** | Default map → `""` (user must supply) |
| `package.xml` | **MODIFIED** | Full dependency list added |
| `CMakeLists.txt` | Kept | Already correct |

---

## Troubleshooting Guide

### Robot does not move

```bash
# Check controller state
ros2 control list_controllers

# If diff_cont is not 'active':
ros2 control set_controller_state diff_cont active

# Check if cmd_vel is reaching the controller
ros2 topic echo /diff_cont/cmd_vel_unstamped
```

### Robot drives but cannot turn

Verify the friction values in `four_wheel.gazebo` were saved correctly:
```bash
grep -A3 "mu1\|fdir1" $(ros2 pkg prefix articubot_one)/share/articubot_one/description/four_wheel.gazebo
# Should show: mu1=10.0, mu2=0.1, fdir1=1 0 0
```

### TF errors / "could not find parent frame"

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf and check for disconnected subtrees

# Check for duplicate TF publishers:
ros2 topic echo /tf | grep -E "odom|base"
# Should see only one publisher for odom→base_footprint (the EKF)
```

### No /scan data

```bash
# Check Gazebo sensor is publishing
ros2 topic list | grep scan
# Should show: /scan, /ultra_front/scan, /ultra_rear/scan

# Check gz_bridge is running
ros2 node list | grep parameter_bridge

# Verify bridge config
cat $(ros2 pkg prefix articubot_one)/share/articubot_one/config/gz_bridge.yaml
```

### SLAM does not build a map

Verify the SLAM config is in mapping mode:
```bash
grep "^    mode:" $(ros2 pkg prefix articubot_one)/share/articubot_one/config/mapper_params_online_async.yaml
# Should print:     mode: mapping
```

Also ensure `/scan` is publishing before starting SLAM:
```bash
ros2 topic hz /scan  # must show ~10 Hz
```

### SLAM localization fails to load map

The localization YAML requires you to pass the map path explicitly:
```bash
ros2 launch articubot_one online_async_launch.py \
  params_file:=$(ros2 pkg prefix articubot_one)/share/articubot_one/config/mapper_params_localization.yaml \
  use_sim_time:=true
# Then edit mapper_params_localization.yaml map_file_name to your map path,
# OR use localization_launch.py with AMCL instead.
```

### Nav2 fails to start

```bash
# Check all lifecycle nodes are active
ros2 lifecycle list /bt_navigator
ros2 lifecycle list /controller_server

# Verify /odometry/filtered is publishing (EKF output)
ros2 topic hz /odometry/filtered  # should be ~50 Hz

# Check /map is available
ros2 topic hz /map
```

### Odometry drifts badly when turning

Double-check wheel_separation in the controllers config:
```bash
grep wheel_separation $(ros2 pkg prefix articubot_one)/share/articubot_one/config/my_controllers.yaml
# Must be: wheel_separation: 0.18
```

If you have added encoders to the real robot, measure the actual wheel separation physically and update this value.

### Gazebo physics instability (robot bouncing or flipping)

The cylinder wheel collision geometry should be stable. If you still see instability:

```bash
# Check kp/kd in four_wheel.gazebo
grep -A2 "kp\|kd" $(ros2 pkg prefix articubot_one)/share/articubot_one/description/four_wheel.gazebo
# kp=1e6, kd=1.0

# Try reducing simulation step size in your world file:
# <physics type="ode">
#   <max_step_size>0.001</max_step_size>
#   <real_time_factor>1</real_time_factor>
# </physics>
```

---

## Quick Command Reference

```bash
# Build
colcon build --symlink-install --packages-select articubot_one && source install/setup.bash

# Simulation
ros2 launch articubot_one launch_sim.launch.py

# Mapping
ros2 launch articubot_one online_async_launch.py use_sim_time:=true

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map --ros-args -p use_sim_time:=true

# Localization
ros2 launch articubot_one localization_launch.py map:=~/ros2_ws/maps/my_map.yaml use_sim_time:=true

# Navigation
ros2 launch articubot_one navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true

# RViz
rviz2 -d $(ros2 pkg prefix articubot_one)/share/articubot_one/config/map.rviz

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
