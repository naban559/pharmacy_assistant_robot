# Session Modifications & Revert Guide

This document contains a complete inventory of every file modified, added, or configured during this session, including the **BEFORE** and **AFTER** states, technical rationale, and exact step-by-step instructions to **undo/revert all changes** back to the initial starting state.

---

## 1. Inventory of Session Modifications

| # | File / Component | Key Changes in Session | Before State | After State |
|---|---|---|---|---|
| 1 | [og.world](file:///home/nabhan/try_15/src/articubot_one_with_arm/worlds/og.world) | Cube sizes, inertia, and red cube pickup location | $7\text{ cm}$ cubes (`0.07m`), $I_{xx} = 1.63 \times 10^{-5}$, Red cube at $(-3.00, 2.62, 0.135)$ | $2.5\text{ cm}$ cubes (`0.025m`), $I_{xx} = 2.08 \times 10^{-6}$, Red cube at $(-2.88, 2.62, 0.1125)$ |
| 2 | [four_wheel.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.xacro) | Camera tilt angle (`camera_1` joint pitch down) | `rpy="0 0 0"` (horizontal $0^\circ$) | `rpy="0 0.2618 0"` ($15^\circ$ pitch down) |
| 3 | [arm.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm.xacro) | Gripper finger thickness & palm height | Finger height `0.035m` ($3.5\text{ cm}$), Palm height `0.025m` | Finger height `0.010m` ($1.0\text{ cm}$), Palm height `0.015m` |
| 4 | [moveit_pick_place_commander.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/moveit_pick_place_commander.py) | Commander patch script, $+5\text{ cm}$ Z pre-grasp, zero wrist roll lock | Duplicate `_radial_unit`, missing None guard, horizontal pre-grasp, unconstrained wrist roll | Single `_radial_unit`, None guard, $+5\text{ cm}$ Z vertical pre-grasp, forced zero wrist roll (`joint_5 = 0.0`) |
| 5 | [pick_place_commander.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/pick_place_commander.py) | $+5\text{ cm}$ Z pre-grasp, zero wrist roll lock, parameters | Horizontal pre-grasp, center offset `0.02925m` | $+5\text{ cm}$ Z vertical pre-grasp, center offset `0.0m`, forced zero wrist roll (`joint_5 = 0.0`) |
| 6 | [object_detector.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/object_detector.py) | Perception thresholds & topic sync slop | `min_area_px: 60.0`, `sync_slop_sec: 0.05` | `min_area_px: 10.0`, `confidence_area_px: 100.0`, `sync_slop_sec: 0.50` |
| 7 | [amms_waypoints.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/amms_waypoints.yaml) | Pickup waypoints X and Y coordinates | `pickup_point_1_red`: `[-3.0, 2.30, 1.570796]` (Y=2.30 for all) | `pickup_point_1_red`: `[-2.88, 2.15, 1.570796]` (Y=2.15 for all) |
| 8 | [moveit_nav_pick_place.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/moveit_nav_pick_place.yaml) | Geometric offsets & confidence threshold | `approach_distance: 0.09871875`, `grasp_surface_to_center_offset: 0.02925`, `min_confidence: 0.25` | `approach_distance: 0.05`, `grasp_surface_to_center_offset: 0.0`, `min_confidence: 0.15` |
| 9 | [pick_place_service.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/pick_place_service.yaml) | Geometric offsets & confidence threshold | `approach_distance: 0.09871875`, `grasp_surface_to_center_offset: 0.02925`, `min_confidence: 0.25` | `approach_distance: 0.05`, `grasp_surface_to_center_offset: 0.0`, `min_confidence: 0.15` |
| 10 | [nav2_amms.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/nav2_amms.yaml) | Local & global costmap inflation radius | `inflation_radius: 0.02` ($2\text{ cm}$) | `inflation_radius: 0.15` ($15\text{ cm}$ safety buffer) |

---

## 2. Detailed Breakdown of Changes

### 1. Gazebo World Model ([og.world](file:///home/nabhan/try_15/src/articubot_one_with_arm/worlds/og.world))
- **Cube Resizing**: `cube_red`, `cube_green`, and `cube_blue` dimensions reduced from $7\text{ cm}$ to **$2.5\text{ cm}$** (`0.025m 0.025m 0.025m`).
- **Cube Inertia**: Updated to $I_{xx} = I_{yy} = I_{zz} = 2.08 \times 10^{-6}\text{ kg}\cdot\text{m}^2$.
- **Red Cube Location**: Moved `cube_red` from table center ($X = -3.00\text{ m}$) to the right side of pickup table 1 at **$X = -2.88\text{ m}, Y = 2.62\text{ m}, Z = 0.1125\text{ m}$**.

### 2. Camera Field-of-View Tilt ([four_wheel.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.xacro))
- **Camera Pitch**: Changed `Rigid 1` joint origin from `rpy="0 0 0"` to **`rpy="0 0.2618 0"` ($15^\circ$ pitch down)**.
- **Rationale**: Directs the optical axis toward the pickup table surface so $2.5\text{ cm}$ cubes are centered in the RGB-D view.

### 3. Gripper Finger Geometry ([arm.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm.xacro))
- **Finger Thickness**: Reduced `k12_gripper_finger_height` from `0.035m` ($3.5\text{ cm}$) to **`0.010m` ($1.0\text{ cm}$ thickness)**.
- **Palm Height**: Reduced `k12_gripper_palm_height` to `0.015m`.

### 4. MoveIt Pick & Place Commander ([moveit_pick_place_commander.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/moveit_pick_place_commander.py))
- **Commander Patch**: Applied user fix script (removed duplicate `_radial_unit`, added `target is None` guard, added startup home move `_go_home_once`).
- **Top-Down Vertical Pre-Grasp**: `_pick_plan_for_grasp` sets `pre_grasp = (grasp[0], grasp[1], grasp[2] + approach)` ($+5\text{ cm}$ Z offset).
- **Execution Trajectory**: Added `self._sleep(0.5)` pause at pre-grasp pose before descending straight down to grasp.
- **Zero Wrist Roll Lock**: Added `JointConstraint` for `arm_wrist_roll_joint = 0.0` in MoveGroup constraints and forced `positions[4] = 0.0` in `_move_joints`.

### 5. Perception Pipeline ([object_detector.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/object_detector.py))
- **Detection Area**: Lowered `min_area_px` to `10.0` and `confidence_area_px` to `100.0`.
- **Topic Sync Slop**: Increased `sync_slop_sec` from `0.05` to `0.50` s to resolve frame drops in Gazebo-ROS2 bridge.

### 6. Navigation Waypoints & Costmap Safety ([amms_waypoints.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/amms_waypoints.yaml) & [nav2_amms.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/nav2_amms.yaml))
- **Waypoint 1 Alignment**: `waypoint_pickup_point_1_red` updated to `[-2.88, 2.15, 1.570796]`.
- **Table Collision Gap**: Pickup waypoints Y shifted from `2.30` to `2.15` (leaving a $21\text{ cm}$ safety gap in front of table edge).
- **Costmap Inflation**: Local and global costmap `inflation_radius` increased from `0.02m` to **`0.15m`**.

---

## 3. How to Undo / Revert Everything Back to Initial State

To undo all changes made during this session and restore the workspace back to its original state, perform either of the two methods below.

### Method A: Automated Revert Command (Recommended)

Run the following shell commands in your terminal from the repository root:

```bash
cd /home/nabhan/try_15/src

# 1. Restore all tracked modified files to original git commit
git restore .

# 2. Remove temporary backup file created by fix_commander script
rm -f k12_pick_place/k12_pick_place/moveit_pick_place_commander.py.bak

# 3. Rebuild packages to clean up workspace
colcon build --packages-select articubot_one k12_pick_place k12_moveit_config
```

---

### Method B: Manual File-by-File Revert Instructions

If you prefer to manually revert individual files, apply the following line edits:

#### 1. Revert [og.world](file:///home/nabhan/try_15/src/articubot_one_with_arm/worlds/og.world)
- Change `<pose>-2.88 2.62 0.1125 0 0 0</pose>` for `cube_red` back to `<pose>-3.00 2.62 0.135 0 0 0</pose>`.
- Change `<pose>0 2.62 0.1125 0 0 0</pose>` for `cube_green` back to `<pose>0 2.62 0.135 0 0 0</pose>`.
- Change `<pose>3.00 2.62 0.1125 0 0 0</pose>` for `cube_blue` back to `<pose>3.00 2.62 0.135 0 0 0</pose>`.
- Change `<size>0.025 0.025 0.025</size>` back to `<size>0.07 0.07 0.07</size>`.
- Change inertia `0.00000208` back to `0.00001633`.

#### 2. Revert [four_wheel.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.xacro)
- Line 237: Change `<origin xyz="0.195 0.0 0.13875" rpy="0 0.2618 0"/>` back to `<origin xyz="0.195 0.0 0.13875" rpy="0 0 0"/>`.

#### 3. Revert [arm.xacro](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm.xacro)
- Line 31: Change `k12_gripper_palm_height` back to `0.025`.
- Line 34: Change `k12_gripper_finger_height` back to `0.035`.

#### 4. Revert [amms_waypoints.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/amms_waypoints.yaml)
- Lines 7 & 36: Change `waypoint_pickup_point_1_red` back to `[-3.0, 2.30, 1.570796]`.
- Lines 8 & 37: Change `waypoint_pickup_point_2_green` back to `[0.0, 2.30, 1.570796]`.
- Lines 9 & 38: Change `waypoint_pickup_point_3_blue` back to `[3.0, 2.30, 1.570796]`.

#### 5. Revert Configuration YAML Files ([moveit_nav_pick_place.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/moveit_nav_pick_place.yaml) & [pick_place_service.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/pick_place_service.yaml))
- Change `approach_distance` back to `0.09871875`.
- Change `lift_height` back to `0.1184625`.
- Change `grasp_z_bias` back to `-0.0073125`.
- Change `grasp_surface_to_center_offset` back to `0.02925`.
- Change `min_detection_confidence` back to `0.25`.
- Change `closed_gripper_width` back to `0.007`.

#### 6. Revert [nav2_amms.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/nav2_amms.yaml)
- Lines 288 & 327: Change local & global costmap `inflation_radius` back to `0.02`.

---

## 4. Rebuild Verification

After executing either revert method, verify and compile the workspace using:

```bash
colcon build --packages-select articubot_one k12_pick_place k12_moveit_config
```
