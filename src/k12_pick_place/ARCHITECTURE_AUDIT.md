# ARCHITECTURE_AUDIT.md

**STATIC AUDIT ONLY — RUNTIME NOT EXECUTED**

No ROS 2/Gazebo runtime was available. All claims are from direct source code inspection. Runtime evidence commands are provided where needed.

---

## 1. Active Launch Graph

```
amms_nav_pick_place.launch.py (ROOT)
│
├── amms_navigation.launch.py
│   ├── amms_gazebo.launch.py (from articubot_one)
│   │   ├── rsp.launch.py → robot_state_publisher
│   │   ├── gz sim (ExecuteProcess) → og.world
│   │   ├── twist_mux
│   │   ├── spawn_entity (four_wheel)
│   │   ├── controller spawners: diff_cont, joint_broad, arm_position_controller, gripper_position_controller
│   │   ├── parameter_bridge (gz_bridge.yaml)
│   │   └── image_bridge (depth camera topics)
│   ├── nav2_bringup/localization_launch.py
│   │   ├── map_server (nav2_amms.yaml + og.yaml)
│   │   ├── amcl (nav2_amms.yaml)
│   │   └── lifecycle_manager_localization
│   ├── nav2_bringup/navigation_launch.py
│   │   ├── controller_server (nav2_amms.yaml)
│   │   ├── smoother_server
│   │   ├── planner_server
│   │   ├── behavior_server
│   │   ├── bt_navigator
│   │   ├── waypoint_follower
│   │   ├── velocity_smoother
│   │   └── lifecycle_manager_navigation
│   ├── pointcloud_to_laserscan (pointcloud_to_laserscan.yaml)
│   ├── ekf_filter_node (ekf_odom.yaml)
│   ├── amms_waypoint_manager (amms_waypoints.yaml)
│   └── rviz2
│
├── amms_scene_spawner (inline params)
├── k12_object_detector (pick_place_service.yaml)
├── k12_pick_place_commander (pick_place_service.yaml) ← ACTIVE
├── k12_moveit_pick_place_commander ← INACTIVE (use_moveit_commander:=false)
├── move_group ← INACTIVE (use_moveit_commander:=false)
├── amms_readiness_gate (inline params)
└── amms_task_manager (amms_waypoints.yaml)
```

**Critical note:** `amms_navigation.launch.py` imports from `nav2_bringup`, NOT from `articubot_one_with_arm/launch/`. The files `articubot_one_with_arm/launch/navigation_launch.py` and `localization_launch.py` are NOT in the active graph.

---

## 2. Active Nodes (30 ROS nodes + 1 Gazebo process)

| # | Node Name | Package | Executable | Parameter File | Status |
|---|-----------|---------|------------|----------------|--------|
| 1 | robot_state_publisher | robot_state_publisher | robot_state_publisher | inline (xacro) | ACTIVE |
| 2 | gz sim | (ExecuteProcess) | gz sim | — | ACTIVE |
| 3 | twist_mux | twist_mux | twist_mux | articubot_one/config/twist_mux.yaml | ACTIVE |
| 4 | pointcloud_to_laserscan | k12_pick_place | cloud_to_scan | pointcloud_to_laserscan.yaml | ACTIVE |
| 5 | ekf_filter_node | robot_localization | ekf_node | ekf_odom.yaml | ACTIVE |
| 6 | map_server | nav2_map_server | map_server | nav2_amms.yaml + og.yaml | ACTIVE |
| 7 | amcl | nav2_amcl | amcl | nav2_amms.yaml | ACTIVE |
| 8 | lifecycle_manager_localization | nav2_lifecycle_manager | lifecycle_manager | inline | ACTIVE |
| 9 | controller_server | nav2_controller | controller_server | nav2_amms.yaml | ACTIVE |
| 10 | smoother_server | nav2_smoother | smoother_server | nav2_amms.yaml | ACTIVE |
| 11 | planner_server | nav2_planner | planner_server | nav2_amms.yaml | ACTIVE |
| 12 | behavior_server | nav2_behaviors | behavior_server | nav2_amms.yaml | ACTIVE |
| 13 | bt_navigator | nav2_bt_navigator | bt_navigator | nav2_amms.yaml | ACTIVE |
| 14 | waypoint_follower | nav2_waypoint_follower | waypoint_follower | nav2_amms.yaml | ACTIVE |
| 15 | velocity_smoother | nav2_velocity_smoother | velocity_smoother | nav2_amms.yaml | ACTIVE |
| 16 | lifecycle_manager_navigation | nav2_lifecycle_manager | lifecycle_manager | inline | ACTIVE |
| 17 | amms_waypoint_manager | k12_pick_place | waypoint_manager | amms_waypoints.yaml | ACTIVE |
| 18 | rviz2 | rviz2 | rviz2 | amms_nav_pick_place.rviz | ACTIVE |
| 19 | amms_scene_spawner | k12_pick_place | scene_spawner | inline | ACTIVE |
| 20 | k12_object_detector | k12_pick_place | object_detector | pick_place_service.yaml | ACTIVE |
| 21 | k12_pick_place_commander | k12_pick_place | pick_place_commander | pick_place_service.yaml | ACTIVE |
| 22 | amms_readiness_gate | k12_pick_place | readiness_gate | inline | ACTIVE |
| 23 | amms_task_manager | k12_pick_place | amms_task_manager | amms_waypoints.yaml | ACTIVE |
| 24-27 | controller spawners | controller_manager | spawner | — | ACTIVE |
| 28 | parameter_bridge | ros_gz_bridge | parameter_bridge | gz_bridge.yaml | ACTIVE |
| 29 | image_bridge | ros_gz_image | image_bridge | — | ACTIVE |
| 30 | spawn_entity | ros_gz_sim | create | — | ACTIVE |

**Inactive nodes (conditional):**
- `k12_moveit_pick_place_commander` — `use_moveit_commander:=false`
- `move_group` — `use_moveit_commander:=false`

---

## 3. Active Topics (Data Flow)

```
/camera/depth_camera/image        → image_bridge → k12_object_detector (rgb)
/camera/depth_camera/depth_image  → image_bridge → k12_object_detector (depth)
/camera/depth_camera/camera_info  → parameter_bridge → k12_object_detector (intrinsics)

k12_object_detector → /amms/detections (String, JSON) → k12_pick_place_commander
                                                      → amms_task_manager
k12_object_detector → /amms/detected_object_poses_camera (PoseArray) → rviz2

amms_task_manager → /amms/pick_target (String, JSON) → k12_pick_place_commander
amms_task_manager → /amms/task_state (String) → rviz2 / operator

/diff_cont/odom → ekf_filter_node → /odometry/filtered → bt_navigator, controller_server
/imu → ekf_filter_node

/velodyne_points → pointcloud_to_laserscan → /scan → amcl, controller_server, behavior_server

/controller_manager/list_controllers → readiness_gate
```

---

## 4. Active Services/Actions

**Services:**
- `/amms/pick_object` (std_srvs/Trigger) — task_manager → commander
- `/amms/place_object` (std_srvs/Trigger) — task_manager → commander
- `/amms/fold_arm` (std_srvs/Trigger) — task_manager → commander
- `/set_entity_state`, `/get_entity_state`, `/get_link_properties`, `/set_link_properties` — gazebo_msgs (Gazebo Classic, NOT available under Ignition)

**Actions:**
- `navigate_to_pose` (nav2_msgs/NavigateToPose) — task_manager → bt_navigator
- `drive_on_heading` (nav2_msgs/DriveOnHeading) — task_manager → behavior_server
- `backup` (nav2_msgs/BackUp) — task_manager → behavior_server
- `/arm_position_controller/follow_joint_trajectory` (FollowJointTrajectory) — commander → controller_manager

---

## 5. TF Frames

```
map → odom → base_link → arm_base_link → end_effector
  ↑         ↑              ↑
  AMCL      EKF            URDF (robot_state_publisher)
```

- `map → odom`: Published by AMCL
- `odom → base_link`: Published by EKF (`ekf_odom.yaml`: `base_link_frame: base_footprint`, `world_frame: odom`)
- `base_link → arm_base_link`: Published by robot_state_publisher (URDF static transform)
- `base_link → base_footprint`: Note: EKF publishes `base_footprint`, but Nav2 and task_manager reference `base_link`. The URDF defines the relationship.

**Frame mismatch warning:** EKF config says `base_link_frame: base_footprint` but Nav2 and task_manager use `base_link`. The URDF likely has `base_footprint → base_link` as a fixed joint. This works if robot_state_publisher publishes the full chain, but it means EKF publishes `odom → base_footprint`, and the static URDF publishes `base_footprint → base_link`. Nav2 sees `map → odom → base_footprint → base_link`.

---

## 6. Current A/B/C Command Flow

**Source:** `amms_task_manager.py` lines 339-592

```
_ABC_WAYPOINTS = {
    'A': 'pickup_point_1_red',   # [-3.0, 2.10, π/2]
    'B': 'pickup_point_2_green', # [0.0, 2.10, π/2]
    'C': 'pickup_point_3_blue',  # [3.0, 2.10, π/2]
}
```

**Flow:**
1. `_mission_loop()` (line 345): background thread, waits for readiness, loops
2. Prints `Robot ready. Enter target [A/B/C]: `, reads `input()`
3. Validates against `_ABC_WAYPOINTS` keys
4. Calls `_run_abc_mission(target_char)` (line 382)

**`_run_abc_mission` phases (lines 382-592):**
1. Capture initial pose via TF
2. Fold arm
3. Navigate to pickup waypoint
4. Stop, verify localization
5. `_acquire_reachable_object()` — dynamic detection
6. Validate, confirm fresh detection
7. Pick with retries
8. Log "grasp verified" (NOT physical verification)
9. Fold arm, backup
10. Navigate to `place_approach` waypoint
11. Navigate to `place_station` waypoint
12. `_ensure_place_reachable()` — reposition if needed
13. Verify localization
14. Place object
15. Log "release verified" (NOT physical verification)
16. Fold arm, backup
17. Navigate to `place_clear` waypoint
18. Return to initial pose
19. Return to `WAIT_FOR_COMMAND`

---

## 7. Current Dynamic Detection Flow

**Source:** `object_detector.py` (344 lines)

1. Subscribes to synchronized RGB + Depth + CameraInfo (`ApproximateTimeSynchronizer`)
2. HSV color thresholding: red (0-10, 170-180), green (40-85), blue (95-130)
3. Morphological open/close
4. Contour detection, area filtering (`min_area_px: 60`)
5. Centroid from moments
6. Depth via median filter in 5px radius patch
7. 3D position via pinhole back-projection using live CameraInfo intrinsics
8. Shape classification: red→cube, green→cylinder, blue→rectangular_block (hardcoded by color, not contour analysis for known colors)
9. Confidence scoring: `area_score * depth_score`
10. Publishes JSON to `/amms/detections`

**No object XYZ is hardcoded in the detector.** All positions come from live RGB-D perception.

---

## 8. Current Pick Flow

**Source:** `pick_place_commander.py` `_execute_pick()` (line 1704)

1. Task manager publishes pick target to `/amms/pick_target`
2. Commander receives via `_on_pick_target()` → stores in `latest_external_pick_target`
3. On `/amms/pick_object` service call:
   - `_wait_for_service_pick_target()` waits for stable detection
   - `_execute_pick(target)`:
     a. Open gripper
     b. Move to home joints
     c. Move to pre-grasp (approach_distance behind grasp point)
     d. Move to grasp point (from detected object)
     e. Close gripper (velocity-based stall detection)
     f. `_attach_object()` — Gazebo grasp-assist (Gazebo Classic services)
     g. Sleep 0.5s
     h. Move to lift pose (grasp Z + lift_height)
     i. Move to carry joints

**Grasp pose is computed dynamically from detected object position.** The task manager's `_grasp_pose_from_object()` transforms the detector's output to `arm_base_link` and applies center offsets.

---

## 9. Current Placement Flow (CRITICAL — MAIN ISSUE)

**Source:** `pick_place_commander.py` `_execute_place()` (line 1737)

### Step-by-step trace:

**A. Place pose computation (`_current_place_pose`, line 809):**

```python
def _current_place_pose(self) -> Tuple[float, float, float]:
    fallback = self._compute_fallback_place_pose()
    entity_name = self.held_entity_name
    offset = self._place_xy_offset_for_entity(entity_name)
    
    for adjustment_x in self._place_slot_reach_adjustments():
        shelf_pose = self._shelf_d_pose_for_entity(entity_name, adjustment_x=adjustment_x)
        # shelf_pose = (shelf[0] + xy_offset[0] + adjustment_x,
        #               shelf[1] + xy_offset[1],
        #               shelf_top_z)
        # = (0.0 + 0.0 + adjustment_x, -2.75 + 0.0, 0.10)
        # = (adjustment_x, -2.75, 0.10) in MAP frame
        
        transformed = self._transform_fixed_point('map', shelf_pose, 'arm_base_link')
        # Transforms the point from map frame to arm_base_link frame
        # This depends on the current TF: map → odom → base_link → arm_base_link
        
        candidate = (transformed[0], transformed[1], transformed[2])
        if self._within_workspace(candidate) and self.ik.solve(candidate) is not None:
            return candidate
    
    # Fallback
    return fallback
```

**B. Fallback computation (`_compute_fallback_place_pose`, line 865):**

```python
def _compute_fallback_place_pose(self):
    configured_fallback = (0.505, 0.0, 0.026)  # from pick_place_service.yaml
    shelf = (0.0, -2.75, 0.0)  # shelf_d_position
    place_top_z = 0.10  # shelf_d_top_z
    
    shelf_xy_z = (0.0, -2.75, 0.10)
    transformed = _transform_fixed_point('map', shelf_xy_z, 'arm_base_link')
    # shelf_z_arm = transformed[2]  # Z of table top in arm_base_link
    
    clearance = 0.008775  # release_clearance
    safe_z = shelf_z_arm + clearance
    candidate = (0.505, 0.0, safe_z)
    return candidate
```

**C. Release height computation (`_release_height_for_entity`, line 1419):**

```python
def _release_height_for_entity(self, entity_name):
    fallback = 0.12  # shelf_c_release_height / place_table_release_height
    
    heights = {
        'k12_red_cube': 0.07425,
        'k12_green_cylinder': 0.07425,
        'k12_blue_block': 0.07425,
    }
    object_height = heights.get(entity_name, fallback)
    shelf_top_z = 0.10  # from _shelf_c_param('top_z')
    clearance = 0.008775
    
    return shelf_top_z + 0.5 * object_height + clearance
    # = 0.10 + 0.037125 + 0.008775
    # = 0.1459 (in MAP frame)
```

**D. Execution sequence (`_execute_place`, line 1737):**

```python
place = self._current_place_pose()  # (X, Y, Z) in arm_base_link
pre_place_unit = self._radial_unit(place)
pre_place = self._reachable_offset_pose(place, pre_place_unit, approach)
place_lift = self._select_place_lift_pose(place, 'pre-place high')

self._move_pose(place_lift, 3.0, 'pre-place high')
self._move_pose(pre_place, 3.0, 'pre-place')
self._move_pose(place, 2.5, 'place')           # ← THIS IS THE CRITICAL MOVE
self._sleep(0.60)                               # place_contact_settle_sec
# Open gripper / release
self._retreat_straight_up_from_place(place)
self._move_joints(home_joints, 3.0, 'return home')
```

### The push-down problem:

The `place` pose Z is `transformed[2]` — the Z coordinate of the table top surface after TF transformation from map to arm_base_link. This should be the correct Z of the table surface in the arm's frame.

**However, the fallback pose uses hardcoded X=0.505, Y=0.0** — this is NOT dynamically computed from the robot's actual position relative to the table. If the robot stops at a different position (which it does, due to the 0.45m tolerance), the arm will try to reach a point that was computed for a different robot pose.

**The primary place pose IS dynamically computed** (via TF transform of the shelf position), but it depends on:
1. The TF chain being accurate at the moment of computation
2. The robot being close enough to the table for the arm to reach
3. The shelf_d_position (0.0, -2.75) being the correct table center in map frame

---

## 10. Current Placement Coordinate Source

| Quantity | Source | Value | Frame |
|---|---|---|---|
| Place station waypoint | `amms_waypoints.yaml` | (0.0, -2.10, -π/2) | map |
| Shelf/table position | `pick_place_service.yaml` `shelf_d_position` | (0.0, -2.75, 0.0) | map |
| Table top Z | `pick_place_service.yaml` `shelf_d_top_z` | 0.10 m | map |
| Alignment target | `amms_waypoints.yaml` `align_place_station_target` | (0.0, -2.75, 0.0) | map |
| Alignment stop distance | `amms_waypoints.yaml` | 0.25 m | — |
| Computed standoff | Derived | (0.0, -2.50, -π/2) | map |

**The place_station waypoint (0.0, -2.10) and the shelf_d_position (0.0, -2.75) are DIFFERENT coordinates.** The robot navigates to Y=-2.10, but the table center is at Y=-2.75. The arm must reach 0.65 m forward from the robot's stopping position to the table center. This is within the arm's max radius (0.658125 m from `pick_place_service.yaml`) but barely.

---

## 11. Current Placement Z Calculation

**Complete Z chain:**

1. Table surface in map frame: Z = 0.10 m (`shelf_d_top_z`)
2. Transform to arm_base_link: Z_arm = TF('map' → 'arm_base_link', (0, -2.75, 0.10)).z
3. Object-specific release height: `shelf_top_z + 0.5 * object_height + clearance` = 0.10 + 0.037125 + 0.008775 = 0.1459 m (map frame)
4. This is then transformed to arm_base_link for the actual arm target
5. Fallback: (0.505, 0.0, Z_arm + 0.008775) — hardcoded X,Y

**The Z calculation depends on:**
- `shelf_d_top_z: 0.10` — this is the configured table surface height
- TF accuracy — the transform from map to arm_base_link at the moment of computation
- `release_clearance: 0.008775` — only 8.8 mm above the table

**Potential issues:**
- If the TF reports the arm base at a slightly different height (AMCL vertical estimate, EKF drift), the Z will be wrong
- The 8.8 mm clearance provides almost no margin
- The `_release_height_for_entity()` computes release height in MAP frame but `_current_place_pose()` uses the TF-transformed Z directly — there may be a frame mismatch in how these interact

---

## 12. Current Navigation Tolerances

| Tolerance | Value | Source | Subsystem |
|---|---|---|---|
| Nav2 xy_goal_tolerance | 0.05 m | `nav2_amms.yaml:130` | Nav2 |
| Nav2 yaw_goal_tolerance | 0.10 rad | `nav2_amms.yaml:131` | Nav2 |
| Planner tolerance | 0.25 m | `nav2_amms.yaml:191` | Nav2 planner |
| position_only_goal_tolerance | 0.45 m | `amms_waypoints.yaml:100` | Task manager |
| position_only_min_obstacle_range | 0.25 m | `amms_waypoints.yaml:101` | Task manager |
| DWB trans_stopped_velocity | 0.03 m/s | `nav2_amms.yaml:158` | Nav2 controller |
| Progress checker radius | 0.08 m | `nav2_amms.yaml:124` | Nav2 controller |

---

## 13. Current Manipulation Tolerances

| Tolerance | Value | Source | Subsystem |
|---|---|---|---|
| place_final_standoff_xy_tolerance | 0.45 m | `amms_waypoints.yaml:72` | Task manager |
| place_final_standoff_yaw_tolerance | 0.40 rad | `amms_waypoints.yaml:73` | Task manager |
| place_final_approach_heading_tolerance | 0.35 rad | `amms_waypoints.yaml:65` | Task manager |
| place_final_approach_lateral_tolerance | 0.30 m | `amms_waypoints.yaml:66` | Task manager |
| approach_distance (arm) | 0.09871875 m | `pick_place_service.yaml:49` | Commander |
| lift_height | 0.1184625 m | `pick_place_service.yaml:50` | Commander |
| release_clearance | 0.008775 m | `pick_place_service.yaml:86` | Commander |
| place_contact_settle_sec | 0.60 s | `pick_place_service.yaml:105` | Commander |
| place_lift_height | 0.078975 m | `pick_place_service.yaml:106` | Commander |
| IK timeout | 0.05 s | (ik.py default) | Commander |

**Critical observation:** The `position_only_goal_tolerance` (0.45 m) and `place_final_standoff_xy_tolerance` (0.45 m) are the same value. Both are navigation-level tolerances that are far too loose for manipulation. The arm needs the robot within ~0.15 m of the correct position to reliably reach the table.

---

## 14. Current Grasp/Release Verification Mechanism

**Grasp verification (`_verify_grasp_secured`, line 1443):**
- Checks `self.held_entity_name is not None`
- Checks it matches the expected entity
- Calls `_hold_gripper_closed()`
- This is INTERNAL STATE ONLY — the `held_entity_name` is set by `_attach_object()` which calls Gazebo Classic services

**Release verification (`_verify_release_complete`, line 1455):**
- Checks `self.holding_object()` returns False
- This is the cleared state after `_release_held_object()`

**The "Grasp verified" / "Release verified" messages in the task manager (lines 502-508, 561-565) are log messages that fire after the Trigger service returns, NOT independent physical verification.**

---

## 15. Simulator Type and Available APIs

**Simulator:** Gazebo Sim (Ignition), NOT Gazebo Classic
- Launch: `ruby gz sim -r -v4 <world> --force-version 6`
- World: `og.world`
- Bridge: `ros_gz_bridge` (parameter_bridge) + `ros_gz_image` (image_bridge)

**Available APIs:**
- Topic bridging via `ros_gz_bridge` (gz_bridge.yaml)
- Image bridging via `ros_gz_image`
- Entity spawning via `ros_gz_sim/create`
- `/clock` bridge

**NOT available (Gazebo Classic only):**
- `/set_entity_state` (gazebo_msgs/srv/SetEntityState)
- `/get_entity_state` (gazebo_msgs/srv/GetEntityState)
- `/get_link_properties` (gazebo_msgs/srv/GetLinkProperties)
- `/set_link_properties` (gazebo_msgs/srv/SetLinkProperties)

**Impact:** The grasp assist and release snap-to-table in `pick_place_commander.py` depend on these services. They will fail silently (code handles this gracefully with warnings). Grasp fidelity depends entirely on gripper friction.

---

## 16. Unused/Duplicate Components

### INACTIVE (conditional, not launched):
| Component | Why Inactive | Keep/Remove |
|---|---|---|
| `k12_moveit_pick_place_commander` | `use_moveit_commander:=false` | KEEP — optional path |
| `move_group` | `use_moveit_commander:=false` | KEEP — optional path |

### OBSOLETE YAML files (not loaded by active launch):
| File | Loaded By | Status |
|---|---|---|
| `pick_place.yaml` | Nothing (auto_start: true, run_once: true) | OBSOLETE |
| `pick_place_standalone.yaml` | Nothing (auto_start: true, run_once: true) | OBSOLETE |
| `moveit_pick_place_standalone.yaml` | Nothing | OBSOLETE |
| `slam_toolbox_amms.yaml` | Nothing (SLAM not used in active launch) | OBSOLETE |

### OBSOLETE launch files:
| File | Status |
|---|---|
| `pick_place_demo.launch.py` | OBSOLETE — standalone demo |
| `amms_mapping.launch.py` | OBSOLETE — SLAM mapping (not used) |
| `moveit_pick_place_demo.launch.py` | OBSOLETE — MoveIt demo |

### OBSOLETE articubot_one launch files:
| File | Status |
|---|---|
| `articubot_one_with_arm/launch/navigation_launch.py` | OBSOLETE — superseded by nav2_bringup |
| `articubot_one_with_arm/launch/localization_launch.py` | OBSOLETE — superseded by nav2_bringup |
| `articubot_one_with_arm/launch/launch_robot.launch.py` | OBSOLETE — real robot launch |
| `articubot_one_with_arm/launch/online_async_launch.py` | OBSOLETE — SLAM |
| `articubot_one_with_arm/config/nav2_params.yaml` | OBSOLETE — articubot's own Nav2 config |

### Duplicate code paths:
- `_place_table_release_position()` (line 1342) simply calls `_shelf_d_release_position()` — redundant alias
- `_shelf_c_release_position()` (line 1339) simply calls `_shelf_d_release_position()` — redundant alias
- `_place_table_pose_for_entity()` (line 1368) simply calls `_shelf_d_pose_for_entity()` — redundant alias
- `_shelf_c_pose_for_entity()` (line 1360) simply calls `_shelf_d_pose_for_entity()` — redundant alias

These exist because the system was designed for multiple placement surfaces (Shelf C, Shelf D, place table) but they all share the same parameters now.

---

## 17. Exact Files That Must Change

| File | Reason |
|---|---|
| `amms_task_manager.py` | Rewrite as explicit state machine with proper phases, operator interface, placement navigation |
| `amms_waypoints.yaml` | Fix tolerances, separate navigation vs manipulation tolerances |
| `pick_place_service.yaml` | Fix placement parameters (clearance, fallback pose) |

---

## 18. Exact Files That Must NOT Change

| File | Reason |
|---|---|
| `object_detector.py` | Working perception pipeline, no changes needed |
| `ik.py` | Working IK solver, no changes needed |
| `scene_spawner.py` | Working spawner, no changes needed |
| `readiness_gate.py` | Working gate, no changes needed |
| `waypoint_manager.py` | Visualization only, no changes needed |
| `cloud_to_scan.py` | Working converter, no changes needed |
| `nav2_amms.yaml` | Nav2 config is correct for this robot |
| `ekf_odom.yaml` | EKF config is correct |
| `pointcloud_to_laserscan.yaml` | Scanner config is correct |
| `gz_bridge.yaml` | Bridge config is correct |
| `amms_nav_pick_place.launch.py` | Launch structure is correct |
| `amms_navigation.launch.py` | Navigation launch is correct |
| All URDF/xacro files | Robot description is correct |
| All MoveIt config files | MoveIt path is inactive, leave alone |

---

## 19. Risks and Regressions

| Risk | Mitigation |
|---|---|
| Rewriting task_manager breaks existing pick flow | Preserve the Trigger service interface; task_manager calls same services |
| Tighter tolerances cause Nav2 failures | Keep retry logic (3 attempts) and costmap clearing |
| State machine refactor introduces bugs | Test each phase independently; keep the existing phase names for logging |
| Removing position_only from place_station breaks something | The alignment logic already handles this; verify with runtime test |
| stdin through ros2 launch is unreliable | Add robust EOFError handling; document interactive launch requirement |

---

## 20. Validation Plan

**STATIC AUDIT ONLY — RUNTIME NOT EXECUTED**

To validate after implementation:

```bash
# 1. Build
colcon build --packages-select k12_pick_place

# 2. Launch (interactive terminal required)
ROS_DOMAIN_ID=77 ros2 launch k12_pick_place amms_nav_pick_place.launch.py

# 3. Verify prompt appears:
#    Robot ready. Enter target [A/B/C]:

# 4. Test A:
#    - Enter A
#    - Watch log for state transitions
#    - Verify robot navigates to pickup_point_1_red
#    - Verify detection appears in log
#    - Verify pick executes
#    - Verify robot navigates to place station
#    - Verify robot reaches manipulation position (within 0.15m of table)
#    - Verify place executes
#    - Verify object is released above surface (not pushed into it)
#    - Verify robot returns to initial pose

# 5. Test B, C (same generic pipeline)

# 6. Test no object:
#    - Remove objects from world
#    - Enter A
#    - Verify "No object detected" message
#    - Verify robot returns to initial pose

# 7. Test invalid command:
#    - Enter D
#    - Verify re-prompt

# 8. Capture runtime metrics:
ros2 topic echo /amms/task_state
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /amcl_pose
```
