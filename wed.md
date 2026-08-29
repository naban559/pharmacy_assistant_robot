# Walkthrough: Phased Dynamic Pick-and-Place (A/B/C Command Model)

## Summary of Changes

### Modified Files

| File | What Changed |
|------|-------------|
| [amms_task_manager.py](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/amms_task_manager.py) | Replaced `_run_requested_mission()` with the A/B/C command loop |
| [amms_waypoints.yaml](file:///home/nabhan/try_15/src/k12_pick_place/config/amms_waypoints.yaml) | `autostart: false` |

### Everything Else: Untouched

`pick_place_commander.py`, `object_detector.py`, `moveit_pick_place_commander.py`, all YAML configs, URDF, Nav2 stack, launch files — **zero changes**.

---

## How to Use

Launch the system normally:
```bash
ros2 launch k12_pick_place amms_nav_pick_place.launch.py
```

Once Nav2, AMCL, the arm controllers, and the detector are all active, you will see this prompt in the **`amms_task_manager`** terminal:

```
Robot ready. Enter target [A/B/C]:
```

Enter a single character:

| Command | Target | Object |
|---------|--------|--------|
| `A` | `pickup_point_1_red` (X=−3.0 m) | Red cube |
| `B` | `pickup_point_2_green` (X= 0.0 m) | Green cube |
| `C` | `pickup_point_3_blue` (X=+3.0 m) | Blue cube |

---

## Phase-by-Phase Flow

```
WAIT_FOR_COMMAND       ← operator enters A/B/C
  SELECT_TARGET        ← captures initial map pose
  NAVIGATE_TO_PICK_ZONE
  SEARCH_FOR_OBJECT    ← waits for live RGB-D detection
  VALIDATE_OBJECT      ← checks reachability via IK
  CONFIRM_FRESH_DETECTION
  PICK_OBJECT          ← calls /amms/pick_object trigger
  VERIFY_GRASP         ← Gazebo attach confirmation
  RETRACT_ARM          ← fold arm with object
  POST_PICK_BACKUP
  NAVIGATE_TO_PLACE_APPROACH
  NAVIGATE_TO_PLACE_ZONE
  DETECT_PLACEMENT_SURFACE
  ESTIMATE_PLACE_POSE  ← TF-based placement target
  PLACE_OBJECT         ← calls /amms/place_object trigger
  VERIFY_RELEASE       ← Gazebo detach confirmation
  RETRACT_ARM
  POST_PLACE_BACKUP
  RETURN_TO_INITIAL_POSE ← navigates back to captured pose
  TASK_COMPLETE
WAIT_FOR_COMMAND       ← ready for next command
```

### What is Dynamic (Runtime-Computed)

- **Object position** — from live RGB-D camera via `/amms/detections`
- **Grasp pose** — computed from detected object position in `arm_base_link` frame via `_grasp_pose_from_object()`
- **Object shape/size** — classified by `object_detector.py` via contour analysis
- **Initial pose** — captured from TF (`map←base_link`) at command time
- **Placement IK** — transforms `place_table_position` to current `arm_base_link` frame via TF at execution time

### What is Configured (Static but Not Hard-Coded Object Positions)

- Pickup area nav goals: waypoints in `amms_waypoints.yaml`
- Placement station: `place_station` waypoint
- Physical arm limits, joint constraints, Nav2 parameters

---

## Terminal Output During a Mission

```
Robot ready. Enter target [A/B/C]: A
  → Target: A (pickup_point_1_red).  Navigating…
  ✓ Object detected: red_cube.
  ✓ Grasp verified: red_cube.
  ✓ Object placed: red_cube.
  ✓ Task A complete.  Robot has returned to initial pose.

Robot ready. Enter target [A/B/C]:
```

### Failure Cases

| Failure | Behavior |
|---------|----------|
| Invalid input (e.g. `X`) | `Invalid command "X". Please enter A (red), B (green), or C (blue).` — re-prompt |
| No object at pickup zone | `✗ No object detected...` — returns to initial pose, re-prompts |
| Pick service fails | `✗ Pick failed: ...` — arm recovers to carry pose, returns to initial pose, re-prompts |
| Navigation failure | Exception propagates → `Mission failed: ...` → re-prompts |
| stdin closed (EOF) | Mission loop exits gracefully |

---

## Gazebo Entity Name Update

`_known_gazebo_objects()` now uses the correct `og.world` entity names:

| Object ID | Gazebo Entity | Color | Shape |
|-----------|--------------|-------|-------|
| `red_cube` | `cube_red` | red | cube |
| `green_cube` | `cube_green` | green | cube |
| `blue_cube` | `cube_blue` | blue | cube |

---

## Build Result

```
colcon build --packages-select k12_pick_place
Summary: 1 package finished [1.06s]   ← 0 errors, 0 warnings
```

Python AST parse: **OK**
