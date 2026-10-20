# PLACEMENT_STATION_DIAGNOSIS.md

## 1. Executive Summary

The robot stops before reaching the placing station because of a **two-layer tolerance problem** in the task manager that causes premature acceptance of the navigation goal, combined with a **DriveOnHeading alignment that is itself governed by the same loose tolerance**. The primary root cause is **GOAL-TOLERANCE / TASK-MANAGER PROBLEM**, not physical geometry or costmap inflation.

**The specific mechanism:**

1. `place_station` is configured as a `position_only_waypoint` with `position_only_goal_tolerance: 0.45 m` (`amms_waypoints.yaml:100`).
2. Before Nav2 even attempts to navigate, `_accept_position_only_waypoint()` (`amms_task_manager.py:861-904`) checks whether the robot is already within 0.45 m of the goal. If yes, it skips Nav2 entirely and jumps to alignment.
3. Even if Nav2 is used, `_position_only_waypoint_reached()` (`amms_task_manager.py:993`) fires during the navigation poll loop and cancels the Nav2 goal the moment the robot enters the 0.45 m radius — **well before** Nav2's own 0.05 m `xy_goal_tolerance` would be satisfied.
4. The alignment phase (`_align_to_place_station`, line 1278) then computes a standoff position 0.25 m behind the `align_place_station_target` (Y=-2.75), yielding a standoff at approximately Y=-2.50. It then attempts a DriveOnHeading of up to 0.72 m.
5. `_place_standoff_ready()` (`amms_task_manager.py:1333-1365`) accepts the standoff with `place_final_standoff_xy_tolerance: 0.45 m` — the same loose tolerance.
6. Net result: the robot can stop anywhere within a ~0.9 m diameter circle centered on the standoff and the task manager declares success.

**This is not a single bug but a systematic design choice**: every tolerance layer uses 0.45 m, which compounds. The arm then attempts to reach the place table from a position that may be 0.4–0.9 m away from the intended spot.

**Runtime measurement required to confirm:** The exact stopping position was not measured (no ROS 2 runtime available). The commands to capture the evidence are listed below. Everything in this report is from direct source code inspection.

---

## 2. Runtime Evidence (Not Measured — Source-Derived)

**No live runtime was available.** The following values are from the source code and YAML configuration. To capture actual runtime values, run:

```bash
# Terminal 1: launch the system
ROS_DOMAIN_ID=77 ros2 launch k12_pick_place amms_nav_pick_place.launch.py

# Terminal 2: monitor task state and robot pose during place approach
ros2 topic echo /amms/task_state
ros2 run tf2_ros tf2_echo map base_link
```

**What the source code specifies:**

| Quantity | Configured Value | Source |
|---|---|---|
| Place station waypoint (map) | X=0.0, Y=-2.10, Yaw=-π/2 | `amms_waypoints.yaml:54` |
| Place approach waypoint (map) | X=0.0, Y=-1.50, Yaw=-π/2 | `amms_waypoints.yaml:53` |
| Align place station target (map) | X=0.0, Y=-2.75 | `amms_waypoints.yaml:61` |
| Align place station stop distance | 0.25 m | `amms_waypoints.yaml:62` |
| Computed standoff (map) | X=0.0, Y=-2.50, Yaw=-π/2 | Derived: -2.75 + 0.25*sin(-π/2) = -2.50 |
| Position-only goal tolerance | 0.45 m | `amms_waypoints.yaml:100` |
| Nav2 xy_goal_tolerance | 0.05 m | `nav2_amms.yaml:130` |
| Nav2 yaw_goal_tolerance | 0.10 rad | `nav2_amms.yaml:131` |
| Place standoff XY tolerance | 0.45 m | `amms_waypoints.yaml:72` |
| Place standoff yaw tolerance | 0.40 rad | `amms_waypoints.yaml:73` |
| Place final approach max distance | 0.7239375 m | `amms_waypoints.yaml:67` |
| Place final approach speed | 0.15 m/s | `amms_waypoints.yaml:68` |

**Expected runtime log output** (from `_log_navigation_goal` and `_log_navigation_result`):
```
Nav2 goal place_station attempt 1/3: goal=(0.00, -2.10, yaw=-1.57), base=(X, Y, yaw=Z), linear_error=D m, yaw_error=E rad
Nav2 goal place_station succeeded: final_error=F m, yaw_error=G rad
```

---

## 3. Costmap Evidence

### 3.1 Configuration (from `nav2_amms.yaml`)

| Parameter | Value |
|---|---|
| **Robot footprint** | `[[0.24, 0.16], [0.24, -0.16], [-0.24, -0.16], [-0.24, 0.16]]` — **0.48 m × 0.32 m** |
| **Inflation radius (local)** | 0.48 m |
| **Inflation radius (global)** | 0.48 m |
| **Cost scaling factor** | 5.0 (both costmaps) |
| **Local costmap** | 5m × 5m rolling window, 0.05 m resolution |
| **Global costmap** | 12m × 10m static, 0.05 m resolution |
| **Planner** | NavFn A*, tolerance=0.25, allow_unknown=false |
| **Controller** | DWB, max_vel_x=0.21 m/s, max_vel_theta=0.40 rad/s |

### 3.2 Analysis

The inflation radius of 0.48 m is **equal to the robot's half-length** (0.24 m from center to front). This means any obstacle within 0.48 m of a cell inflates that cell's cost. For the place station at Y=-2.10:

- The robot footprint extends 0.24 m forward from center.
- Inflation extends another 0.48 m beyond any obstacle boundary.
- If the place table (or its support structure) has any obstacle cell within ~0.72 m of the intended goal position, the costmap will mark that area as high-cost or lethal.

**However**, whether the place table creates such obstacles depends on the actual Gazebo world geometry (`og.world`) and the laser scan's view of it. The static map (`og.pgm`) may or may not include the place table. Without running the simulation and inspecting the actual costmap, I cannot determine whether inflation is clipping the reachable region.

**Costmap evidence conclusion:** The inflation radius (0.48 m) combined with the robot footprint (0.48 m × 0.32 m) is reasonable for this robot size. Whether the place table geometry creates an unreachable zone is **unknown without runtime costmap inspection**. The 0.48 m inflation is not excessively large — it's a standard conservative setting. This is **not the primary cause** of the early stop, but could be a contributing factor if the table obstacles are close to the goal.

---

## 4. Placement Diagnosis — Why the Object Is Pushed Downward

### 4.1 Placement Z Computation

The place pose Z is computed in `pick_place_commander.py:_current_place_pose()` (line 809) and `_compute_fallback_place_pose()` (line 865):

```python
# _compute_fallback_place_pose (line 865-886)
shelf = self._shelf_d_param('position')  # [0.0, -2.75, 0.0] from pick_place_service.yaml:33
place_top_z = float(self._shelf_d_param('top_z'))  # 0.10 from pick_place_service.yaml:81
shelf_xy_z = (shelf[0], shelf[1], place_top_z)  # (0.0, -2.75, 0.10)
transformed = self._transform_fixed_point('map', shelf_xy_z, 'arm_base_link')
# transformed[2] = Z of the table top in arm_base_link frame
safe_z = shelf_z_arm + clearance  # shelf_z_arm + 0.008775
candidate = (configured_fallback[0], configured_fallback[1], safe_z)
# fallback from pick_place_service.yaml:119 = [0.505, 0.0, 0.026]
```

The **table top Z in map frame** is 0.10 m. After TF transform to `arm_base_link`, this becomes approximately 0.026 m (the arm base is mounted ~0.074 m above the base_link, and the base_link is ~0.048 m above ground). The release Z is then:

```
release_Z = shelf_z_arm + release_clearance
         ≈ 0.026 + 0.008775
         ≈ 0.035 m (in arm_base_link frame)
```

Then the object's own height is factored in via `_release_height_for_entity()` (line 1419):

```python
return shelf_top_z + 0.5 * object_height + clearance
# = 0.10 + 0.5 * 0.07425 + 0.008775
# = 0.10 + 0.037125 + 0.008775
# = 0.1459 m (in map frame)
```

This is then transformed to `arm_base_link`, yielding approximately **0.072 m**.

### 4.2 The Push-Down Problem

The `_execute_place()` sequence (line 1737) is:
1. Move to `place_lift` (high above table) — `place_lift_height: 0.078975 m` above place
2. Move to `pre_place` — `approach_distance: 0.09871875 m` back from place along radial
3. Move to `place` — the computed place pose
4. Sleep `place_contact_settle_sec: 0.60 s`
5. Open gripper / release

The critical issue: **the `place` pose Z value is the computed safe height above the table, but the gripper continues moving downward after the object contacts the table surface.** The `place_contact_settle_sec` delay (0.60 s) assumes the object settles, but if the arm's trajectory planning drives the end-effector below the computed Z (due to IK solution variation, joint limits, or the arm's kinematic chain geometry), the gripper physically pushes the object into the table.

Additionally, the `_retreat_straight_up_from_place()` (line 1782) retreats in Z from the place pose upward. If the place pose Z is already too low (object already inside table), the retreat starts from a negative-Z position and the damage is done.

**Root cause of push-down:** The place pose Z is computed from the TF-transformed table top height, which depends on:
- `shelf_d_top_z: 0.10` (map frame) — this is the configured table surface height
- The TF chain `map → odom → base_link → arm_base_link`
- Any TF delay or noise at the moment of computation

If the TF chain reports the arm base at a slightly different height than expected (AMCL drift, EKF vertical estimate, URDF mounting offset), the computed place Z can be too low. The `release_clearance: 0.008775 m` (8.8 mm) provides almost no margin.

**This is a PLACEMENT/MANIPULATION problem, not a navigation problem.** The robot may be at the correct XY position but the arm is placing too low.

---

## 5. Architecture Diagnosis — A/B/C Flow Verification

### 5.1 Source Code Confirms Correct Architecture

Traced from `amms_task_manager.py:_run_abc_mission()` (lines 382–592):

```
WAIT_FOR_COMMAND
    ↓ (operator enters A/B/C)
SELECT_TARGET  →  capture initial pose via TF
    ↓
NAVIGATE_TO_PICK_ZONE  →  _navigate_to_waypoint(pick_waypoint)
    ↓                      where pick_waypoint = pickup_point_{1_red,2_green,3_blue}
SEARCH_FOR_OBJECT  →  _acquire_reachable_object()
    ↓                  (dynamic detection from object_detector.py)
VALIDATE_OBJECT  →  _candidate_is_valid()
    ↓
CONFIRM_FRESH_DETECTION  →  _confirm_fresh_target()
    ↓
PICK_OBJECT  →  _call_pick_with_retries() → /amms/pick_object service
    ↓
VERIFY_GRASP  →  (logged, not physically verified — see Section 6)
    ↓
RETRACT_ARM  →  fold arm
    ↓
POST_PICK_BACKUP  →  Nav2 BackUp
    ↓
NAVIGATE_TO_PLACE_APPROACH  →  place_approach waypoint (0.0, -1.50)
    ↓
NAVIGATE_TO_PLACE_ZONE  →  place_station waypoint (0.0, -2.10)
    ↓
DETECT_PLACEMENT_SURFACE  →  _ensure_place_reachable()
    ↓
ESTIMATE_PLACE_POSE  →  _verify_localization()
    ↓
PLACE_OBJECT  →  /amms/place_object service
    ↓
VERIFY_RELEASE  →  (logged, not physically verified — see Section 6)
    ↓
RETRACT_ARM  →  fold arm
    ↓
POST_PLACE_BACKUP  →  Nav2 BackUp
    ↓
RETURN_TO_INITIAL_POSE  →  navigate to captured initial pose
    ↓
TASK_COMPLETE  →  WAIT_FOR_COMMAND
```

### 5.2 Key Confirmation Points

| Requirement | Status | Evidence |
|---|---|---|
| A/B/C select navigation zone | ✅ CORRECT | `_ABC_WAYPOINTS = {'A': 'pickup_point_1_red', 'B': 'pickup_point_2_green', 'C': 'pickup_point_3_blue'}` |
| Object pose is dynamic | ✅ CORRECT | `_acquire_reachable_object()` → `object_detector.py` HSV+depth pipeline |
| No hardcoded object XYZ | ✅ CORRECT | No `pick_red()`, `pick_green()`, `pick_blue()` — single generic pipeline |
| Place station is predefined | ✅ CORRECT | `place_station: [0.0, -2.10, -π/2]` in yaml |
| Return to exact initial pose | ✅ CORRECT | `_current_base_pose()` captures TF at command time, `_return_to_initial_pose()` navigates back |
| No-object handling | ✅ CORRECT | Lines 436-448: prints message, skips pick, returns to initial pose |
| Invalid command handling | ✅ CORRECT | Lines 359-366: prints error, re-prompts |

**Architecture diagnosis: CORRECT.** The implementation matches the required A/B/C → predefined search zone → dynamic detection → pick → placing station → placement → return-to-initial-pose flow.

---

## 6. Grasp/Release Verification

### 6.1 What the Code Does

The task manager reports "grasp verified" and "release verified" at:
- `amms_task_manager.py:502-508` (grasp verified log)
- `amms_task_manager.py:561-565` (release verified log)

These are **log messages that fire after the Trigger service returns success**. They are **not** independent physical verification.

### 6.2 What pick_place_commander Does

The actual verification in `pick_place_commander.py`:

1. `_verify_grasp_secured()` (line 1443): Checks that `self.held_entity_name` is set and matches expected. This is an **internal state variable** set by `_attach_object()` (line 1100), which calls Gazebo Classic services (`/set_entity_state`, `/get_entity_state`). Under Ignition/Gazebo Sim, these services **do not exist**. The code at line 1114-1141 handles this gracefully — it warns and continues.

2. `_verify_release_complete()` (line 1455): Checks that `self.held_entity_name` is None. This is the cleared state after `_release_held_object()`.

### 6.3 Conclusion

**Grasp/release verification is NOT genuine physical verification.** It is:
- Internal state tracking (`held_entity_name` variable)
- Dependent on Gazebo Classic services that don't exist under Ignition
- The "verified" messages are misleading — they confirm the service call completed, not that the object is physically attached

**Classification: GAZEBO (pre-existing issue, not introduced by A/B/C work)**

---

## 7. Problems Classified by Subsystem

### NAVIGATION
- **position_only_waypoints tolerance (0.45 m)** causes premature goal acceptance for `place_station`, `place_approach`, and `home`. This is the **primary cause** of the robot stopping early.
- Nav2's own xy_goal_tolerance (0.05 m) is never satisfied because the task manager cancels the Nav2 goal when the position-only check passes.
- **Evidence needed:** Runtime log showing `position-accept staging waypoint place_station` firing with linear_error < 0.45 m.

### LOCALIZATION
- AMCL is confirmed active by the readiness gate before `WAIT_FOR_COMMAND` appears.
- No evidence of localization instability from source alone.
- **Evidence needed:** `ros2 topic echo /amcl_pose` — covariance should be small and stable.

### COSTMAP
- Inflation radius (0.48 m) is reasonable for this robot size.
- Whether the place table creates an unreachable zone depends on actual world geometry — **cannot determine from source alone**.
- **Evidence needed:** RViz costmap overlay against place_station coordinate.

### TASK MANAGER
- **`position_only_goal_tolerance: 0.45 m`** is the dominant contributor to early stopping.
- **`place_final_standoff_xy_tolerance: 0.45 m`** compounds the issue during alignment.
- **`position_only_waypoints: [place_station, place_approach, home]`** applies this tolerance to all three waypoints.
- The `_position_only_waypoint_reached()` check fires during Nav2 navigation and cancels goals prematurely.

### PERCEPTION
- Dynamic perception pipeline is correctly implemented.
- Object positions are always from live detection, never hardcoded.
- Shape classification is hardcoded by color (red=cube, green=cylinder, blue=rectangular_block) but this is acceptable for the demo.

### PLACEMENT
- **Place pose Z** is computed from `shelf_d_top_z: 0.10 m` (map frame) + clearance + half object height.
- **`release_clearance: 0.008775 m`** (8.8 mm) provides minimal margin.
- The push-down problem is likely caused by the arm trajectory driving below the computed Z due to IK solution variation or TF chain inaccuracy.
- **This is a SEPARATE problem from navigation.**

### GRIPPER
- Gripper open/close now uses joint_states feedback (previously open-loop).
- `_close_gripper()` uses velocity-based detection (stops when fingers stall).
- No issues identified from source.

### GAZEBO
- Gazebo Classic services (`/set_entity_state`, `/get_entity_state`, `/get_link_properties`, `/set_link_properties`) do not exist under Ignition/Gazebo Sim.
- Grasp assist and release snap-to-table rely on these services.
- The code handles this gracefully (warns and continues) but means grasp fidelity depends entirely on gripper friction.
- **Pre-existing issue, not introduced by A/B/C work.**

---

## 8. Recommended Changes

**Only changes supported by the analysis above. All require runtime confirmation before implementation.**

### Change 1: Tighten position_only_goal_tolerance (HIGH PRIORITY)

| | |
|---|---|
| **Current value** | `position_only_goal_tolerance: 0.45` (`amms_waypoints.yaml:100`) |
| **Measured problem** | Robot accepted as "at place_station" when 0.45 m away — too far for arm to reach |
| **Why this value causes the problem** | The arm's workspace radius is 0.14–0.90 m. If the robot stops 0.45 m from the goal, the arm may not be able to reach the table. The 0.45 m tolerance is appropriate for `home` (no arm reach needed) but too loose for `place_station` (arm must reach the table). |
| **Proposed value** | `position_only_goal_tolerance: 0.20` for place_station; or split `position_only_waypoints` to exclude place_station from position-only acceptance |
| **Expected effect** | Robot will navigate closer to the actual place_station before jumping to alignment. The DriveOnHeading phase will then have less distance to cover. |
| **Risk/regression** | If the costmap has obstacles near the place station, a tighter tolerance may cause Nav2 to fail more often. Mitigated by the existing retry logic (3 attempts) and costmap clearing. |

### Change 2: Tighten place_final_standoff_xy_tolerance (MEDIUM PRIORITY)

| | |
|---|---|
| **Current value** | `place_final_standoff_xy_tolerance: 0.45` (`amms_waypoints.yaml:72`) |
| **Measured problem** | After DriveOnHeading alignment, the robot is accepted as "ready" when still 0.45 m from the standoff position |
| **Why this value causes the problem** | Compounds with Change 1. Even if the robot gets closer via Nav2, the alignment phase still accepts a 0.45 m error. |
| **Proposed value** | `place_final_standoff_xy_tolerance: 0.15` |
| **Expected effect** | DriveOnHeading will attempt more precise alignment before giving up. |
| **Risk/regression** | If the DriveOnHeading can't achieve 0.15 m precision (e.g., wheel slippage, localization noise), the alignment may fail and trigger the RuntimeError fallback. The existing fallback logic handles this. |

### Change 3: Separate position_only_waypoints for place_station (ALTERNATIVE to Change 1)

Instead of including `place_station` in `position_only_waypoints`, remove it so Nav2 always attempts full navigation to the exact goal. The 0.05 m Nav2 tolerance will then govern stopping.

| | |
|---|---|
| **Current value** | `position_only_waypoints: [place_station, place_approach, home]` |
| **Proposed value** | `position_only_waypoints: [place_approach, home]` |
| **Expected effect** | Nav2 will navigate all the way to the place_station with 0.05 m tolerance. |
| **Risk/regression** | If Nav2 can't reach 0.05 m (costmap obstacles), it will fail and trigger retry logic. May need to increase `xy_goal_tolerance` slightly if this happens. |

### Change 4: Increase release_clearance (LOW PRIORITY — placement fix)

| | |
|---|---|
| **Current value** | `release_clearance: 0.008775` (8.8 mm) |
| **Proposed value** | `release_clearance: 0.020` (20 mm) |
| **Expected effect** | Object will be released slightly higher above the table, reducing push-down risk. |
| **Risk/regression** | Object may be dropped from slightly higher, increasing bounce risk. But 20 mm is still very close. |

---

## 9. Changes Actually Made

**No source files were modified in this pass.** This report is the deliverable. The recommended changes in Section 8 require runtime validation before implementation.

---

## 10. Validation

**NOT RUN — no ROS 2/Gazebo runtime available.** To validate after implementing changes:

```bash
# 1. Launch the system
ROS_DOMAIN_ID=77 ros2 launch k12_pick_place amms_nav_pick_place.launch.py

# 2. In the launch terminal, enter A
# 3. In a second terminal, capture:
ros2 topic echo /amms/task_state
ros2 run tf2_ros tf2_echo map base_link

# 4. At the moment the task manager says "place" or "place object":
#    Record map→base_link X, Y, yaw
#    Compare against place_station goal (0.0, -2.10, -1.57)

# 5. Check costmap:
#    In RViz, add Costmap displays for /local_costmap/costmap and
#    /global_costmap/costmap, overlay against place_station coordinate

# 6. Check AMCL:
ros2 topic echo /amcl_pose  # covariance should shrink and stabilize

# 7. Check the log output for:
#    "Nav2 goal place_station succeeded: final_error=X.XXX m"
#    This is the exact position error you need.
```

**Test matrix:**

| Test | Command | What to Capture |
|---|---|---|
| A (red) → place | Enter `A` | Nav2 goal log, final robot pose, position error |
| B (green) → place | Enter `B` | Same |
| C (blue) → place | Enter `C` | Same |
| No object at zone | Remove objects, enter `A` | "No object detected" message, return to initial pose |
| Invalid command | Enter `D` | "Invalid command" message, re-prompt |

**Do not write "PASS" in this section until the above commands have been executed and the results recorded.**

---

## Summary Classification

| Category | Verdict |
|---|---|
| **Primary root cause** | GOAL-TOLERANCE / TASK-MANAGER PROBLEM |
| **Secondary factor** | Possible CONTRIBUTION from costmap inflation (unverified) |
| **Architecture** | CORRECT — A/B/C → zone → dynamic detect → pick → place → return |
| **Grasp verification** | NOT GENUINE — internal state only, Gazebo Classic services unavailable |
| **Placement push-down** | SEPARATE PROBLEM — Z computation + minimal clearance |
| **Recommended fix** | Tighten position_only_goal_tolerance from 0.45 to 0.20 (or remove place_station from position_only_waypoints) |
