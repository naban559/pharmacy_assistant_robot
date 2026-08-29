# K12 AMMS Workspace — Diagnosis & File Guide

Two packages: `articubot_one_with_arm` (the rover + arm body, physics, and
simulation — "what the robot IS") and `k12_pick_place` (the perception +
brain — "what the robot DOES"). This document explains both, then answers
your three questions: why the arm free-falls, whether the cubes fit the
gripper, and what to edit next to get pick-and-place running.

---

## PART 1 — Root cause reports (read this first)

### Issue 3 (doing this first — it's the blocker): arm has no holding torque

**Root cause:** `gz_ros2_control`'s position command interface is not a
rigid lock — it's a small internal PD (proportional-derivative) controller
built into the plugin. Every joint needs its own
`position_proportional_gain` (and ideally `position_derivative_gain`) set
**inside the `<ros2_control>` block in the URDF/xacro, as a `<param>` under
that joint's `<command_interface name="position">`**. If it's not set
there, it defaults to `0.1` — far too weak to hold any real link against
gravity, so the joint just swings down.

**Where it actually lives:** `articubot_one_with_arm/description/arm_ros2_control.xacro`

**What I found in your workspace:** you (very reasonably) tried to fix
this, but in four places that all look plausible and are all no-ops:

| Where you set it | Why it doesn't work |
|---|---|
| `config/my_controllers.yaml` → `controller_manager: ros__parameters: position_proportional_gain: 30.0` | Not a recognized parameter of the controller_manager node itself — it's a per-joint hardware parameter, read from the URDF, not from this yaml's top namespace. Silently ignored. |
| `config/my_controllers.yaml` → `gz_ros_control:` / `gz_ros2_control:` parameter blocks | These are parameters *for a node named* `gz_ros_control` / `gz_ros2_control`. No such node exists — the controller manager node is called `controller_manager`. Silently ignored. |
| `description/four_wheel.trans` → `<position_proportional_gain>30.0</position_proportional_gain>` directly under `<plugin>` | Not a recognized tag at the plugin level in gz_ros2_control's schema (it only reads per-joint `<param>` tags nested inside each joint's `<command_interface>`). Silently ignored. |
| `launch/launch_sim.launch.py` → `ros2 param set /gz_ros2_control position_proportional_gain 30.0` (run after joint_broad spawns) | Two problems: (1) the node is named `/controller_manager`, not `/gz_ros2_control`, so this targets a node that doesn't exist; (2) even on the right node, this gain isn't a live-settable ROS parameter at all — gz_ros2_control reads it once from the URDF at hardware-*configure* time (before any controller even spawns), so setting it afterward via `ros2 param set` can't reach the code path that uses it. |

**The fix:** I added the gain as a `<param>` inside every arm/gripper
joint's `<command_interface name="position">` in
`description/arm_ros2_control.xacro` (the one place that's actually read).
I also removed the four no-op attempts above so the file doesn't carry
dead config that looks like it's doing something. Values are roughly
scaled to each joint's effort limit (bigger joint closer to the base needs
more gain to hold the weight of everything past it):

```
arm_base_yaw_joint:       P=40,  D=2.0
arm_shoulder_pitch_joint: P=70,  D=3.0   (biggest — carries the whole arm)
arm_elbow_pitch_joint:    P=50,  D=2.5
arm_wrist_pitch_joint:    P=25,  D=1.5
arm_wrist_roll_joint:     P=10,  D=0.5
left/right_gripper_joint: P=200, D=5.0
```

These are reasonable starting points, not tuned optimums — if the arm now
holds but oscillates/shakes, lower the P values; if it still sags, raise
them. `arm_shoulder_pitch_joint` is the one to watch since it's fighting
the most gravity torque.

**One more thing found while fixing this:** the file I wrote had a
genuine XML bug — an HTML-style `--` inside an XML comment, which is
illegal in XML and would have made `xacro` fail to parse the file at all
(a hard crash, not just a weak-PID problem). Fixed and verified this file
and every other `.xacro`/`.trans`/`.world` file in your workspace parses
as valid XML.

### Issue 2: arm mounting height — confirmed from the actual mesh, not guessed

I measured `base_link.stl`'s real bounding box directly (binary STL parse):
z spans **0 to 95mm** in the mesh's own frame. With
`base_footprint→base_link` fixed at `z=0.038m`, the true chassis-top
height above the ground was `0.038 + 0.095 = 0.133m` — matching your
13.5cm estimate almost exactly. In your *current, re-scaled* chassis
(you moved wheel joints from ±0.13/0.09 to ±0.195/0.135, a 1.5× scale-up),
the equivalent mount height should scale by the same 1.5×.

**Where it lives:** `description/four_wheel.xacro`, the line
`<xacro:k12_arm parent="base_link" mount_x="..." mount_y="0.0" mount_z="..."/>`
right before `</robot>`.

You currently have `mount_z="0.142"` — reasonable for the 1.5×-scaled
chassis (0.133 × 1.5 ≈ 0.1425 doesn't quite work since the offset isn't
purely linear with the footprint scale, but it's very close to correct;
leave it as-is, it's not the free-fall cause).

### Issue 1: arm mounting position — currently forward, may need to move back after the scale check below

You have `mount_x="0.12"`. Given your chassis is now ~1.5× the original
footprint (roughly 0.42m long, wheels at ±0.195), 0.12 is a reasonable
"toward the front but clear of the lidar/camera cluster" position, same
logic as before scaled up. Leave this as-is for now — see the scale
mismatch note below before nudging it further, since if we resize the arm
that also changes how much front clearance you need.

### A mismatch I found that you didn't ask about but should know

Your `k12_pick_place/ik.py` `ArmGeometry` constants are correctly kept in
sync with `arm.xacro`'s current dimensions (e.g. `shoulder_offset` is
`0.064125` in both) — good, that part is internally consistent.

**But** the arm itself has drifted back to almost exactly its *original,
pre-fix* full size (`upper_length: 0.367875` now vs. `0.3648` in the very
first version I looked at — a 1% difference, i.e. essentially unscaled),
while the chassis is only 1.5× bigger than the small version I fitted it
to. That reintroduces the original problem: an arm sized for a ~1.1m
chassis, sitting on a chassis that's only ~0.42m long. This is very
likely contributing to instability/tipping and to the arm looking
disproportionate in the sim, independent of the PID issue.

**I did not rescale this for you this round** because you're actively
iterating on both packages' scale yourselves and I don't want to fight
your changes. If you want, tell me and I'll compute a single consistent
scale factor across `arm.xacro` + `ik.py` + every reach-dependent
parameter in the three `pick_place*.yaml` files (I did this exact pass
once already, when the chassis was smaller — happy to redo it against
your current 1.5×-scaled chassis: that works out to roughly **0.45×** of
the very first arm's original dimensions, not the ~1.0× it's at now).

---

## PART 2 — Cube-vs-gripper fit check (Issue you asked about explicitly)

Measured directly from `worlds/pharma.world`:

- `k12_red_cube`, `k12_green_cylinder` (modeled as a box despite the
  name), `k12_blue_block` — **all three are 0.07425 × 0.07425 × 0.07425 m
  cubes.**

Measured from your current (scaled-up) `description/arm.xacro`:

- Closed gap between fingertips: `k12_gripper_finger_gap = 0.02025 m`
- Each finger's max outward travel: `k12_gripper_finger_travel = 0.06075 m`
- **Max gripper opening: `0.02025 + 2 × 0.06075 = 0.14175 m`**

**Verdict: the cubes fit.** 0.07425m is well inside the 0.14175m max
opening (each finger only needs to travel ≈0.027m from closed, less than
half its available range), and `k12_gripper_finger_length = 0.091125m`
is longer than the cube, so the fingers can wrap around it without the
palm bumping the object first. No resizing needed on either side for
grasping to work geometrically — **as long as** the scale mismatch above
doesn't get "fixed" by shrinking the arm without also checking this ratio
again (if you rescale the arm down to 0.45×, the gripper opening becomes
`0.45/1.0 × 0.14175 ≈ 0.064m`, which would then be *too small* for a
0.07425m cube — so if we do the rescale, the cube models need to shrink
to match, or the gripper needs to stay at its current size while
everything else shrinks. Flagging this now so it isn't a surprise later.)

**A separate, real bug I found (not about fit, about placement math):**
`config/pick_place*.yaml` had `place_table_top_z: 0.15` and
`release_cube_height` / `release_cylinder_height` / `release_block_height`
all at `0.1125`. Measured from the world file directly: the actual table
collision top is at `z=0.10` (`<pose>0 0 0.09 ...</pose>` + half of
`0.02m` thickness), and the actual object height is `0.07425m`, not
`0.1125m`. The commander uses `place_table_top_z + 0.5×object_height +
clearance` to decide release height — with the old numbers it would have
aimed about 5–6 cm too high above the table, so placed objects would drop
and bounce instead of settling. **Fixed:** both now read `0.10` and
`0.07425` respectively in all three yaml files.

---

## PART 3 — File-by-file guide

### `articubot_one_with_arm` (the robot's body)

| File | What it does |
|---|---|
| `description/four_wheel.xacro` | Top-level robot description. Defines the chassis (`base_link`, from `base_link.stl`), the 4 wheels, camera/lidar/ultrasonic sensor links, and — near the bottom — instantiates the arm via `<xacro:k12_arm parent="base_link" mount_x=... mount_z=.../>`. This is where the arm's *position* on the rover is set. |
| `description/arm.xacro` | The K12 arm + gripper geometry: 5 revolute joints (`arm_base_yaw`, `arm_shoulder_pitch`, `arm_elbow_pitch`, `arm_wrist_pitch`, `arm_wrist_roll`) plus the parallel gripper (via `gripper.xacro`). All dimensions are `xacro:property` values at the top — this is the single place to resize the arm. |
| `description/gripper.xacro` | The parallel-jaw gripper macro (`end_effector`, `left_gripper`, `right_gripper` links + their prismatic joints). Takes palm/finger dimensions as macro parameters from `arm.xacro`. |
| `description/arm_gazebo.xacro` | Per-link Gazebo material/friction tags for the arm (`Gazebo/Silver` etc., friction coefficients) — cosmetic + grip friction, not motor control. |
| `description/arm_ros2_control.xacro` | **The file that actually controls arm holding torque.** Declares the `<ros2_control>` hardware block (`K12ArmSystem`) with one `<joint>` entry per arm/gripper joint, each with a `position` command interface and (after this fix) explicit PID gains. |
| `description/four_wheel.trans` | Same idea but for the 4 wheels (`FourWheelGazeboSystem`, velocity interfaces) — plus the actual `<gazebo><plugin>gz_ros2_control::GazeboSimROS2ControlPlugin` tag that loads the controller_manager and points it at `my_controllers.yaml` + `gaz_ros2_ctl_use_sim.yaml`. Only one of these plugin blocks should ever exist per robot (Ignition runs one controller_manager per model) — that's why the arm's ros2_control block lives in a separate file but is loaded by *this* plugin, not a second one. |
| `description/four_wheel.gazebo` | Sensor definitions: the RGBD camera (`camera_1`, topic `camera/depth_camera`), front/rear ultrasonic (modeled as narrow lidars), and the main lidar. |
| `description/materials.xacro` | Named color materials (`silver`, etc.) used by `<material name="...">` tags elsewhere. |
| `config/my_controllers.yaml` | `controller_manager` config: lists which controllers exist (`diff_cont`, `joint_broad`, `arm_position_controller`, `gripper_position_controller`) and each one's own parameters (joint lists, interfaces). |
| `config/gaz_ros2_ctl_use_sim.yaml` | Small supplementary file (your addition) that forces `use_sim_time: true` on every controller's namespace so they all read the Gazebo clock instead of the wall clock. |
| `config/gz_bridge.yaml` | `ros_gz_bridge` topic list — maps Gazebo-native topics (clock, scan, IMU, camera info, point cloud) onto ROS 2 topics of the same name. |
| `config/ekf.yaml` | `robot_localization` EKF config — fuses wheel odometry + IMU into a filtered `odom→base_footprint` transform. |
| `config/twist_mux.yaml` | Arbitrates between joystick / nav2 / autonomous `cmd_vel` sources so only one drives the robot at a time. |
| `launch/launch_sim.launch.py` | Standalone rover launch: RSP → Gazebo → spawn → controller spawners → bridges → EKF. Good for testing the rover + arm holding pose in isolation (no pick-and-place logic). |
| `launch/amms_gazebo.launch.py` | The file `k12_pick_place` expects from this package (originally missing entirely — that was last session's "package not found" error). Same idea as `launch_sim.launch.py` but doesn't start its own EKF node (leaves that to `k12_pick_place`'s navigation launch, to avoid two EKF nodes fighting over the same TF). |
| `launch/rsp.launch.py` | Just robot_state_publisher — runs `xacro` on `four_wheel.xacro` and publishes the result as `/robot_description` + the URDF's fixed-joint TFs. |
| `worlds/*.world` | Gazebo Sim world SDF files. `pharma.world` is the one you're using — contains the two tables and the three colored cube/block models. |

### `k12_pick_place` (perception + task logic)

| File | What it does |
|---|---|
| `k12_pick_place/object_detector.py` | Subscribes to the RGBD camera topics, does HSV color-thresholding to find red/green/blue blobs, uses the depth image + camera intrinsics to back-project each blob's pixel centroid into a 3D point, publishes `/amms/detections` and `/amms/detected_object_poses_camera`. |
| `k12_pick_place/pick_place_commander.py` | The brain (~1900 lines). State machine that: waits for a detection → computes an IK-reachable pre-grasp pose → sends arm trajectories via the `arm_position_controller` action → closes the gripper → holds the object (see grasp-assist note below) → drives to the place table → releases → retreats → returns home. Almost every yaml parameter you saw maps to a step in this file. |
| `k12_pick_place/ik.py` | Custom closed-form/planar IK solver (`ArmGeometry` dataclass + solver class) for the 5-DOF arm. Must stay dimensionally in sync with `arm.xacro` — currently is. |
| `k12_pick_place/readiness_gate.py` | Small node that waits for a set of prerequisite topics/services (controllers active, detections flowing, etc.) before signaling the rest of the pipeline it's safe to start — prevents the commander from acting before the sim has fully settled. |
| `k12_pick_place/waypoint_manager.py` | Holds/serves named navigation waypoints (pickup table, place table, home) for the nav2 stack, from `config/amms_waypoints.yaml`. |
| `k12_pick_place/amms_task_manager.py` | Higher-level orchestration node — sequences "navigate to pickup → pick → navigate to place → place" as one task, calling into `pick_place_commander`'s services (`/amms/pick_object`, `/amms/place_object`) rather than doing arm control itself. |
| `k12_pick_place/scene_spawner.py` | Verifies/spawns the table + object models at launch. **Still uses Gazebo Classic's `gazebo_msgs/GetEntityState`** — see pending item below. |
| `k12_pick_place/cloud_to_scan.py` | Converts a 3D point cloud (from the RGBD camera or lidar) into a 2D `LaserScan`, for feeding costmaps that expect 2D scans. |
| `config/pick_place*.yaml` (3 variants: `pick_place.yaml`, `pick_place_service.yaml`, `pick_place_standalone.yaml`) | Same parameter set, three launch contexts: `pick_place.yaml` for the combined nav+pick launch, `_service` for on-demand service calls, `_standalone` for testing the commander alone. Keep all three in sync if you change reach/height/grasp numbers — I updated all three for the table/cube-height fix above. |
| `config/nav2_amms.yaml`, `slam_toolbox_amms.yaml`, `pointcloud_to_laserscan.yaml`, `ekf_odom.yaml` | Standard nav2/SLAM stack configs — costmaps, planners, SLAM toolbox tuning, laser-scan conversion, and this package's own EKF (used when running full navigation, separate from `articubot_one`'s). |
| `config/amms_waypoints.yaml` | Named waypoint coordinates for `waypoint_manager.py`. |
| `launch/pick_place_demo.launch.py` | Simplest entry point: brings up the rover (via `articubot_one`'s `amms_gazebo.launch.py`) + spawns objects + runs the commander standalone. **Use this one first**, before the full nav+pick pipeline. |
| `launch/amms_mapping.launch.py` | Brings up the rover + SLAM toolbox for building a map (teleoperate around, save the map). |
| `launch/amms_navigation.launch.py` | Brings up the rover + nav2 (using a saved map) — no pick-and-place logic, just navigation. |
| `launch/amms_nav_pick_place.launch.py` | The full pipeline: navigation + pick-and-place combined. This is the one you tried first and got the `k12_description` error from — now fixed. |
| `models/k12_test_cube/`, `k12_test_block/`, `k12_test_cylinder/`, `k12_pick_table/` | Standalone Gazebo model folders (model.sdf + model.config) — an alternate way to spawn these objects individually rather than embedding them in the world file. Currently redundant with the cubes already embedded directly in `pharma.world`; not a problem, just two ways to get the same objects into the scene. |

---

## PART 4 — How to actually start pick-and-place, in order

1. **Rebuild clean** (you've changed a lot of files by hand across
   sessions — rule out stale build artifacts):
   ```
   cd ~/try_10   # or whichever workspace has both packages
   rm -rf build install log
   colcon build
   source install/setup.bash
   ```
2. **Test the rover + arm alone first:**
   ```
   ros2 launch articubot_one launch_sim.launch.py
   ```
   Confirm: robot spawns without tipping, and the arm now **holds its
   pose** instead of collapsing (this is the fix from Part 1). If it
   still sags, the PID values need tuning up, not a different mechanism —
   don't reach for another `ros2 param set` or yaml block; edit the
   `<param>` values directly in `arm_ros2_control.xacro`.
3. **Then test the full pick-place pipeline:**
   ```
   ros2 launch k12_pick_place pick_place_demo.launch.py
   ```
   Watch for the arm reaching toward a cube, closing the gripper, and
   check whether the cube now settles onto the place table instead of
   floating/bouncing (this is the height-config fix from Part 2).
4. **Only after both of those look right**, revisit the scale-mismatch
   question in Part 1 — decide whether to rescale the arm down to match
   the chassis (and shrink the cubes to match the smaller gripper
   opening) or scale the chassis mesh up instead. Tell me which direction
   and I'll do the full consistent pass across `arm.xacro` + `ik.py` + all
   three `pick_place*.yaml` files + the cube sizes in `pharma.world` in
   one go, rather than another round of partial rescales.
5. **Still outstanding, unrelated to today's three issues:** the grasp
   "hold" mechanism (`use_gazebo_grasp_assist`,
   `gazebo_set_entity_state_service: /set_entity_state`, etc. in the
   pick_place yaml files) and `scene_spawner.py`'s `GetEntityState` call
   still target **Gazebo Classic** services, which don't exist under
   Ignition/Gazebo Sim. The arm will now move and the gripper will close,
   but the object likely won't stay attached through a lift/carry motion
   yet — that port (to `ros_gz_interfaces/srv/SetEntityPose`) is the next
   piece after you confirm steps 2–3 above work.
