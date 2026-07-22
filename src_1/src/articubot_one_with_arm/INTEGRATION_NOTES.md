# articubot_one + K12 arm integration notes

## What was changed in this package (articubot_one)

- `description/arm.xacro`, `gripper.xacro`, `arm_gazebo.xacro`,
  `arm_ros2_control.xacro` -- new. The K12 5-DOF arm + parallel gripper from
  k12_pick_place, **rescaled to 0.30x** so it's proportionate to this
  rover's actual chassis (0.28 x 0.20 x 0.06 m) instead of the ~4x larger
  chassis it was originally designed for.
- `description/four_wheel.xacro` -- now includes the arm files and mounts
  the arm on top of `base_link` (centered, z=0.06).
- `description/four_wheel.gazebo` -- camera sensor changed from
  `type="camera"` (RGB only) to `type="rgbd_camera"` so the pick-place
  perception node gets depth data. Topic renamed to `camera/depth_camera`.
- `config/my_controllers.yaml` -- added `arm_position_controller`
  (JointTrajectoryController) and `gripper_position_controller`
  (ForwardCommandController) to the *same* controller_manager as the
  wheels. Ignition only starts one controller_manager per model, so the
  arm's ros2_control block loads through the existing
  `gz_ros2_control::GazeboSimROS2ControlPlugin` in `four_wheel.trans`
  rather than a second plugin instance.
- `config/gz_bridge.yaml` -- updated camera bridge entries for the new
  RGBD topic names.
- `launch/amms_gazebo.launch.py` -- new. This is the file
  `k12_pick_place`'s `amms_navigation.launch.py` was looking for from a
  package called `k12_description`, which didn't exist in your
  workspace(s). It does NOT start an EKF node (amms_navigation.launch.py
  starts its own).
- `package.xml` -- added `joint_trajectory_controller` and
  `forward_command_controller` deps.

## What was changed in k12_pick_place

- `package.xml` and all `launch/*.py` -- `k12_description` ->
  `articubot_one`.
- `k12_pick_place/ik.py` -- `ArmGeometry` constants rescaled to match the
  0.30x arm above. **These two files must stay in sync** if you resize the
  arm again.
- `config/pick_place*.yaml` -- `rgb_topic`/`depth_topic` updated to
  `/camera/depth_camera/image` and `/camera/depth_camera/depth_image`
  (Ignition's rgbd_camera default subtopic names, via `ros_gz_image`).
  `approach_distance`, `lift_height`, `place_lift_height`,
  `place_retreat_lift_height`, `reachable_min/max_radius`,
  `reachable_min/max_z`, `best_effort_max_radius` all scaled by 0.30 to
  match the smaller arm's real reach (~0.31 m).
- World file references (`k12_warehouse.world`) changed to `pharma.world`
  since that's what actually exists in articubot_one's `worlds/` -- pick a
  different one if you'd rather.

## What is NOT done yet -- the biggest remaining piece

`pick_place_commander.py`'s "hold" logic (`use_gazebo_grasp_assist`,
`_hold_gripper_closed`, disable-gravity-while-holding, live grasp-offset
tracking, release) calls **Gazebo Classic** services:
`/set_entity_state`, `/get_entity_state`, `/get_link_properties`,
`/set_link_properties` (`gazebo_msgs`). None of these exist under
Ignition/Gazebo Sim -- every one of those calls will fail or time out, so
picking will look like it succeeds (arm moves, gripper closes) but the
object won't actually stay attached/held.

I did not rewrite this yet because it's the riskiest part (it's the core
grasp behavior, ~10 call sites across `pick_place_commander.py`) and I
have no way to run Gazebo here to verify a rewrite actually works. The
Ignition-native replacement is the `ros_gz_interfaces/srv/SetEntityPose`
service (typically at `/world/<world_name>/set_pose`), which can do the
same "teleport the held object to follow the gripper" trick the current
code already does every 20 ms (`held_object_update_period_sec`) -- so the
live-pose-follow mechanism carries over conceptually, it just needs new
service types. Gravity-disable-while-holding doesn't have a direct
Ignition equivalent via a simple service call; since the object's pose is
already being force-set every 20 ms while held, this may not matter much
in practice.

Once you've confirmed the robot spawns and the arm/gripper move
correctly, come back and I'll do this port -- it's much safer to do once
we know the base spawn actually works than to guess at both at once.

## How to test, in order

1. `colcon build` -- should now find both `articubot_one` and
   `k12_pick_place` with no missing-package errors.
2. `ros2 launch articubot_one launch_sim.launch.py` -- confirm the rover
   spawns, drives, and now visibly has the arm on top without tipping
   over.
3. `ros2 launch k12_pick_place pick_place_demo.launch.py` -- confirm the
   arm/gripper controllers spawn (`ros2 control list_controllers`) and
   the arm moves through its trajectory.
4. Only after (2) and (3) look right: test the actual pick/place/hold
   sequence, and expect the object to NOT stay attached (per the "not
   done yet" section above) until the grasp-assist port is done.

Send me the output/errors from whichever step breaks first -- that'll
tell us exactly what to fix next rather than guessing further ahead.
