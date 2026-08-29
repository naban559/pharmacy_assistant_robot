# AMMS Active Architecture File Inventory

This document details all files in the workspace that are **actively used** in the current **AMMS Navigation + Pick & Place Operation** (launched via `ros2 launch k12_pick_place amms_nav_pick_place.launch.py`).

Files not listed in the **Active Files** section below are standalone test scripts, legacy launch files, alternative maps, or unused configurations that can be safely archived or pruned for a clean architecture.

---

## 1. Active Launch Files

These launch files form the entry point and dependency chain for starting simulation, navigation, MoveIt, and pick-and-place task execution.

| Relative File Path | Role / Purpose | Invoked / Loaded By |
| :--- | :--- | :--- |
| [`src/k12_pick_place/launch/amms_nav_pick_place.launch.py`](file:///home/nabhan/try_15/src/k12_pick_place/launch/amms_nav_pick_place.launch.py) | **Main Top-Level Launch File**. Starts navigation sub-launch, perception, commanders, MoveIt `move_group`, readiness gate, and task manager. | Direct command (`ros2 launch`) |
| [`src/k12_pick_place/launch/amms_navigation.launch.py`](file:///home/nabhan/try_15/src/k12_pick_place/launch/amms_navigation.launch.py) | **Navigation & Simulation Bringup**. Includes Gazebo sim, pointcloud-to-laserscan, EKF node, Nav2 localization & navigation, waypoint manager, and RViz. | Included by `amms_nav_pick_place.launch.py` |
| [`src/articubot_one_with_arm/launch/amms_gazebo.launch.py`](file:///home/nabhan/try_15/src/articubot_one_with_arm/launch/amms_gazebo.launch.py) | **Gazebo Sim Bringup**. Spawns four-wheel rover + K12 arm into Ignition Gazebo, starts controller spawners, `twist_mux`, and ROS-GZ bridges. | Included by `amms_navigation.launch.py` |
| [`src/articubot_one_with_arm/launch/rsp.launch.py`](file:///home/nabhan/try_15/src/articubot_one_with_arm/launch/rsp.launch.py) | **Robot State Publisher**. Executes xacro processing on `four_wheel.xacro` and publishes the URDF `/robot_description` topic. | Included by `amms_gazebo.launch.py` |

---

## 2. Active ROS 2 Nodes & Python Source Code

Located in `src/k12_pick_place/k12_pick_place/`. These Python modules implement the runtime nodes, state machines, perception, and solvers.

| Relative File Path | Node / Module Name | Role / Purpose in Pipeline |
| :--- | :--- | :--- |
| [`src/k12_pick_place/k12_pick_place/amms_task_manager.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/amms_task_manager.py) | `amms_task_manager` | High-level mission state machine orchestrating navigation to pick station, object pick, navigation to place station, and place operation. |
| [`src/k12_pick_place/k12_pick_place/amms_command.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/amms_command.py) | CLI Tool / Execution Trigger | Command line utility used to send mission execution commands (e.g. `run_full_demo`, `pick`, `place`, `home`) to the task manager. |
| [`src/k12_pick_place/k12_pick_place/moveit_pick_place_commander.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/moveit_pick_place_commander.py) | `k12_moveit_pick_place_commander` | Pick-and-place service server using MoveIt / MoveGroup interfaces for trajectory planning and execution. |
| [`src/k12_pick_place/k12_pick_place/pick_place_commander.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/pick_place_commander.py) | `k12_pick_place_commander` | Alternative IK-based pick-and-place service server executing joint trajectory trajectories directly. |
| [`src/k12_pick_place/k12_pick_place/object_detector.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/object_detector.py) | `k12_object_detector` | Perception node converting RGBD camera depth/color data into 3D object detection bounding boxes and target grasp poses. |
| [`src/k12_pick_place/k12_pick_place/waypoint_manager.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/waypoint_manager.py) | `amms_waypoint_manager` | Serves target navigation coordinates (`home`, `pick_station`, `place_station`) via ROS 2 service. |
| [`src/k12_pick_place/k12_pick_place/readiness_gate.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/readiness_gate.py) | `amms_readiness_gate` | Pre-flight verification node waiting for all required topics (`/scan`, `/odometry/filtered`, etc.), services, and Nav2 lifecycle nodes before releasing task manager. |
| [`src/k12_pick_place/k12_pick_place/cloud_to_scan.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/cloud_to_scan.py) | `pointcloud_to_laserscan` | Converts 3D PointCloud2 data into 2D `/scan` LaserScan messages for Nav2 costmaps. |
| [`src/k12_pick_place/k12_pick_place/ik.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/ik.py) | Kinematics Helper Library | Analytical/numerical 5-DOF inverse kinematics library used by pick and place commanders. |
| [`src/k12_pick_place/k12_pick_place/__init__.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/__init__.py) | Python Init | Python package module initializer. |

---

## 3. Active Configuration Files

Parameters governing Nav2, localization, controllers, MoveIt, vision, and RViz visualization.

| Relative File Path | Loaded By / Used For |
| :--- | :--- |
| [`src/k12_pick_place/config/nav2_amms.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/nav2_amms.yaml) | **Nav2 Stack Configuration**. Planner server, Controller server, AMCL localization, Behavior server, BT Navigator. |
| [`src/k12_pick_place/config/amms_waypoints.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/amms_waypoints.yaml) | **Waypoint Definitions**. Defines coordinates and orientations for `home`, `pick_station`, `place_station`. Loaded by `waypoint_manager` and `amms_task_manager`. |
| [`src/k12_pick_place/config/ekf_odom.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/ekf_odom.yaml) | **Robot Localization EKF**. Configuration for `ekf_filter_node` fusing wheel odometry and IMU into continuous `odom->base_link` TF. |
| [`src/k12_pick_place/config/pointcloud_to_laserscan.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/pointcloud_to_laserscan.yaml) | **Laser Scan Filter Parameters**. Height limits, angle increment, and range parameters for `cloud_to_scan`. |
| [`src/k12_pick_place/config/pick_place_service.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/pick_place_service.yaml) | **Perception & Grasp Config**. HSV thresholds, camera intrinsic offsets, grasp approach/retreat distances, pre-grasp/fold joint angles. |
| [`src/k12_pick_place/config/moveit_nav_pick_place.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/config/moveit_nav_pick_place.yaml) | **MoveIt Commander Parameters**. Service parameters for `moveit_pick_place_commander`. |
| [`src/articubot_one_with_arm/config/my_controllers.yaml`](file:///home/nabhan/try_15/src/articubot_one_with_arm/config/my_controllers.yaml) | **ros2_control Controllers Config**. Defines `diff_cont` (diff drive), `joint_broad` (joint states), `arm_position_controller`, and `gripper_position_controller`. |
| [`src/articubot_one_with_arm/config/gz_bridge.yaml`](file:///home/nabhan/try_15/src/articubot_one_with_arm/config/gz_bridge.yaml) | **Ignition Gazebo ROS Bridge Config**. Bridges `/clock`, `/scan`, `/tf`, `/joint_states`, `/imu`, `/camera/depth_camera/*` between Gazebo and ROS 2. |
| [`src/articubot_one_with_arm/config/gaz_ros2_ctl_use_sim.yaml`](file:///home/nabhan/try_15/src/articubot_one_with_arm/config/gaz_ros2_ctl_use_sim.yaml) | **Gazebo ROS2 Control Sim Param**. Configures `gz_ros2_control` plugin to use sim time. |
| [`src/articubot_one_with_arm/config/twist_mux.yaml`](file:///home/nabhan/try_15/src/articubot_one_with_arm/config/twist_mux.yaml) | **Twist Mux Config**. Velocity topic priority routing (`Nav2` vs `joystick`). |
| [`src/k12_pick_place/rviz/amms_nav_pick_place.rviz`](file:///home/nabhan/try_15/src/k12_pick_place/rviz/amms_nav_pick_place.rviz) | **RViz Display Configuration**. RViz setup for map, TF, robot model, costmaps, camera feed, 3D object markers. |

---

## 4. Robot URDF / XACRO Description & 3D Meshes

Located in `src/articubot_one_with_arm/description/` and `src/articubot_one_with_arm/meshes/`.

| Relative File Path | Role / Purpose |
| :--- | :--- |
| [`src/articubot_one_with_arm/description/four_wheel.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.xacro) | **Top-Level Robot Xacro**. Assembles mobile base chassis, wheels, camera with optical frame, LiDAR, ultrasonic sensors, and mounts the 5-DOF K12 arm. |
| [`src/articubot_one_with_arm/description/four_wheel.gazebo`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.gazebo) | **Gazebo Sensors & Dynamics**. Configures Gazebo RGBD camera sensor, GPU LiDAR, IMU, ultrasonic sensors, and anisotropic wheel friction (`mu1=10.0`, `mu2=0.1`). |
| [`src/articubot_one_with_arm/description/four_wheel.trans`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/four_wheel.trans) | **ros2_control Wheel Interfaces**. Defines `FourWheelGazeboSystem` for 4 drive wheels and attaches `GazeboSimROS2ControlPlugin`. |
| [`src/articubot_one_with_arm/description/arm.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm.xacro) | **5-DOF Robotic Arm Definition**. Arm links (link1 to link5), joint limits, physical dimensions, inertia macros (`k12_box_inertial`, `k12_cylinder_z_inertial`). |
| [`src/articubot_one_with_arm/description/arm_gazebo.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm_gazebo.xacro) | **Arm Gazebo Material/Physics**. Gazebo self-collision and material properties for arm links. |
| [`src/articubot_one_with_arm/description/arm_ros2_control.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/arm_ros2_control.xacro) | **ros2_control Arm & Gripper Hardware**. Defines `K12ArmSystem` position command interfaces with tuned PD gains (`position_proportional_gain`, `position_derivative_gain`). |
| [`src/articubot_one_with_arm/description/gripper.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/gripper.xacro) | **Parallel Gripper Definition**. End-effector palm, left/right gripper fingers, linear slider joints, grasp collision bounds. |
| [`src/articubot_one_with_arm/description/materials.xacro`](file:///home/nabhan/try_15/src/articubot_one_with_arm/description/materials.xacro) | **Visual Materials**. Color definitions (silver, black, blue, red) for RViz rendering. |
| [`src/articubot_one_with_arm/meshes/*.stl`](file:///home/nabhan/try_15/src/articubot_one_with_arm/meshes/) | **Chassis & Sensor Meshes**. `base_link.stl`, `camera_1.stl`, `lidar_1.stl`, `left_front_1.stl`, `right_front_1.stl`, `left_rear_1.stl`, `right_rear_1.stl`, `ultra_front_1.stl`, `ultra_rear_1.stl`. |

---

## 5. Active World, Map, and Model Files

Environment and Gazebo model files loaded during execution.

| Relative File Path | Role / Purpose |
| :--- | :--- |
| [`src/articubot_one_with_arm/worlds/og.world`](file:///home/nabhan/try_15/src/articubot_one_with_arm/worlds/og.world) | **Active Gazebo Simulation World**. Simulation environment world containing ground plane, ambient lighting, and physics engine setup. |
| [`src/k12_pick_place/maps/og.yaml`](file:///home/nabhan/try_15/src/k12_pick_place/maps/og.yaml) | **Nav2 Map YAML Descriptor**. Defines map resolution (`0.05`), origin, free/occupied threshold for map server. |
| [`src/k12_pick_place/maps/og.pgm`](file:///home/nabhan/try_15/src/k12_pick_place/maps/og.pgm) | **Nav2 Occupancy Grid Image**. 2D bitmap representation of the obstacle map. |
| [`src/k12_pick_place/models/k12_pick_table/`](file:///home/nabhan/try_15/src/k12_pick_place/models/k12_pick_table/) | **Pick/Place Table Model**. `model.sdf` & `model.config` for physical tables spawned by `scene_spawner`. |
| [`src/k12_pick_place/models/k12_test_block/`](file:///home/nabhan/try_15/src/k12_pick_place/models/k12_test_block/) | **Test Target Object Model**. `model.sdf` & `model.config` for pickable target box spawned on table. |
| [`src/k12_pick_place/models/k12_test_cube/`](file:///home/nabhan/try_15/src/k12_pick_place/models/k12_test_cube/) & [`k12_test_cylinder/`](file:///home/nabhan/try_15/src/k12_pick_place/models/k12_test_cylinder/) | **Alternative Target Object Models**. SDF models for cube and cylinder objects. |

---

## 6. Active MoveIt Configuration Files

Located in `src/k12_moveit_config/`.

| Relative File Path | Loaded By / Purpose |
| :--- | :--- |
| [`src/k12_moveit_config/config/k12.srdf`](file:///home/nabhan/try_15/src/k12_moveit_config/config/k12.srdf) | **Semantic Robot Description (SRDF)**. Loaded by `move_group` in `amms_nav_pick_place.launch.py`. Defines `arm` planning group, `gripper` end-effector, named states (`home`, `pre_grasp`, `fold`), and self-collision matrix. |
| [`src/k12_moveit_config/config/ompl_planning.yaml`](file:///home/nabhan/try_15/src/k12_moveit_config/config/ompl_planning.yaml) / [`ompl.yaml`](file:///home/nabhan/try_15/src/k12_moveit_config/config/ompl.yaml) | **OMPL Motion Planner Config**. Defines planners (RRTConnect, RRTstar, PRMstar) and projection evaluators for the `arm` group. |
| [`src/k12_moveit_config/config/kinematics.yaml`](file:///home/nabhan/try_15/src/k12_moveit_config/config/kinematics.yaml) | **Kinematics Plugin Config**. Configures `kdl_kinematics_plugin/KDLKinematicsPlugin` solver parameters for the `arm` group. |
| [`src/k12_moveit_config/config/joint_limits.yaml`](file:///home/nabhan/try_15/src/k12_moveit_config/config/joint_limits.yaml) | **Joint Speed & Acceleration Limits**. Velocity and acceleration upper bounds for arm joints. |
| [`src/k12_moveit_config/config/controllers.yaml`](file:///home/nabhan/try_15/src/k12_moveit_config/config/controllers.yaml) | **MoveIt Controller Manager Config**. Configures FollowJointTrajectory action namespace for `arm_position_controller`. |
| [`src/k12_moveit_config/package.xml`](file:///home/nabhan/try_15/src/k12_moveit_config/package.xml) & [`CMakeLists.txt`](file:///home/nabhan/try_15/src/k12_moveit_config/CMakeLists.txt) | **Package Build System**. ROS 2 package manifest and installation rules for `k12_moveit_config`. |

---

## 7. Package Manifests & Build Files

| Relative File Path | Role |
| :--- | :--- |
| [`src/k12_pick_place/package.xml`](file:///home/nabhan/try_15/src/k12_pick_place/package.xml) | ROS 2 package manifest for `k12_pick_place`. |
| [`src/k12_pick_place/setup.py`](file:///home/nabhan/try_15/src/k12_pick_place/setup.py) | Python package build & entry point declaration script. |
| [`src/k12_pick_place/setup.cfg`](file:///home/nabhan/try_15/src/k12_pick_place/setup.cfg) | Python package installation configuration. |
| [`src/k12_pick_place/resource/k12_pick_place`](file:///home/nabhan/try_15/src/k12_pick_place/resource/k12_pick_place) | ament index marker file. |
| [`src/articubot_one_with_arm/package.xml`](file:///home/nabhan/try_15/src/articubot_one_with_arm/package.xml) | ROS 2 package manifest for `articubot_one`. |
| [`src/articubot_one_with_arm/CMakeLists.txt`](file:///home/nabhan/try_15/src/articubot_one_with_arm/CMakeLists.txt) | CMake build rules and resource installation for `articubot_one`. |

---

## 8. Removed Legacy / Inactive Files (Cleanup Completed)

All legacy launch files, unused config files, alternative maps/worlds, redundant URDFs, test scripts, and static models (`src/k12_pick_place/models/` containing `k12_pick_table`, `k12_test_block`, `k12_test_cube`, `k12_test_cylinder`) have been **permanently removed** from the workspace. Model generation for simulation spawning is now handled dynamically on the fly by [`scene_spawner.py`](file:///home/nabhan/try_15/src/k12_pick_place/k12_pick_place/scene_spawner.py), leaving a minimal, clean architecture containing only the active pipeline files.
