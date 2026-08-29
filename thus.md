# Workspace Summary: try_15

## Overview
ROS 2 workspace for a mobile manipulator (articubot_one_with_arm) performing pick-and-place tasks in a Gazebo simulation with MoveIt 2.

## Packages
| Package | Purpose |
|---------|---------|
| `articubot_one_with_arm` | Robot URDF/Xacro (4-wheel chassis + 5-DOF arm + gripper), Gazebo sim, controllers, Nav2 |
| `k12_pick_place` | Perception, pick/place logic, task orchestration, scene spawning |
| `k12_moveit_config` | MoveIt 2 configuration (SRDF, kinematics, planners) |

## Key Files
- **Main Launch:** `amms_nav_pick_place.launch.py`
- **Diagnostic Report:** `wed.md` (detailed analysis of placement failures)
- **Nav2 Config:** `config/nav2_amms.yaml`
- **Pick/Place Config:** `config/pick_place_service.yaml`

## Current Task
Pick blue rectangular block from pick table → Place on Shelf D

## Known Issues (from wed.md)
1. Release XY position is off Shelf D (0.09m in front)
2. Place Z vs Release Z mismatch causes downward pushing
3. MoveIt PlanningScene Shelf D geometry mismatch
4. "place_table" config aliases Shelf D, not physical table

## Recommendations
- Fix `shelf_d_position` to center [5.0, -2.5, 0.0]
- Adjust release XY offset
- Align Place Z with Release Z
- Update MoveIt collision objects
