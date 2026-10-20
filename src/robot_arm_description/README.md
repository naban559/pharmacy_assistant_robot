# robot_arm_description

Xacro/URDF description of a 4-DOF hobby-servo arm (base yaw, shoulder, elbow, wrist pitch), built from
the supplied STL meshes. **No gripper yet** — `wrist_link` is the last link.

```
robot_arm_description/
├── urdf/   robot.urdf.xacro  robot.ros2_control.xacro  robot.gazebo.xacro
├── meshes/ base.stl shoulder.stl upper_arm.stl fore_arm.stl wrist.stl   (millimetres; scaled 0.001)
├── config/ controllers.yaml  display.rviz
├── launch/ display.launch.py  gazebo.launch.py
├── tools/  analyze_meshes.py  verify_kinematics.py
└── docs/   MODEL_NOTES.md  (joint table, provenance, verification checklist)
```

## Build
```bash
cd ~/ros2_ws/src && cp -r /path/to/robot_arm_description .
cd ~/ros2_ws && colcon build --packages-select robot_arm_description && source install/setup.bash
```

## RViz (no simulator needed)
```bash
sudo apt install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-rviz2
ros2 launch robot_arm_description display.launch.py            # sliders move the 4 joints
ros2 launch robot_arm_description display.launch.py debug_colors:=false   # black-PLA look
```

## Gazebo + ros2_control
| Setup | Install | Launch |
|---|---|---|
| ROS 2 Humble + Ignition Fortress | `ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-controllers` | `ros2 launch robot_arm_description gazebo.launch.py sim_backend:=ignition` |
| ROS 2 Jazzy + Gazebo Harmonic | `ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers` | `ros2 launch robot_arm_description gazebo.launch.py sim_backend:=gz` |

`sim_backend` defaults to auto-detect from `$ROS_DISTRO`. Move the arm:
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
 "{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_pitch_joint],
   points: [{positions: [0.5, 0.3, 0.6, 0.0], time_from_start: {sec: 3}}]}"
```

## Offline check (no ROS needed)
```bash
pip install numpy scipy pillow trimesh python-fcl xacro
python3 tools/analyze_meshes.py        # reproduces every STL-derived number
python3 tools/verify_kinematics.py     # expands xacro, checks tree/inertias, collision-free joint windows
```

## Renaming the package
Replace `robot_arm_description` in `package.xml`, `CMakeLists.txt`, the `$(find …)`/`package://` strings in
`urdf/*.xacro`, and `PKG` in `launch/gazebo.launch.py`.

## Status
Geometry, kinematics, inertials and xacro expansion were verified offline. The launch files and the
ros2_control/Gazebo plugin wiring follow the standard Humble/Fortress and Jazzy/Harmonic patterns but
were **not run against a live ROS/Gazebo install** in the environment that produced this package.
Values that need checking on the real arm are listed in `docs/MODEL_NOTES.md`.
