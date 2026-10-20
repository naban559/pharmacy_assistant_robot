"""Spawn the arm in Gazebo and start ros2_control controllers.

    ros2 launch robot_arm_description gazebo.launch.py                    # auto-detect from $ROS_DISTRO
    ros2 launch robot_arm_description gazebo.launch.py sim_backend:=ignition   # Humble + Ignition Fortress
    ros2 launch robot_arm_description gazebo.launch.py sim_backend:=gz         # Jazzy  + Gazebo Harmonic

Then, e.g.:
    ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
      "{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_pitch_joint], \
        points: [{positions: [0.5, 0.3, 0.6, 0.0], time_from_start: {sec: 3}}]}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'robot_arm_description'


def _has(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except Exception:
        return False


def _setup(context, *args, **kwargs):
    backend = LaunchConfiguration('sim_backend').perform(context)
    if backend == 'auto':
        distro = os.environ.get('ROS_DISTRO', 'humble')
        backend = 'ignition' if distro in ('foxy', 'galactic', 'humble') else 'gz'
    world = LaunchConfiguration('world').perform(context)

    pkg_share = get_package_share_directory(PKG)
    share_parent = os.path.dirname(pkg_share)          # lets Gazebo resolve package://robot_arm_description/...
    actions = []
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        current = os.environ.get(var, '')
        actions.append(SetEnvironmentVariable(var, os.pathsep.join(p for p in (current, share_parent) if p)))

    # ---- simulator + spawner + clock bridge (package names differ between installs) ----
    if _has('ros_gz_sim'):
        sim_pkg, bridge_pkg = 'ros_gz_sim', 'ros_gz_bridge'
        sim_launch, sim_arg = 'gz_sim.launch.py', 'gz_args'
    elif _has('ros_ign_gazebo'):
        sim_pkg, bridge_pkg = 'ros_ign_gazebo', 'ros_ign_bridge'
        sim_launch, sim_arg = 'ign_gazebo.launch.py', 'ign_args'
    else:
        raise RuntimeError('Neither ros_gz_sim nor ros_ign_gazebo is installed (see README).')
    msg_ns = 'gz.msgs' if backend == 'gz' else 'ignition.msgs'

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(sim_pkg), 'launch', sim_launch)),
        launch_arguments={sim_arg: f'-r {world}'}.items()))

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 PathJoinSubstitution([FindPackageShare(PKG), 'urdf', 'robot.urdf.xacro']),
                 ' sim_backend:=', backend,
                 ' debug_colors:=', LaunchConfiguration('debug_colors')]),
        value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package=sim_pkg, executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'robot_arm', '-z', '0.0'])

    clock = Node(package=bridge_pkg, executable='parameter_bridge', output='screen',
                 arguments=[f'/clock@rosgraph_msgs/msg/Clock[{msg_ns}.Clock'])

    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    arm = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['arm_controller', '--controller-manager', '/controller_manager'])

    actions += [
        rsp, clock, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_backend', default_value='auto',
                              description="auto | ignition (Humble/Fortress) | gz (Jazzy/Harmonic)"),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('debug_colors', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
