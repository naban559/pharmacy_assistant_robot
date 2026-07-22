import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'articubot_one'

    # ── Robot State Publisher ─────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={
            'use_sim_time':     'true',
            'use_ros2_control': 'true',
            'sim_mode':         'true',
        }.items()
    )

    # ── Joystick ──────────────────────────────────────────────────────────
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ── Twist Mux ─────────────────────────────────────────────────────────
    # Output remapped to /diff_cont/cmd_vel_unstamped consumed by ros2_control
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # ── World argument ────────────────────────────────────────────────────
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'pharma.world'
    )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='SDF world file to load in Ignition Gazebo'
    )

    # ── Ignition Gazebo ───────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={
            'gz_args':          ['-r -v4 ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── Spawn robot ───────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'four_wheel',
            '-z',     '0.1',
        ],
        output='screen'
    )

    # ── Controller spawners — delayed until robot is in Gazebo ───────────
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )

    # ADDED: these two were missing entirely. Without spawning them, no
    # controller ever claims the arm/gripper joints' position command
    # interfaces -- so raising position_proportional_gain in
    # arm_ros2_control.xacro has nothing to act on, and the joints stay
    # uncommanded and fall under gravity regardless of the gain value.
    # This, not the PID gains, is why the arm was still collapsing.
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller'],
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_position_controller'],
    )
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[arm_controller_spawner],
        )
    )
    delayed_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[gripper_controller_spawner],
        )
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────
    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # ── Camera image/depth bridge ─────────────────────────────────────────
    # CHANGED: was ['/camera/image_raw'], left over from before the camera
    # sensor became type="rgbd_camera" on topic "camera/depth_camera" (see
    # four_wheel.gazebo). object_detector.py needs both color and depth.
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera/depth_camera/image',
            '/camera/depth_camera/depth_image',
        ]
    )

    # ── EKF node ──────────────────────────────────────────────────────────
    ekf_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_arm_controller_spawner,
        delayed_gripper_controller_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        ekf_node,
    ])
