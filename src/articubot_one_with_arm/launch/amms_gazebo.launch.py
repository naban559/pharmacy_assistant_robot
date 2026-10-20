import os
import uuid

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    """
    Ignition-Gazebo equivalent of the k12_description/launch/amms_gazebo.launch.py
    that k12_pick_place's amms_navigation.launch.py expects. Spawns the merged
    rover+arm (four_wheel.xacro, now including the K12 arm/gripper) into
    Gazebo Sim, brings up the controller_manager with wheel + arm + gripper
    controllers, and bridges sensor topics to ROS.

    Deliberately does NOT start an EKF node here -- amms_navigation.launch.py
    starts its own ekf_filter_node with its own ekf_odom.yaml. Starting a
    second one here would fight over the odom->base_link TF.
    """

    package_name = 'articubot_one'

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )
    gazebo_partition_arg = DeclareLaunchArgument(
        'gazebo_partition',
        default_value='amms_' + uuid.uuid4().hex,
        description='Isolate Gazebo sensors, clock, spawning and services from other running worlds.',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'worlds',
            'og.world'
        ),
    )

    use_arm_ros2_control_arg = DeclareLaunchArgument(
        'use_arm_ros2_control',
        default_value='true'
    )

    spawn_arm_controllers_arg = DeclareLaunchArgument(
        'spawn_arm_controllers',
        default_value='true'
    )

    publish_static_map_to_odom_arg = DeclareLaunchArgument(
        'publish_static_map_to_odom',
        default_value='false'
    )

    publish_diff_drive_odom_tf_arg = DeclareLaunchArgument(
        'publish_diff_drive_odom_tf',
        default_value='false'
    )

    publish_front_diff_drive_odom_arg = DeclareLaunchArgument(
        'publish_front_diff_drive_odom',
        default_value='false'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true'
    )

    spawn_arm_controllers = LaunchConfiguration('spawn_arm_controllers')

    # ---------------------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------------------
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true',
            'sim_mode': 'true',
        }.items(),
    )

    # ---------------------------------------------------------------------
    # Twist mux
    # ---------------------------------------------------------------------
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            twist_mux_params,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')
        ],
    )

    # ---------------------------------------------------------------------
    # Gazebo
    # ---------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': [PythonExpression(["'-r -v4 ' if '", gui, "' == 'true' else '-r -s -v4 '"]), world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------------------------------------------------------------
    # Spawn robot
    # ---------------------------------------------------------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'four_wheel',
            '-z',
            '0.1',
        ],
        output='screen',
    )

    # ---------------------------------------------------------------------
    # Controller spawners
    # ---------------------------------------------------------------------
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller'],
        condition=IfCondition(spawn_arm_controllers),
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_position_controller'],
        condition=IfCondition(spawn_arm_controllers),
    )

    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                diff_drive_spawner,
                joint_broad_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    # ---------------------------------------------------------------------
    # ros_gz_bridge
    # ---------------------------------------------------------------------
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    )

    # ---------------------------------------------------------------------
    # RGBD image bridge
    # ---------------------------------------------------------------------
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera/depth_camera/image',
            '/camera/depth_camera/depth_image',
        ],
    )

    resource_path = os.path.dirname(get_package_share_directory('robot_arm_description'))

    return LaunchDescription([
        gazebo_partition_arg,
        SetEnvironmentVariable('IGN_PARTITION', LaunchConfiguration('gazebo_partition')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gazebo_partition')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        use_sim_time_arg,
        gui_arg,
        world_arg,
        use_arm_ros2_control_arg,
        spawn_arm_controllers_arg,
        publish_static_map_to_odom_arg,
        publish_diff_drive_odom_tf_arg,
        publish_front_diff_drive_odom_arg,
        verbose_arg,
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_controller_spawners,
        ros_gz_bridge,
        ros_gz_image_bridge,
    ])
