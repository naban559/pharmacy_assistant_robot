from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='77',
        description='ROS domain used to isolate this AMMS demo from stale Gazebo/DDS sessions.',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    run_task_manager_arg = DeclareLaunchArgument('run_task_manager', default_value='true')
    spawn_tables_arg = DeclareLaunchArgument('spawn_pick_place_tables', default_value='true')
    spawn_objects_arg = DeclareLaunchArgument('spawn_test_objects', default_value='true')
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Compatibility argument only. Use amms_mapping.launch.py for SLAM mapping.',
    )
    start_map_saver_arg = DeclareLaunchArgument(
        'start_map_saver',
        default_value='false',
        description='Compatibility argument only. Use amms_mapping.launch.py to save maps.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('articubot_one'),
            'worlds',
            'pharma.world',
        ]),
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'maps',
            'k12_warehouse.yaml',
        ]),
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'nav2_amms.yaml',
        ]),
    )
    scan_params_arg = DeclareLaunchArgument(
        'scan_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'pointcloud_to_laserscan.yaml',
        ]),
    )
    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'ekf_odom.yaml',
        ]),
    )
    pick_place_params_arg = DeclareLaunchArgument(
        'pick_place_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'pick_place_service.yaml',
        ]),
    )
    waypoint_params_arg = DeclareLaunchArgument(
        'waypoint_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'amms_waypoints.yaml',
        ]),
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'rviz',
            'amms_nav_pick_place.rviz',
        ]),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('k12_pick_place'),
                'launch',
                'amms_navigation.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'scan_params_file': LaunchConfiguration('scan_params_file'),
            'ekf_params_file': LaunchConfiguration('ekf_params_file'),
            'waypoint_params_file': LaunchConfiguration('waypoint_params_file'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
        }.items(),
    )

    scene_spawner = Node(
        package='k12_pick_place',
        executable='scene_spawner',
        name='amms_scene_spawner',
        parameters=[{
            'spawn_tables': ParameterValue(
                LaunchConfiguration('spawn_pick_place_tables'),
                value_type=bool,
            ),
            'spawn_objects': ParameterValue(
                LaunchConfiguration('spawn_test_objects'),
                value_type=bool,
            ),
        }],
        output='screen',
    )

    detector = Node(
        package='k12_pick_place',
        executable='object_detector',
        name='k12_object_detector',
        parameters=[LaunchConfiguration('pick_place_params_file')],
        output='screen',
    )

    commander = Node(
        package='k12_pick_place',
        executable='pick_place_commander',
        name='k12_pick_place_commander',
        parameters=[LaunchConfiguration('pick_place_params_file')],
        output='screen',
    )

    readiness_gate = Node(
        package='k12_pick_place',
        executable='readiness_gate',
        name='amms_readiness_gate',
        condition=IfCondition(LaunchConfiguration('run_task_manager')),
        parameters=[{
            'required_topics': [
                '/scan',
                '/odometry/filtered',
                '/amms/detections',
            ],
            'required_services': [
                '/amms/pick_object',
                '/amms/place_object',
                '/amms/fold_arm',
                '/controller_manager/list_controllers',
                # REMOVED: '/get_entity_state' -- this is a Gazebo Classic
                # (gazebo_msgs) service and does not exist under Ignition/
                # Gazebo Sim. Leaving it here would make this gate wait the
                # full 300s timeout on a service that can never appear,
                # blocking the task manager from ever starting. This is the
                # same still-pending Gazebo Classic -> Ignition grasp-assist
                # port noted elsewhere -- once that port is done, whatever
                # its real Ignition service turns out to be can be added
                # back here.
            ],
            'required_lifecycle_nodes': [
                '/map_server',
                '/amcl',
                '/controller_server',
                '/smoother_server',
                '/planner_server',
                '/behavior_server',
                '/bt_navigator',
                '/velocity_smoother',
            ],
            'required_actions': ['navigate_to_pose', 'drive_on_heading'],
            'required_trajectory_actions': [
                '/arm_position_controller/follow_joint_trajectory',
            ],
            'required_transforms': ['map<-base_link'],
            'timeout_sec': 300.0,
            'lifecycle_recovery_enabled': True,
            'lifecycle_recovery_initial_delay_sec': 20.0,
            'lifecycle_recovery_period_sec': 5.0,
            'lifecycle_transition_timeout_sec': 8.0,
        }],
        output='screen',
    )

    task_manager = Node(
        package='k12_pick_place',
        executable='amms_task_manager',
        name='amms_task_manager',
        condition=IfCondition(LaunchConfiguration('run_task_manager')),
        parameters=[LaunchConfiguration('waypoint_params_file')],
        output='screen',
    )

    return LaunchDescription([
        ros_domain_id_arg,
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            PathJoinSubstitution([
                FindPackageShare('k12_pick_place'),
                'models',
            ]),
        ),
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        run_task_manager_arg,
        spawn_tables_arg,
        spawn_objects_arg,
        slam_arg,
        start_map_saver_arg,
        world_arg,
        map_arg,
        nav2_params_arg,
        scan_params_arg,
        ekf_params_arg,
        pick_place_params_arg,
        waypoint_params_arg,
        rviz_config_arg,
        navigation,
        scene_spawner,
        detector,
        commander,
        readiness_gate,
        task_manager,
    ])
