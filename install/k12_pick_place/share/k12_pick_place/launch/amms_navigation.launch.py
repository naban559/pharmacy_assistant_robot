from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='77',
        description='ROS domain used to isolate the AMMS navigation demo from stale Gazebo/DDS sessions.',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
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

    amms_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('articubot_one'),
                'launch',
                'amms_gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_arm_ros2_control': 'true',
            'spawn_arm_controllers': 'true',
            'publish_static_map_to_odom': 'false',
            'publish_diff_drive_odom_tf': 'false',
            'publish_front_diff_drive_odom': 'false',
            'verbose': 'true',
        }.items(),
    )

    scan_converter = Node(
        package='k12_pick_place',
        executable='cloud_to_scan',
        name='pointcloud_to_laserscan',
        parameters=[
            LaunchConfiguration('scan_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            LaunchConfiguration('ekf_params_file'),
            {'publish_tf': True, 'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true',
        }.items(),
    )

    waypoint_manager = Node(
        package='k12_pick_place',
        executable='waypoint_manager',
        name='amms_waypoint_manager',
        parameters=[
            LaunchConfiguration('waypoint_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
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
        world_arg,
        map_arg,
        nav2_params_arg,
        scan_params_arg,
        ekf_params_arg,
        waypoint_params_arg,
        rviz_config_arg,
        amms_gazebo,
        scan_converter,
        ekf_odom,
        localization,
        navigation,
        waypoint_manager,
        rviz,
    ])
