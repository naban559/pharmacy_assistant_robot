import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='77',
        description='ROS domain used to isolate the AMMS navigation demo from stale Gazebo/DDS sessions.',
    )
    gazebo_partition_arg = DeclareLaunchArgument(
        'gazebo_partition',
        default_value='amms_' + uuid.uuid4().hex,
        description='Gazebo Transport partition shared by this world, its bridges and entity services.',
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('articubot_one'),
            'worlds',
            'og.world',
        ]),
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'maps',
            'og.yaml',
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
    use_cloud_to_scan_arg = DeclareLaunchArgument(
        'use_cloud_to_scan',
        default_value='false',
        description=(
            'Enable only for a replacement point-cloud lidar. The default '
            'Ignition robot already bridges its 360-degree LaserScan to /scan; '
            'do not publish a second scan source on the same topic.'
        ),
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
            'gazebo_partition': LaunchConfiguration('gazebo_partition'),
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
        condition=IfCondition(LaunchConfiguration('use_cloud_to_scan')),
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

    navigation_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file'),
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': PathJoinSubstitution([
            FindPackageShare('k12_pick_place'), 'config', 'navigate.xml'])},
        convert_types=True,
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
            'params_file': navigation_params,
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
        gazebo_partition_arg,
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        # ROS domains do not isolate Gazebo Transport. Keep stale worlds from
        # feeding this run's clock, camera and lidar bridges.
        SetEnvironmentVariable('IGN_PARTITION', LaunchConfiguration('gazebo_partition')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gazebo_partition')),
        LogInfo(msg=['AMMS Gazebo partition: ', LaunchConfiguration('gazebo_partition')]),
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
        use_cloud_to_scan_arg,
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
