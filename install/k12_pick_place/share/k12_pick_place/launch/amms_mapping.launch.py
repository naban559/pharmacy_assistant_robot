from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    spawn_tables_arg = DeclareLaunchArgument(
        'spawn_pick_place_tables',
        default_value='false',
        description='Spawn static pick/place tables while manually building the map',
    )
    spawn_objects_arg = DeclareLaunchArgument(
        'spawn_test_objects',
        default_value='false',
        description='Compatibility argument; mapping does not spawn movable test objects by default',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('articubot_one'),
            'worlds',
            'pharma.world',
        ]),
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'slam_toolbox_amms.yaml',
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
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('k12_pick_place'),
            'config',
            'nav2_amms.yaml',
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

    scene_spawner = Node(
        package='k12_pick_place',
        executable='scene_spawner',
        name='amms_scene_spawner',
        condition=IfCondition(LaunchConfiguration('spawn_pick_place_tables')),
        parameters=[{
            'spawn_tables': ParameterValue(
                LaunchConfiguration('spawn_pick_place_tables'),
                value_type=bool,
            ),
            'spawn_objects': False,
        }],
        output='screen',
    )

    scan_converter = Node(
        package='k12_pick_place',
        executable='cloud_to_scan',
        name='pointcloud_to_laserscan',
        parameters=[LaunchConfiguration('scan_params_file')],
        output='screen',
    )

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('ekf_params_file')],
        output='screen',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    map_saver = [
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            parameters=[LaunchConfiguration('nav2_params_file')],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_saver']},
            ],
            output='screen',
        ),
    ]

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
        spawn_tables_arg,
        spawn_objects_arg,
        world_arg,
        slam_params_arg,
        scan_params_arg,
        ekf_params_arg,
        nav2_params_arg,
        rviz_config_arg,
        amms_gazebo,
        scene_spawner,
        scan_converter,
        ekf_odom,
        slam_toolbox,
        *map_saver,
        rviz,
    ])
