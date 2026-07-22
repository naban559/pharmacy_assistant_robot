from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo client if true',
    )
    run_sequence_arg = DeclareLaunchArgument(
        'run_sequence',
        default_value='true',
        description='Start the autonomous pick-place commander if true',
    )
    spawn_objects_arg = DeclareLaunchArgument(
        'spawn_test_objects',
        default_value='true',
        description='Spawn the graspable test objects if true',
    )
    # ADDED: explicit world arg, forwarded straight through to
    # amms_gazebo.launch.py, instead of relying on that file's own default
    # via one extra layer of indirection. Fixes the intermittent case where
    # Gazebo's world-selection GUI falls back to its own "empty.sdf" prompt
    # if the world isn't resolved fast enough during the include chain.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('articubot_one'),
            'worlds',
            'pharma.world',
        ]),
        description='Full path to the Gazebo world file to load',
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('k12_pick_place'),
        'config',
        'pick_place_standalone.yaml',
    ])

    amms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('articubot_one'),
                'launch',
                'amms_gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'use_arm_ros2_control': 'true',
            'spawn_arm_controllers': 'true',
            'verbose': 'true',
        }.items(),
    )

    scene_spawner = Node(
        package='k12_pick_place',
        executable='scene_spawner',
        name='amms_scene_spawner',
        condition=IfCondition(LaunchConfiguration('spawn_test_objects')),
        parameters=[{
            'spawn_tables': True,
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
        parameters=[config_file],
        output='screen',
    )

    commander = Node(
        package='k12_pick_place',
        executable='pick_place_commander',
        name='k12_pick_place_commander',
        condition=IfCondition(LaunchConfiguration('run_sequence')),
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        gui_arg,
        run_sequence_arg,
        spawn_objects_arg,
        world_arg,
        amms_launch,
        scene_spawner,
        detector,
        commander,
    ])
