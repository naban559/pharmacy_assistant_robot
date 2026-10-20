import os
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Convenience launch file for bringing up Gazebo Sim + Navigation + MoveIt + MoveItPickPlaceCommander.
    Includes amms_nav_pick_place.launch.py with use_moveit_commander:=true.
    """
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    run_task_manager_arg = DeclareLaunchArgument('run_task_manager', default_value='true')
    ros_domain_id_arg = DeclareLaunchArgument('ros_domain_id', default_value='77')
    gazebo_partition_arg = DeclareLaunchArgument(
        'gazebo_partition', default_value='amms_' + uuid.uuid4().hex)

    nav_pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('k12_pick_place'),
                'launch',
                'amms_nav_pick_place.launch.py',
            ])
        ]),
        launch_arguments={
            'use_moveit_commander': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'run_task_manager': LaunchConfiguration('run_task_manager'),
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'gazebo_partition': LaunchConfiguration('gazebo_partition'),
        }.items(),
    )

    return LaunchDescription([
        ros_domain_id_arg,
        gazebo_partition_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        run_task_manager_arg,
        nav_pick_place,
    ])
