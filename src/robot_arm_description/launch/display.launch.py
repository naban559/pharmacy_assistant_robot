"""RViz-only visualisation: robot_state_publisher + joint_state_publisher_gui + RViz.

    ros2 launch robot_arm_description display.launch.py
    ros2 launch robot_arm_description display.launch.py debug_colors:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'robot_arm_description'
    xacro_file = PathJoinSubstitution([FindPackageShare(pkg), 'urdf', 'robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([FindPackageShare(pkg), 'config', 'display.rviz'])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' debug_colors:=', LaunchConfiguration('debug_colors'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('debug_colors', default_value='true',
                              description='Colour each link differently (false = all black)'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
