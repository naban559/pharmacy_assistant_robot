import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    laser_filter_config = os.path.join(
        get_package_share_directory('articubot_one'),
        'config', 'laser_filter.yaml'
    )

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'frame_id': 'lidar_1',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            remappings=[('/scan', '/scan_raw')]
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            output='screen',
            parameters=[laser_filter_config],
            remappings=[
                ('/scan', '/scan_raw'),
                ('/scan_filtered', '/scan')
            ]
        )
    ])
