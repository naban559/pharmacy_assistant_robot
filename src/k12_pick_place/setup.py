from glob import glob
from setuptools import setup

package_name = 'k12_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vijay',
    maintainer_email='vijaykumargorli2003@gmail.com',
    description='Autonomous RGB-D pick-and-place demo layer for the K12 AMMS.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = k12_pick_place.object_detector:main',
            'pick_place_commander = k12_pick_place.pick_place_commander:main',
            'moveit_pick_place_commander = k12_pick_place.moveit_pick_place_commander:main',
            'cloud_to_scan = k12_pick_place.cloud_to_scan:main',
            'waypoint_manager = k12_pick_place.waypoint_manager:main',
            'amms_task_manager = k12_pick_place.amms_task_manager:main',
            'amms_command = k12_pick_place.amms_command:main',
            'readiness_gate = k12_pick_place.readiness_gate:main',
        ],
    },
)
