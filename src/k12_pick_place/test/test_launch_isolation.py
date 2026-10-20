"""Evaluate launch environments without starting ROS or Gazebo processes."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip('launch')
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


SOURCE = Path(__file__).resolve().parents[2]
LAUNCH_FILES = [
    SOURCE / 'k12_pick_place/launch/amms_nav_pick_place.launch.py',
    SOURCE / 'k12_pick_place/launch/amms_navigation.launch.py',
    SOURCE / 'articubot_one_with_arm/launch/amms_gazebo.launch.py',
]


def launch_environment(path, partition=None):
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = launch.LaunchContext()
    if partition is not None:
        context.launch_configurations['gazebo_partition'] = partition
    for action in module.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
            action.execute(context)
    return dict(context.environment)


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda path: path.name)
def test_independent_launches_do_not_share_gazebo_sensor_transport(path):
    first = launch_environment(path)
    second = launch_environment(path)
    assert first['IGN_PARTITION'] != second['IGN_PARTITION']
    assert first['IGN_PARTITION'] == first['GZ_PARTITION']
    assert second['IGN_PARTITION'] == second['GZ_PARTITION']


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda path: path.name)
def test_included_launch_uses_parent_partition_for_world_and_bridges(path):
    environment = launch_environment(path, 'amms_parent_session')
    assert environment['IGN_PARTITION'] == 'amms_parent_session'
    assert environment['GZ_PARTITION'] == 'amms_parent_session'
