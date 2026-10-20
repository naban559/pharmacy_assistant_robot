"""Independent world/config/URDF regression for the usable docking envelope."""
import importlib.util
from pathlib import Path
import random

import numpy as np
import pytest
import yaml

TOOL = Path(__file__).resolve().parents[1] / 'tools/verify_station_workspace.py'
SPEC = importlib.util.spec_from_file_location('station_workspace', TOOL)
WORKSPACE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKSPACE)


def test_all_cubes_are_centered_and_supported_by_equal_size_stations():
    geometry = WORKSPACE.StationGeometry()
    _, placing_size = geometry.model_geometry('placing_station')
    for index, color in enumerate(('red', 'green', 'blue'), 1):
        support, size = geometry.model_geometry(f'pickup_point_{index}_{color}')
        cube, cube_size = geometry.model_geometry(f'cube_{color}')
        assert size == pytest.approx(placing_size)
        assert cube[:2] == pytest.approx(support[:2])
        assert cube[2] - cube_size[2] / 2 == pytest.approx(support[2] + size[2] / 2)
    placing, _ = geometry.model_geometry('placing_station')
    assert geometry.commander['place_table_top_z'] == pytest.approx(placing[2] + placing_size[2] / 2)


def test_ik_geometry_matches_actual_xml_origins_and_joint_axes():
    geometry = WORKSPACE.StationGeometry()
    rng = random.Random(43)
    for _ in range(50):
        joints = [rng.uniform(lo, hi) for lo, hi in geometry.ik.limits.ordered()]
        assert geometry.independent_fk(joints) == pytest.approx(geometry.ik.forward(joints), abs=1e-12)


def test_tight_docking_envelope_fits_arm_camera_and_full_wheel_extent():
    nav = yaml.safe_load((TOOL.parents[1] / 'config/nav2_amms.yaml').read_text())
    controller = nav['controller_server']['ros__parameters']
    checker = controller['grasp_goal_checker']
    assert controller['FineApproach']['xy_goal_tolerance'] == checker['xy_goal_tolerance']
    result = WORKSPACE.verify(trials=0, translation=checker['xy_goal_tolerance'],
                              yaw=checker['yaw_goal_tolerance'])
    assert result['passed'], result['failures']
    assert sum(case['passed'] for case in result['cases'].values()) == 162
    for case in result['cases'].values():
        assert case['minimum_wheel_clearance_m'] > .02
        assert case['minimum_chassis_clearance_m'] > case['minimum_wheel_clearance_m']
        assert case['maximum_fk_error_m'] < 1e-9


def test_navigation_tolerance_cannot_be_widened_without_losing_reach_or_clearance():
    result = WORKSPACE.verify(trials=0, translation=.025, yaw=.05)
    assert not result['passed']
    assert any('IK=False' in failure for failure in result['failures'])
    assert any(case['minimum_wheel_clearance_m'] < .01 for case in result['cases'].values())


def test_open_gripper_tolerance_and_table_clearance_are_millimetres():
    geometry = WORKSPACE.StationGeometry()
    _, cube_size = geometry.model_geometry('cube_red')
    props = geometry.properties
    gap = props['k12_gripper_finger_gap'] + 2 * props['k12_gripper_finger_travel']
    assert (gap - cube_size[1]) / 2 == pytest.approx(.0025)
    finger_to_table = cube_size[2] / 2 + geometry.task['object_grasp_z_bias'] - props['finger_length'] / 2
    assert finger_to_table == pytest.approx(.0025)
    assert np.isfinite(finger_to_table)
