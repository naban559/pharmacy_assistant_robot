"""Unit tests for GazeboObserver and physical placement verification logic."""

import time
import pytest
from k12_pick_place.gazebo_observer import GazeboObserver, resolve_entities, ENTITY_MAP


def test_entity_resolution_all_colors():
    """Verify entity mappings for red, green, and blue."""
    for color in ('red', 'green', 'blue'):
        cube, plat, meta = resolve_entities(color)
        assert cube == f'cube_{color}'
        assert plat == f'placing_platform_{color}'
        assert meta['platform_size'] == (0.06, 0.06, 0.015)
        assert meta['cube_size'] == (0.025, 0.025, 0.025)

    # Test variations
    cube_a, plat_a, _ = resolve_entities('A')
    assert cube_a == 'cube_red'
    assert plat_a == 'placing_platform_red'

    cube_b, plat_b, _ = resolve_entities('B')
    assert cube_b == 'cube_green'

    cube_c, plat_c, _ = resolve_entities('C')
    assert cube_c == 'cube_blue'


def test_pre_release_fails_when_observations_missing():
    """Missing cube or platform observation must reject pre-release."""
    obs = GazeboObserver()
    ok, msg, _ = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'unavailable or stale' in msg

    # Only platform present
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    ok, msg, _ = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'cube_red measurement unavailable' in msg


def test_pre_release_fails_when_stale():
    """Stale observation must reject pre-release."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    obs.update_pose_manual('cube_red', (-0.08, -2.64, 0.1315))

    # Manually make observation old
    obs.poses['cube_red']['wall_time'] -= 10.0
    ok, msg, _ = obs.verify_cube_above_platform('cube_red', 'placing_platform_red', max_age_sec=2.0)
    assert not ok
    assert 'stale' in msg


def test_pre_release_rejects_ground_level_cube():
    """Cube at ground level (dropped early) must reject release."""
    obs = GazeboObserver()
    # Platform top is at 0.1075 + 0.0075 = 0.115m
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Cube center at ground level Z=0.0125 -> bottom at Z=0.00m
    obs.update_pose_manual('cube_red', (-0.08, -2.64, 0.0125))

    ok, msg, metrics = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'below platform top' in msg
    assert metrics['gap'] < 0.0


def test_pre_release_rejects_airborne_cube():
    """Cube hovering too high must reject release to prevent unguided drops."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Cube center at Z=0.20m -> bottom at 0.1875m -> gap ~ 72.5mm (> 20mm)
    obs.update_pose_manual('cube_red', (-0.08, -2.64, 0.20))

    ok, msg, metrics = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'hovering too high' in msg
    assert metrics['gap'] > 0.020


def test_pre_release_rejects_lateral_overhang():
    """Lateral offset > 14.5mm (less than 3mm margin) must reject release."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Offset dx = 16mm (overhanging beyond 14.5mm threshold)
    obs.update_pose_manual('cube_red', (-0.08 + 0.016, -2.64, 0.1315))

    ok, msg, _ = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'lateral containment check failed' in msg


def test_pre_release_accepts_valid_clearance_and_containment():
    """Valid release pose (e.g. 4mm clearance, centered) passes pre-release check."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Platform top = 0.115m. Cube center at 0.1315m -> bottom at 0.119m -> gap = 4mm
    obs.update_pose_manual('cube_red', (-0.08 + 0.003, -2.64 - 0.002, 0.1315))

    ok, msg, metrics = obs.verify_cube_above_platform('cube_red', 'placing_platform_red')
    assert ok
    assert 'Pre-release verified' in msg
    assert pytest.approx(metrics['gap'], abs=0.001) == 0.004


def test_post_release_verifies_resting_cube():
    """Post-release verifies resting height and physical stability."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Settled cube: center at 0.1275m -> bottom at 0.115m (exactly on platform top)
    obs.update_pose_manual('cube_red', (-0.081, -2.641, 0.1275), velocity=(0.001, 0.0, 0.0))

    ok, msg, metrics = obs.verify_cube_resting_on_platform('cube_red', 'placing_platform_red')
    assert ok
    assert 'Post-release verified' in msg
    assert abs(metrics['height_diff']) < 0.002


def test_post_release_rejects_unsettled_moving_cube():
    """Moving cube (speed > 10mm/s) fails post-release check."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    obs.update_pose_manual('cube_red', (-0.08, -2.64, 0.1275), velocity=(0.05, 0.02, 0.0))

    ok, msg, metrics = obs.verify_cube_resting_on_platform('cube_red', 'placing_platform_red')
    assert not ok
    assert 'stability check failed' in msg


def test_post_release_rejects_fallen_cube():
    """Cube that dropped to ground level fails post-release check."""
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red', (-0.08, -2.64, 0.1075))
    # Cube fell onto arena floor (Z=0.0125m)
    obs.update_pose_manual('cube_red', (-0.08, -2.64, 0.0125), velocity=(0.0, 0.0, 0.0))

    ok, msg, _ = obs.verify_cube_resting_on_platform('cube_red', 'placing_platform_red')
    assert not ok


def test_measured_centre_plan_preservation():
    """Verify that _current_place_pose returns the complete accepted plan with chosen pitch."""
    import threading
    from types import SimpleNamespace as NS
    from k12_pick_place.moveit_pick_place_commander import MoveItPickPlaceCommander as Commander
    from k12_pick_place.ik import PlanarArmIK

    ik = PlanarArmIK()
    center = (-0.08, -2.64, 0.1315)
    # Simulate a target requiring alternate pitch (e.g. pi/2 - 0.08)
    chosen_plan = {'pre_grasp_joints': (0.1, 0.2, 0.3, 0.4, 0.5), 'pre_grasp': (0.30, 0.0, -0.04), 'pitch': 1.49}
    node = NS(
        hold_lock=threading.Lock(),
        held_entity_name='red_cube',
        observer=NS(get_pose=lambda *a, **k: {'position':(-.08,-2.64,.1075), 'orientation':(0.,0.,0.,1.)},
                    world_point_in_frame=lambda node, point, frame: (0.30,0.,-.0525)),
        get_parameter=lambda name: NS(value={
            'place_table_frame': 'map', 'target_frame': 'arm_base_link',
            'grasp_z_bias': 0.010, 'place_table_release_height': 0.1315
        }[name]),
        ik=NS(pick_plan=lambda pt, **kw: chosen_plan),
        get_logger=lambda: NS(info=lambda *a: None, error=lambda *a: None, warn=lambda *a: None),
    )

    tcp, plan = Commander._current_place_pose(node, return_plan=True)
    assert tcp == (0.30, 0.0, -.0525)
    assert plan is chosen_plan
    assert plan['pitch'] == 1.49


def test_unknown_identity_rejected():
    with pytest.raises(ValueError):
        resolve_entities('unknown')


def test_zero_stamp_bridge_measures_motion_and_ignores_local_links():
    from types import SimpleNamespace as N
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    now = [1_000_000_000]
    observer = GazeboObserver()
    observer.node = N(get_clock=lambda: N(now=lambda: N(nanoseconds=now[0])))
    t = TransformStamped(child_frame_id='cube_red')
    t.transform.rotation.w = 1.
    observer._on_pose_message(TFMessage(transforms=[t]))
    assert observer.get_pose('cube_red')['speed'] == float('inf')
    now[0] += 100_000_000
    t.transform.translation.x = .01
    observer._on_pose_message(TFMessage(transforms=[t]))
    assert observer.get_pose('cube_red')['speed'] == pytest.approx(.1)
    t.child_frame_id = 'left_gripper'
    observer._on_pose_message(TFMessage(transforms=[t]))
    assert observer.get_pose('left_gripper') is None


def test_rotation_and_world_registration_do_not_use_amcl_as_world():
    import numpy as np
    import math
    from types import SimpleNamespace as N
    from geometry_msgs.msg import TransformStamped
    from k12_pick_place.gazebo_observer import pose_matrix, matrix_quaternion
    q = (0., 0., math.sin(-math.pi/4), math.cos(-math.pi/4))
    matrix = pose_matrix((.1,-2.25,0.), q)
    assert np.allclose(pose_matrix((0.,0.,0.), matrix_quaternion(matrix))[:3,:3], matrix[:3,:3])
    obs = GazeboObserver()
    obs.update_pose_manual('four_wheel', (.1,-2.25,0.), q)
    t = TransformStamped()
    t.transform.translation.x = -.12
    t.transform.translation.z = -.199
    t.transform.rotation.w = 1.
    node = N(tf_buffer=N(lookup_transform=lambda *a, **k: t))
    arm = obs.world_point_in_frame(node, (.1,-2.64,.1415), 'arm_base_link')
    assert arm == pytest.approx((.27,0.,-.0575))


def test_five_mm_is_radial_and_support_requires_a_dwell():
    obs = GazeboObserver()
    obs.update_pose_manual('placing_platform_red',(-.08,-2.64,.1075))
    obs.update_pose_manual('cube_red',(-.076,-2.636,.1275), stamp_sec=1.)
    assert not obs.verify_cube_resting_on_platform('cube_red','placing_platform_red',max_horizontal_error=.005)[0]
    obs.update_pose_manual('cube_red',(-.08,-2.64,.1275),stamp_sec=2.,velocity=(0.,0.,0.))
    assert not obs.verify_cube_resting_on_platform('cube_red','placing_platform_red',stability_duration=1.)[0]
    for i in range(1,13):
        obs.update_pose_manual('cube_red',(-.08,-2.64,.1275),stamp_sec=2.+i*.1,velocity=(0.,0.,0.))
    assert obs.verify_cube_resting_on_platform('cube_red','placing_platform_red',stability_duration=1.)[0]
