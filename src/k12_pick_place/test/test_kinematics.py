"""Regression tests against independent URDF frame composition."""
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from k12_pick_place.ik import PlanarArmIK

ROOT = Path(__file__).resolve().parents[2]
IK = PlanarArmIK()


def transform(axis=None, angle=0.0, xyz=(0, 0, 0)):
    result = np.eye(4)
    result[:3, 3] = xyz
    if axis is not None:
        c, s = math.cos(angle), math.sin(angle)
        if axis == 'z':
            result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        elif axis == 'y':
            result[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        else:
            result[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return result


def urdf_fk(q):
    y, s, e, w, r = q
    return (transform('z', y, (0, 0, .080))
            @ transform('y', s, (.0098, 0, .030))
            @ transform('y', e, (0, 0, .200))
            @ transform('y', w, (.140, 0, 0))
            @ transform('x', r, (.030, 0, 0))
            @ transform(xyz=(.050, 0, 0))
            @ transform(xyz=(.055, 0, 0)))


def test_fk_matches_independent_urdf_chain():
    rng = random.Random(42)
    for _ in range(500):
        q = tuple(rng.uniform(lo, hi) for lo, hi in IK.limits.ordered())
        assert np.allclose(IK.forward(q), urdf_fk(q)[:3, 3], atol=1e-12)


def test_zero_pose_and_tool_reference():
    assert IK.forward((0, 0, 0, 0, 0)) == pytest.approx((.2848, 0, .310))
    gripper = ET.parse(ROOT / 'articubot_one_with_arm/description/gripper.xacro')
    assert gripper.find(".//joint[@name='gripper_tcp_joint']") is not None
    srdf = ET.parse(ROOT / 'k12_moveit_config/config/k12.srdf')
    assert srdf.find(".//group[@name='arm']/chain").get('tip_link') == 'gripper_tcp'


@pytest.mark.parametrize('roll', [-3.10, -.7, .0, .8, 3.10])
def test_vertical_descent_preserves_entire_orientation_and_nonzero_roll(roll):
    q = IK.solve((.30, .02, -.0265), pitch=math.pi/2, roll=roll)
    path = IK.vertical_path(q, -.0765)
    expected_rotation = urdf_fk(q)[:3, :3]
    for first, last in zip(path, path[1:]):
        assert max(abs(a-b) for a, b in zip(first, last)) < .10
        for t in np.linspace(0, 1, 21):
            # Includes the same joint interpolation curve as a common quintic time law.
            state = tuple(a+(b-a)*t for a, b in zip(first, last))
            fk = urdf_fk(state)
            assert np.allclose(fk[:3, :3], expected_rotation, atol=1e-12)
            assert np.linalg.norm(fk[:2, 3] - [.30, .02]) < .0002
            assert state[4] == pytest.approx(roll)
    assert IK.forward(path[-1])[2] == pytest.approx(-.0765)


def test_unreachable_path_is_rejected_instead_of_clamped_or_partial():
    q = IK.solve((.30, 0, -.02), pitch=math.pi/2)
    with pytest.raises(ValueError, match='unreachable|discontinuous'):
        IK.vertical_path(q, -.4)
    assert IK.pick_plan((.36, .121, -.072)) is None
    with pytest.raises(ValueError, match='Unreachable'):
        IK.solve_clamped((.9, 0, 0))
    assert IK.solve((float('nan'), 0, 0)) is None


@pytest.mark.parametrize('point,approach,lift', [
    ((.30, 0, -.0765), .05, .12),
    ((.23, 0, .1265), .025, .025),
    ((.23, .07, .1265), .025, .025),
    ((.23, -.07, .1265), .025, .025),
])
def test_full_pick_and_place_paths_and_limits(point, approach, lift):
    plan = IK.pick_plan(point, approach=approach, lift=lift)
    assert plan is not None
    descent = IK.vertical_path(plan['pre_grasp_joints'], point[2])
    ascent = IK.vertical_path(descent[-1], plan['lift'][2])
    for q in descent + ascent:
        assert IK.valid_joints(q, margin=.01)
    assert IK.forward(descent[-1]) == pytest.approx(point)
    # Top-down finger tips end 20 mm below TCP. At pick the TCP is 10 mm
    # above the 25 mm cube center: 2.5 mm clearance above its support.
    if point[2] < 0:
        table_top = point[2] - .010 - .0125
        assert point[2] - .020 - table_top == pytest.approx(.0025)


def test_seed_avoids_branch_switches():
    rng = random.Random(13)
    checked = 0
    for _ in range(500):
        q = tuple(rng.uniform(lo+.05, hi-.05) for lo, hi in IK.limits.ordered())
        p = IK.forward(q)
        if p[0] < .05:
            continue
        solved = IK.solve(p, pitch=sum(q[1:4]), roll=q[4], seed=q)
        assert solved is not None
        assert urdf_fk(solved) == pytest.approx(urdf_fk(q), abs=1e-9)
        assert solved == pytest.approx(q, abs=1e-9)
        checked += 1
    assert checked > 100
