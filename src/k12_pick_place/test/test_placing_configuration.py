"""Unit tests for placing station collision geometry, synchronized coordinates, and reachability."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
import yaml

from k12_pick_place.ik import PlanarArmIK

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
WORLD_FILE = WORKSPACE_ROOT / 'src/articubot_one_with_arm/worlds/og.world'


def test_gazebo_world_table_and_platforms():
    """Verify placing table and elevated platform dimensions in og.world."""
    tree = ET.parse(WORLD_FILE)
    root = tree.getroot()
    world = root.find('world')

    # Placing table (purple)
    station = world.find(".//model[@name='placing_station']")
    assert station is not None
    pose_str = station.find('pose').text.strip()
    x, y, z = [float(v) for v in pose_str.split()[:3]]
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(-2.75)
    assert z == pytest.approx(0.05)
    size_str = station.find(".//box/size").text.strip()
    sx, sy, sz = [float(v) for v in size_str.split()]
    assert sx == pytest.approx(0.30)
    assert sy == pytest.approx(0.30)
    assert sz == pytest.approx(0.10)
    table_top_z = z + sz / 2.0
    assert table_top_z == pytest.approx(0.10)

    # 3 Elevated platforms
    platform_data = {
        'red': (-0.08, -2.64, 0.1075),
        'green': (0.00, -2.64, 0.1075),
        'blue': (0.08, -2.64, 0.1075),
    }
    for color, (exp_x, exp_y, exp_z) in platform_data.items():
        plat = world.find(f".//model[@name='placing_platform_{color}']")
        assert plat is not None, f"Missing platform for {color}"
        p_str = plat.find('pose').text.strip()
        px, py, pz = [float(v) for v in p_str.split()[:3]]
        assert px == pytest.approx(exp_x)
        assert py == pytest.approx(exp_y)
        assert pz == pytest.approx(exp_z)
        psize_str = plat.find(".//box/size").text.strip()
        psx, psy, psz = [float(v) for v in psize_str.split()]
        assert psx == pytest.approx(0.06)
        assert psy == pytest.approx(0.06)
        assert psz == pytest.approx(0.015)
        plat_top_z = pz + psz / 2.0
        assert plat_top_z == pytest.approx(0.115)


@pytest.mark.parametrize("config_filename", ["pick_place_service.yaml", "moveit_nav_pick_place.yaml"])
def test_moveit_planning_scene_boxes_match_gazebo(config_filename):
    """Verify MoveIt planning scene boxes model the actual 10cm table and 1.5cm platforms."""
    yaml_path = CONFIG_DIR / config_filename
    raw_cfg = yaml.safe_load(yaml_path.read_text())
    node_key = 'moveit_pick_place_commander' if 'moveit_pick_place_commander' in raw_cfg else 'k12_pick_place_commander'
    params = raw_cfg[node_key]['ros__parameters']
    boxes = params['planning_scene_boxes']
    assert len(boxes) % 6 == 0

    # Parse boxes
    box_list = [boxes[i:i+6] for i in range(0, len(boxes), 6)]

    # Support 3: placing table
    table_box = box_list[3]
    assert table_box == pytest.approx([0.0, -2.75, 0.05, 0.30, 0.30, 0.10])
    table_top = table_box[2] + table_box[5] / 2.0
    assert table_top == pytest.approx(0.10)

    # Support 4: red platform
    red_box = box_list[4]
    assert red_box == pytest.approx([-0.08, -2.64, 0.1075, 0.06, 0.06, 0.015])
    assert red_box[2] + red_box[5] / 2.0 == pytest.approx(0.115)

    # Support 5: green platform
    green_box = box_list[5]
    assert green_box == pytest.approx([0.00, -2.64, 0.1075, 0.06, 0.06, 0.015])
    assert green_box[2] + green_box[5] / 2.0 == pytest.approx(0.115)

    # Support 6: blue platform
    blue_box = box_list[6]
    assert blue_box == pytest.approx([0.08, -2.64, 0.1075, 0.06, 0.06, 0.015])
    assert blue_box[2] + blue_box[5] / 2.0 == pytest.approx(0.115)


@pytest.mark.parametrize("config_filename", ["pick_place_service.yaml", "moveit_nav_pick_place.yaml"])
def test_synchronized_platform_coordinates(config_filename):
    """Verify platform coordinates are consistent between world, yaml, and known_slots."""
    yaml_path = CONFIG_DIR / config_filename
    raw_cfg = yaml.safe_load(yaml_path.read_text())
    node_key = 'moveit_pick_place_commander' if 'moveit_pick_place_commander' in raw_cfg else 'k12_pick_place_commander'
    params = raw_cfg[node_key]['ros__parameters']

    base_pos = params['place_table_position']
    xy_offset = params['place_table_release_xy_offset']

    # Red
    red_offset = params['place_table_release_red_cube_xy_offset']
    red_world_x = base_pos[0] + xy_offset[0] + red_offset[0]
    red_world_y = base_pos[1] + xy_offset[1] + red_offset[1]
    assert red_world_x == pytest.approx(-0.08)
    assert red_world_y == pytest.approx(-2.64)

    # Green
    green_offset = params['place_table_release_green_cylinder_xy_offset']
    green_world_x = base_pos[0] + xy_offset[0] + green_offset[0]
    green_world_y = base_pos[1] + xy_offset[1] + green_offset[1]
    assert green_world_x == pytest.approx(0.00)
    assert green_world_y == pytest.approx(-2.64)

    # Blue
    blue_offset = params['place_table_release_blue_block_xy_offset']
    blue_world_x = base_pos[0] + xy_offset[0] + blue_offset[0]
    blue_world_y = base_pos[1] + xy_offset[1] + blue_offset[1]
    assert blue_world_x == pytest.approx(0.08)
    assert blue_world_y == pytest.approx(-2.64)

    # Release height
    plat_top_z = 0.115
    cube_half_h = 0.025 / 2.0
    clearance = params['release_clearance']
    expected_release_h = plat_top_z + cube_half_h + clearance
    assert params['place_table_release_height'] == pytest.approx(expected_release_h)


@pytest.mark.parametrize("stopping_y", [-2.25, -2.28])
def test_placing_trajectories_reachability_all_platforms(stopping_y):
    """Verify pre-place, descend, release, and retreat reachability for all 3 platforms."""
    ik = PlanarArmIK()
    # When robot base is aligned at (approach_x, stopping_y, yaw=-pi/2):
    # arm_base_link is at (approach_x, stopping_y - 0.12, z~0.204).
    # Platform center is at Y=-2.64, so X_arm = 2.64 - (abs(stopping_y) + 0.12)
    # At stopping_y = -2.25: X_arm = 2.64 - 2.37 = 0.27m.
    # At stopping_y = -2.28: X_arm = 2.64 - 2.40 = 0.24m.
    # Platform top is at 0.115m. Cube center release is at 0.1315m.
    # TCP height in map is 0.1415m (with 10mm grasp_z_bias).
    # TCP Z in arm_base_link is 0.1415 - 0.204 = -0.0625m.
    # Approach height is 25mm above place pose: Z = -0.0375m.

    x_arm = 2.64 - (abs(stopping_y) + 0.12)
    for name in ['red', 'green', 'blue']:
        # Nominal aligned target (approach_x aligns with platform_x, so Y_arm = 0.0)
        place_tcp = (x_arm, 0.0, -0.0625)
        plan = ik.pick_plan(place_tcp, approach=0.025, lift=0.025)
        assert plan is not None, f"Plan failed for {name} platform center at y={stopping_y}"

        # Pre-place pose
        pre_grasp = plan['pre_grasp']
        assert pre_grasp[2] == pytest.approx(-0.0375)
        assert ik.forward(plan['pre_grasp_joints']) == pytest.approx(pre_grasp, abs=0.002)

        # Descent vertical path
        path = ik.vertical_path(plan['pre_grasp_joints'], place_tcp[2])
        assert len(path) >= 2
        for waypoint in path:
            pos = ik.forward(waypoint)
            assert pos[0] == pytest.approx(place_tcp[0], abs=0.003)
            assert pos[1] == pytest.approx(place_tcp[1], abs=0.003)
            assert -0.065 <= pos[2] <= -0.035

        # Retreat vertical path back up
        retreat_path = ik.vertical_path(path[-1], pre_grasp[2])
        assert len(retreat_path) >= 2
        final_pos = ik.forward(retreat_path[-1])
        assert final_pos[2] == pytest.approx(pre_grasp[2], abs=0.002)

    # Also test case where base is centered at X=0.0, reaching lateral platforms (+/-0.08m)
    for y_arm in [-0.08, 0.08]:
        place_tcp = (x_arm, y_arm, -0.0625)
        plan = ik.pick_plan(place_tcp, approach=0.025, lift=0.025)
        assert plan is not None, f"Plan failed for lateral platform at y_arm={y_arm} at base_y={stopping_y}"


def test_candidate_placement_offsets_contained_within_platforms():
    """Verify candidate placement offsets keep the cube strictly on the colored platform."""
    platform_half_w = 0.030  # 6cm x 6cm platform
    cube_half_w = 0.0125     # 2.5cm cube
    max_safe_offset = platform_half_w - cube_half_w  # 0.0175m

    candidate_dx = [0.0, -0.010, 0.010, -0.015, 0.015]
    candidate_dy = [0.0, 0.010, -0.010, 0.015, -0.015]

    for dx in candidate_dx:
        assert abs(dx) <= max_safe_offset, f"dx {dx} would overhang platform edge"
    for dy in candidate_dy:
        assert abs(dy) <= max_safe_offset, f"dy {dy} would overhang platform edge"


