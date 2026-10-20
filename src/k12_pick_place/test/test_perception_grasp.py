"""RGB-D and physical-grasp regressions, requiring ROS imports but no running graph."""
from types import MethodType, SimpleNamespace
import time

import cv2
import numpy as np
import pytest

from k12_pick_place.object_detector import ObjectDetector, project_pixel_to_ros_optical
from k12_pick_place.moveit_pick_place_commander import MoveItPickPlaceCommander


def bind(cls, names, **values):
    node = SimpleNamespace(**values)
    for name in names:
        setattr(node, name, MethodType(getattr(cls, name), node))
    return node


def parameters(values):
    return lambda name: SimpleNamespace(value=values[name])


def test_projection_rejects_nonfinite_depth_and_preserves_subpixel_centroid():
    assert project_pixel_to_ros_optical(320.5, 240.25, .4, 400, 400, 320, 240) == pytest.approx((.0005, .00025, .4))
    assert project_pixel_to_ros_optical(320, 240, float('nan'), 400, 400, 320, 240) is None
    assert project_pixel_to_ros_optical(320, 240, float('inf'), 400, 400, 320, 240) is None


def test_all_three_cube_colors_are_detected_with_valid_registered_depth():
    rgb = np.zeros((80, 160, 3), dtype=np.uint8)
    for x, bgr in ((10, (0, 0, 255)), (60, (0, 255, 0)), (110, (255, 0, 0))):
        cv2.rectangle(rgb, (x, 20), (x+21, 41), bgr, -1)
    found = []
    node = bind(ObjectDetector, ['_detect', '_median_depth', '_classify_shape', '_detection_confidence'],
        rgb_image=rgb, depth_image=np.full((80, 160), .4, dtype=np.float32),
        get_parameter=parameters({'min_area_px': 15, 'max_depth_m': 4.5,
            'known_object_shape': 'cube', 'confidence_area_px': 500}),
        color_ranges={'red': [(np.array([0, 60, 30]), np.array([12, 255, 255]))],
                      'green': [(np.array([35, 60, 30]), np.array([85, 255, 255]))],
                      'blue': [(np.array([86, 60, 30]), np.array([135, 255, 255]))]},
        _project_pixel_to_camera=lambda u, v, depth: project_pixel_to_ros_optical(u, v, depth, 400, 400, 80, 40),
        _bgr=ObjectDetector._bgr, _publish=lambda detections, image: found.extend(detections))
    node._detect()
    assert {obj['id'] for obj in found} == {'red_cube', 'green_cube', 'blue_cube'}
    assert all(obj['depth_m'] == pytest.approx(.4) for obj in found)
    assert all(obj['u'] % 1 == .5 for obj in found)


def test_depth_does_not_fall_back_to_background_when_object_depth_is_missing():
    mask = np.zeros((20, 20), dtype=np.uint8)
    depth = np.ones((20, 20), dtype=np.float32)
    mask[8:13, 8:13] = 255
    depth[8:13, 8:13] = np.nan
    node = bind(ObjectDetector, ['_median_depth'], depth_image=depth,
                get_parameter=parameters({'max_depth_m': 4.5}))
    assert node._median_depth(mask, 10, 10) is None
    assert node._median_depth(np.zeros_like(mask), 10, 10) is None


def test_stability_requires_distinct_frames_and_restarts_after_large_motion():
    node = bind(MoveItPickPlaceCommander, ['_accept_stable_pick_target', '_reset_stable_pick_target'],
        get_parameter=parameters({'stable_detection_count': 3, 'require_stable_detection': True,
            'stable_detection_position_tolerance_m': .008, 'stable_detection_timeout_sec': 10}),
        _target_key=MoveItPickPlaceCommander._target_key,
        _target_position=MoveItPickPlaceCommander._target_position)
    node._reset_stable_pick_target()
    target = {'id': 'red_cube', 'arm_base_position': (.325, 0, -.0765), '_observation_stamp': (1, 0)}
    assert not node._accept_stable_pick_target(target)
    for _ in range(10):
        assert not node._accept_stable_pick_target(target)
    assert node.stable_target_count == 1
    assert not node._accept_stable_pick_target(dict(target, _observation_stamp=(2, 0)))
    assert node._accept_stable_pick_target(dict(target, _observation_stamp=(3, 0)))
    assert not node._accept_stable_pick_target(dict(target, _observation_stamp=(4, 0), arm_base_position=(.35, 0, -.0765)))


def test_observations_use_camera_acquisition_transform():
    requests = []
    node = bind(MoveItPickPlaceCommander, ['_transform_objects'],
        tf_buffer=SimpleNamespace(transform=lambda point, *args, **kwargs: (requests.append(point) or point)))
    result = node._transform_objects({'header': {'frame_id': 'camera_optical',
        'stamp': {'sec': 41, 'nanosec': 250000000}}, 'objects': [
            {'id': 'red_cube', 'position': {'x': .1, 'y': .2, 'z': .4}}]}, 'map')
    assert result[0][1] == pytest.approx((.1, .2, .4))
    assert requests[0].header.stamp.sec == 41
    assert requests[0].header.stamp.nanosec == 250000000


def test_latched_pick_selection_requires_distinct_matching_camera_frames():
    grasp = (.325, 0., -.0765)
    payload = {'header': {'stamp': {'sec': 1, 'nanosec': 0}}}
    obj = {'id': 'red_cube', 'confidence': 1.}
    params = {'prefer_external_pick_target': True, 'external_pick_target_timeout_sec': 3.,
              'object_timeout_sec': 2., 'external_target_match_tolerance_m': .05,
              'target_frame': 'arm_base_link', 'stable_detection_count': 3,
              'require_stable_detection': True, 'stable_detection_position_tolerance_m': .008,
              'stable_detection_timeout_sec': 10.}
    node = bind(MoveItPickPlaceCommander, ['_select_external_pick_target', '_observation_stamp',
                '_accept_stable_pick_target', '_reset_stable_pick_target'],
        get_parameter=parameters(params), latest_payload=payload, latest_time=time.monotonic(),
        latest_external_pick_target={'object_id': 'red_cube', 'object': obj,
                                     'arm_base_position': grasp},
        latest_external_pick_target_time=time.monotonic(),
        _position_from_payload=MoveItPickPlaceCommander._position_from_payload,
        _target_key=MoveItPickPlaceCommander._target_key,
        _target_position=MoveItPickPlaceCommander._target_position,
        _detection_has_confidence=lambda obj: True,
        _transform_objects=lambda *args: [(obj, grasp)],
        _grasp_pose_from_object=lambda position, obj: position,
        _target_from_object=lambda obj, grasp, position: dict(obj, arm_base_position=grasp),
        get_logger=lambda: SimpleNamespace(info=lambda *args: None))
    node._reset_stable_pick_target()
    for _ in range(4):
        assert not node._accept_stable_pick_target(node._select_external_pick_target())
    assert node.stable_target_count == 1
    payload['header']['stamp']['sec'] = 2
    assert not node._accept_stable_pick_target(node._select_external_pick_target())
    payload['header']['stamp']['sec'] = 3
    assert node._accept_stable_pick_target(node._select_external_pick_target())
    node._transform_objects = lambda *args: [(obj, (.5, 0., -.0765))]
    assert node._select_external_pick_target() is None


@pytest.mark.parametrize('fingers,valid', [((.009, .009), True), ((.0105, .0075), True),
    ((0, 0), False), ((.0115, .0115), False), ((.009, 0), False),
    ((.0114, .0066), False), ((float('nan'), .009), False)])
def test_physical_contact_rejects_empty_close_one_finger_and_excessive_offset(fingers, valid):
    node = bind(MoveItPickPlaceCommander, ['_verify_finger_contact'],
        get_parameter=parameters({'grasp_object_width_m': .025, 'grasp_width_tolerance_m': .004,
                                  'grasp_centering_tolerance_m': .002}),
        _finger_feedback=lambda: fingers)
    if valid:
        node._verify_finger_contact()
    else:
        with pytest.raises(RuntimeError, match='No credible two-finger contact'):
            node._verify_finger_contact()


def test_physical_contact_accepts_asymmetric_contact():
    node = bind(MoveItPickPlaceCommander, ['_verify_finger_contact'],
        get_parameter=parameters({'grasp_object_width_m': .025, 'grasp_width_tolerance_m': .008,
                                  'grasp_centering_tolerance_m': .006}),
        _finger_feedback=lambda: (0.0064, 0.0115))
    node._verify_finger_contact()


def test_opening_waits_until_fingers_reach_tight_tolerance(monkeypatch):
    monkeypatch.setattr('rclpy.ok', lambda: True)
    joint_positions = {'left_gripper_joint': .010, 'right_gripper_joint': .010}
    waits = []
    def advance(_seconds):
        waits.append(True)
        joint_positions.update(left_gripper_joint=.0115, right_gripper_joint=.0115)
    node = bind(MoveItPickPlaceCommander, ['_command_gripper_until_reached'],
        get_parameter=parameters({'gripper_joint_tolerance_m': .0005}),
        latest_joint_positions=joint_positions, _sleep=advance,
        _publish_gripper=lambda *args, **kwargs: None)
    node._command_gripper_until_reached(.0115, .0115, 'test opening')
    assert waits, 'A 1.5 mm shortfall must not count as fully open'
