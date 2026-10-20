"""Exercise camera callbacks without a simulator or any motion controllers."""
import json
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo

from k12_pick_place.object_detector import ObjectDetector


@pytest.fixture
def detector():
    rclpy.init()
    node = ObjectDetector()
    node.debug_pub = Mock()
    node.detection_pub = Mock()
    node.pose_pub = Mock()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def rgb_message(node, hue=0, encoding='rgb8'):
    hsv = np.zeros((100, 120, 3), dtype=np.uint8)
    hsv[40:65, 45:70] = (hue, 255, 220)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if encoding == 'rgb8' else bgr
    msg = node.bridge.cv2_to_imgmsg(image, encoding=encoding)
    msg.header.frame_id = 'camera_optical'
    msg.header.stamp.sec = 1
    return msg


@pytest.mark.parametrize('hue', [0, 175])
@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_red_overlay_without_depth_or_calibration(detector, hue, encoding):
    msg = rgb_message(detector, hue, encoding)
    with patch('k12_pick_place.object_detector.cv2.putText', wraps=cv2.putText) as text:
        detector.rgb_sub.signalMessage(msg)
    assert 'RED' in [call.args[1] for call in text.call_args_list]
    output = detector.debug_pub.publish.call_args.args[0]
    image = detector.bridge.imgmsg_to_cv2(output, 'bgr8')
    assert tuple(image[40, 45]) == (0, 0, 255)
    assert output.header == msg.header
    detector.detection_pub.publish.assert_not_called()
    detector.pose_pub.publish.assert_not_called()


@pytest.mark.parametrize('dtype,value,expected', [
    (np.float32, 1.0, 1), (np.uint16, 1000, 1),
    (np.float32, np.nan, 0), (np.float32, 0.0, 0),
    (np.float32, 5.0, 0),
])
def test_synchronized_depth_and_cached_calibration(detector, dtype, value, expected):
    info = CameraInfo()
    info.k = [100., 0., 60., 0., 100., 50., 0., 0., 1.]
    detector._on_info(info)  # Stamp zero; deliberately not synchronized to images.
    rgb = rgb_message(detector)
    depth = detector.bridge.cv2_to_imgmsg(np.full((100, 120), value, dtype=dtype))
    depth.header = rgb.header
    detector.rgb_sub.signalMessage(rgb)
    detector.depth_sub.signalMessage(depth)
    objects = json.loads(detector.detection_pub.publish.call_args.args[0].data)['objects']
    assert len(objects) == expected
    if expected:
        assert objects[0]['id'] == 'red_cube'
        assert objects[0]['depth_m'] == pytest.approx(1.)
    detector.debug_pub.publish.assert_called_once()


def test_misaligned_depth_rejects_positions_but_preserves_overlay(detector):
    rgb = rgb_message(detector)
    depth = detector.bridge.cv2_to_imgmsg(np.ones((50, 60), dtype=np.float32))
    depth.header = rgb.header
    detector.rgb_sub.signalMessage(rgb)
    detector.depth_sub.signalMessage(depth)
    assert json.loads(detector.detection_pub.publish.call_args.args[0].data)['objects'] == []
    detector.debug_pub.publish.assert_called_once()


def test_missing_calibration_does_not_publish_guessed_grasp_positions(detector):
    rgb = rgb_message(detector)
    depth = detector.bridge.cv2_to_imgmsg(np.ones((100, 120), dtype=np.float32))
    depth.header = rgb.header
    detector.rgb_sub.signalMessage(rgb)
    detector.depth_sub.signalMessage(depth)
    assert json.loads(detector.detection_pub.publish.call_args.args[0].data)['objects'] == []
    detector.debug_pub.publish.assert_called_once()


@pytest.mark.parametrize('hue,color', [(60, 'green'), (110, 'blue')])
def test_configured_cube_shape_is_honored_for_all_colors(detector, hue, color):
    result = detector.set_parameters([Parameter('known_object_shape', value='cube')])
    assert result[0].successful
    info = CameraInfo()
    info.k = [100., 0., 60., 0., 100., 50., 0., 0., 1.]
    detector._on_info(info)
    rgb = rgb_message(detector, hue=hue)
    depth = detector.bridge.cv2_to_imgmsg(np.ones((100, 120), dtype=np.float32))
    depth.header = rgb.header
    detector.rgb_sub.signalMessage(rgb)
    detector.depth_sub.signalMessage(depth)
    objects = json.loads(detector.detection_pub.publish.call_args.args[0].data)['objects']
    assert [obj['id'] for obj in objects] == [f'{color}_cube']


def test_calibration_for_different_image_size_is_rejected(detector):
    info = CameraInfo()
    info.k = [100., 0., 60., 0., 100., 50., 0., 0., 1.]
    info.height, info.width = 480, 640
    detector._on_info(info)
    detector._read_rgb(rgb_message(detector))
    assert detector._project_pixel_to_camera(60, 50, 1.) is None


def test_purple_placing_station_detection_and_classification(detector):
    info = CameraInfo()
    info.k = [100., 0., 60., 0., 100., 50., 0., 0., 1.]
    detector._on_info(info)
    rgb = rgb_message(detector, hue=138)
    depth = detector.bridge.cv2_to_imgmsg(np.full((100, 120), 1.0, dtype=np.float32))
    depth.header = rgb.header
    detector.rgb_sub.signalMessage(rgb)
    detector.depth_sub.signalMessage(depth)
    objects = json.loads(detector.detection_pub.publish.call_args.args[0].data)['objects']
    assert len(objects) == 1
    assert objects[0]['id'] == 'purple_station'
    assert objects[0]['color'] == 'purple'
    assert objects[0]['shape'] == 'station'
