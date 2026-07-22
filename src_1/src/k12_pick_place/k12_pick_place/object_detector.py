import json
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def project_pixel_to_ros_optical(
    u: int,
    v: int,
    depth: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Optional[Tuple[float, float, float]]:
    if fx <= 0.0 or fy <= 0.0 or depth <= 0.0:
        return None
    return (
        (float(u) - cx) * depth / fx,
        (float(v) - cy) * depth / fy,
        depth,
    )


def detection_confidence(
    area: float,
    depth: float,
    confidence_area_px: float = 500.0,
    max_depth_m: float = 4.5,
) -> float:
    area_norm = max(1.0, float(confidence_area_px))
    area_score = min(1.0, max(0.0, float(area) / area_norm))
    depth_limit = max(0.1, float(max_depth_m))
    depth_score = max(0.25, 1.0 - 0.35 * min(1.0, float(depth) / depth_limit))
    return round(max(0.0, min(1.0, area_score * depth_score)), 3)


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('k12_object_detector')

        self.declare_parameter('rgb_topic', '/camera/depth_camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth_camera/camera_info')
        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pose_array_topic', '/amms/detected_object_poses_camera')
        self.declare_parameter('debug_image_topic', '/amms/detection_debug')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('min_area_px', 60.0)
        self.declare_parameter('confidence_area_px', 500.0)
        self.declare_parameter('max_depth_m', 4.5)
        self.declare_parameter('projection_mode', 'ros_optical')
        self.declare_parameter('fallback_width', 640)
        self.declare_parameter('fallback_height', 480)
        self.declare_parameter('fallback_horizontal_fov', 1.047)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.05)

        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.rgb_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.rgb_frame = 'cam_1'
        self.depth_frame = 'cam_1'
        self.camera_info: Optional[CameraInfo] = None
        self.last_stamp = None
        self._projection_warning_sent = False

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.get_parameter('rgb_topic').value,
            qos_profile=qos,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            self.get_parameter('depth_topic').value,
            qos_profile=qos,
        )
        self.info_sub = Subscriber(
            self,
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            qos_profile=qos,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop_sec').value),
        )
        self.sync.registerCallback(self._on_sync)

        self.detection_pub = self.create_publisher(String, self.get_parameter('detections_topic').value, 10)
        self.pose_pub = self.create_publisher(PoseArray, self.get_parameter('pose_array_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 10)

        self.color_ranges = {
            'red': [
                (np.array([0, 90, 70]), np.array([10, 255, 255])),
                (np.array([170, 90, 70]), np.array([180, 255, 255])),
            ],
            'green': [(np.array([40, 70, 60]), np.array([85, 255, 255]))],
            'blue': [(np.array([95, 70, 60]), np.array([130, 255, 255]))],
        }

    def _on_sync(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Could not convert synchronized RGB-D images: {exc}')
            return

        self.camera_info = info_msg
        self.rgb_frame = info_msg.header.frame_id or rgb_msg.header.frame_id or self.rgb_frame
        self.depth_frame = depth_msg.header.frame_id or self.rgb_frame
        self.last_stamp = rgb_msg.header.stamp
        self._detect()

    def _detect(self) -> None:
        if self.rgb_image is None or self.depth_image is None:
            return

        rgb = self.rgb_image.copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        detections = []

        for color, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < float(self.get_parameter('min_area_px').value):
                    continue

                moments = cv2.moments(contour)
                if abs(moments['m00']) < 1e-6:
                    continue

                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])
                depth = self._median_depth(mask, u, v)
                if depth is None:
                    continue

                point = self._project_pixel_to_camera(u, v, depth)
                if point is None:
                    continue

                shape = self._classify_shape(contour, color)
                x, y, w, h = cv2.boundingRect(contour)
                confidence = self._detection_confidence(area, depth)
                detections.append({
                    'id': f'{color}_{shape}',
                    'color': color,
                    'shape': shape,
                    'confidence': confidence,
                    'u': u,
                    'v': v,
                    'depth_m': depth,
                    'area_px': area,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'position': {
                        'x': point[0],
                        'y': point[1],
                        'z': point[2],
                    },
                })

                cv2.rectangle(rgb, (x, y), (x + w, y + h), self._bgr(color), 2)
                cv2.circle(rgb, (u, v), 4, (255, 255, 255), -1)
                cv2.putText(
                    rgb,
                    f'{color} {shape} {confidence:.2f} {depth:.2f}m',
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    self._bgr(color),
                    1,
                    cv2.LINE_AA,
                )

        detections.sort(key=lambda d: d['depth_m'])
        self._publish(detections, rgb)

    def _detection_confidence(self, area: float, depth: float) -> float:
        return detection_confidence(
            area,
            depth,
            float(self.get_parameter('confidence_area_px').value),
            float(self.get_parameter('max_depth_m').value),
        )

    def _median_depth(self, mask: np.ndarray, u: int, v: int) -> Optional[float]:
        radius = 5
        y0 = max(0, v - radius)
        y1 = min(self.depth_image.shape[0], v + radius + 1)
        x0 = max(0, u - radius)
        x1 = min(self.depth_image.shape[1], u + radius + 1)
        patch = self.depth_image[y0:y1, x0:x1]
        local_mask = mask[y0:y1, x0:x1] > 0
        values = patch[local_mask]
        if values.size == 0:
            values = patch.reshape(-1)

        values = values.astype(np.float32)
        if self.depth_image.dtype == np.uint16:
            values = values / 1000.0

        values = values[np.isfinite(values)]
        values = values[(values > 0.05) & (values < float(self.get_parameter('max_depth_m').value))]
        if values.size == 0:
            return None
        return float(np.median(values))

    def _intrinsics(self) -> Tuple[float, float, float, float]:
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )

        width = float(self.get_parameter('fallback_width').value)
        height = float(self.get_parameter('fallback_height').value)
        fov = float(self.get_parameter('fallback_horizontal_fov').value)
        fx = width / (2.0 * math.tan(fov / 2.0))
        fy = fx
        return fx, fy, width / 2.0, height / 2.0

    def _project_pixel_to_camera(self, u: int, v: int, depth: float) -> Optional[Tuple[float, float, float]]:
        fx, fy, cx, cy = self._intrinsics()
        if fx <= 0.0 or fy <= 0.0:
            return None

        mode = self.get_parameter('projection_mode').value
        if mode != 'ros_optical' and not self._projection_warning_sent:
            self.get_logger().warn(
                f'Ignoring unsupported projection_mode={mode}; detector now publishes ROS optical points.'
            )
            self._projection_warning_sent = True
        return project_pixel_to_ros_optical(u, v, depth, fx, fy, cx, cy)

    def _classify_shape(self, contour: np.ndarray, color: str) -> str:
        if color == 'red':
            return 'cube'
        if color == 'green':
            return 'cylinder'
        if color == 'blue':
            return 'rectangular_block'

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        area = max(float(cv2.contourArea(contour)), 1.0)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        x, y, w, h = cv2.boundingRect(contour)
        aspect = w / max(float(h), 1.0)
        if circularity > 0.72:
            return 'cylinder'
        if len(approx) == 4 and 0.8 <= aspect <= 1.25:
            return 'cube'
        return 'rectangular_block'

    def _publish(self, detections: List[Dict], debug_image: np.ndarray) -> None:
        frame_id = self.rgb_frame or self.depth_frame or 'cam_1'
        stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()

        payload = {
            'header': {
                'frame_id': frame_id,
                'stamp': {
                    'sec': int(stamp.sec),
                    'nanosec': int(stamp.nanosec),
                },
            },
            'objects': detections,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.detection_pub.publish(msg)

        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = stamp
        for detection in detections:
            pose = Pose()
            pose.position.x = detection['position']['x']
            pose.position.y = detection['position']['y']
            pose.position.z = detection['position']['z']
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pose_pub.publish(pose_array)

        if bool(self.get_parameter('publish_debug_image').value):
            image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            image_msg.header.frame_id = frame_id
            image_msg.header.stamp = stamp
            self.debug_pub.publish(image_msg)

    @staticmethod
    def _bgr(color: str) -> Tuple[int, int, int]:
        return {
            'red': (0, 0, 255),
            'green': (0, 220, 0),
            'blue': (255, 80, 0),
        }.get(color, (255, 255, 255))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
