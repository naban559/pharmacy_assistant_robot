import json
import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def project_pixel_to_ros_optical(
    u: float,
    v: float,
    depth: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Optional[Tuple[float, float, float]]:
    if fx <= 0.0 or fy <= 0.0 or depth <= 0.0 or not math.isfinite(depth):
        return None
    return (
        (float(u) - cx) * depth / fx,
        (float(v) - cy) * depth / fy,
        depth,
    )


def known_cube_center(points_camera, camera_to_cube_rotation, side):
    """Estimate a known cube center from its visible surfaces, without resizing it.

    Coordinates are rotated into the configured cube axes. Fully observed axes
    use both bounds; a single visible face is extended half a side away from
    the camera. Return None for insufficient/inconsistent surface samples.
    """
    points = np.asarray(points_camera, dtype=float)
    rotation = np.asarray(camera_to_cube_rotation, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 24 or side <= 0:
        return None
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 24:
        return None
    aligned = points @ rotation.T
    low, high = np.quantile(aligned, [.01, .99], axis=0)
    span = high-low
    if np.max(span) > 1.35*side or np.max(span) < .60*side:
        return None
    center = (low+high)/2
    direction = np.median(aligned, axis=0)
    for axis in range(3):
        if span[axis] < .5*side:
            center[axis] = (low[axis]+side/2 if direction[axis] >= 0
                            else high[axis]-side/2)
    return tuple(float(v) for v in rotation.T @ center)


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

        self.declare_parameter('rgb_topic', '/camera/depth_camera/image')
        self.declare_parameter('depth_topic', '/camera/depth_camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/depth_camera/camera_info')
        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pose_array_topic', '/amms/detected_object_poses_camera')
        self.declare_parameter('debug_image_topic', '/amms/detection_debug')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('min_area_px', 10.0)
        self.declare_parameter('confidence_area_px', 100.0)
        self.declare_parameter('max_depth_m', 4.5)
        self.declare_parameter('projection_mode', 'ros_optical')
        self.declare_parameter('min_saturation', 60)
        self.declare_parameter('min_value', 30)
        self.declare_parameter('known_object_shape', '')
        self.declare_parameter('allow_fallback_intrinsics', False)
        self.declare_parameter('known_cube_size_m', 0.0)
        self.declare_parameter('cube_alignment_frame', 'map')
        self.declare_parameter('known_cube_yaw', 0.0)
        self.declare_parameter('fallback_width', 640)
        self.declare_parameter('fallback_height', 480)
        self.declare_parameter('fallback_horizontal_fov', 1.047)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.50)

        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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
        # Calibration is cached: it need not be published at the image rate.
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, qos,
        )
        self.rgb_sub.registerCallback(self._on_rgb)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop_sec').value),
        )
        self.sync.registerCallback(self._on_sync)

        self.detection_pub = self.create_publisher(String, self.get_parameter('detections_topic').value, 10)
        self.pose_pub = self.create_publisher(PoseArray, self.get_parameter('pose_array_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 10)

        saturation = int(self.get_parameter('min_saturation').value)
        value = int(self.get_parameter('min_value').value)
        self.color_ranges = {
            'red': [
                (np.array([0, saturation, value]), np.array([15, 255, 255])),
                (np.array([160, saturation, value]), np.array([180, 255, 255])),
            ],
            'green': [(np.array([35, saturation, value]), np.array([85, 255, 255]))],
            'blue': [(np.array([86, saturation, value]), np.array([128, 255, 255]))],
            'purple': [(np.array([130, saturation, value]), np.array([158, 255, 255]))],
        }

    def _on_info(self, info_msg: CameraInfo) -> None:
        self.camera_info = info_msg

    def _read_rgb(self, rgb_msg: Image) -> None:
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        self.rgb_frame = rgb_msg.header.frame_id or 'cam_1'
        self.last_stamp = rgb_msg.header.stamp

    def _on_rgb(self, rgb_msg: Image) -> None:
        try:
            self._read_rgb(rgb_msg)
        except Exception as exc:
            self.get_logger().warn(f'Could not convert RGB image: {exc}')
            return
        # Show color recognition even when depth is absent or invalid.
        self._detect(localize=False)

    def _on_sync(self, rgb_msg: Image, depth_msg: Image) -> None:
        try:
            self._read_rgb(rgb_msg)
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Could not convert synchronized RGB-D images: {exc}')
            return
        self.depth_frame = depth_msg.header.frame_id or self.rgb_frame
        if self.depth_image.shape[:2] != self.rgb_image.shape[:2]:
            self.get_logger().warn('RGB/depth sizes differ; aligned depth is required.', throttle_duration_sec=5.0)
            self._publish([], self.rgb_image, publish_debug=False)
            return
        self._detect()

    def _detect(self, localize: bool = True) -> None:
        if self.rgb_image is None or (localize and self.depth_image is None):
            return

        rgb = self.rgb_image.copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        detections = []

        for color, ranges in self.color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            kernel = np.ones((3, 3), np.uint8)
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

                u_f = float(moments['m10'] / moments['m00'])
                v_f = float(moments['m01'] / moments['m00'])
                u = int(round(u_f))
                v = int(round(v_f))
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(rgb, (x, y), (x + w, y + h), self._bgr(color), 2)
                cv2.circle(rgb, (u, v), 3, (255, 255, 255), -1)
                cv2.putText(rgb, color.upper(), (x, max(20, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(rgb, color.upper(), (x, max(20, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, self._bgr(color), 2, cv2.LINE_AA)
                if not localize:
                    continue
                depth = self._median_depth(mask, u, v)
                if depth is None:
                    continue

                point = self._project_pixel_to_camera(u_f, v_f, depth)
                if point is None:
                    continue

                center = self._cube_center(contour, depth) if hasattr(self, '_cube_center') else None
                center_valid = center is not None
                if center_valid:
                    point = center
                shape = self._classify_shape(contour, color)
                x, y, w, h = cv2.boundingRect(contour)
                confidence = self._detection_confidence(area, depth)
                try:
                    cube_size_param = float(self.get_parameter('known_cube_size_m').value)
                except Exception:
                    cube_size_param = 0.025
                detections.append({
                    'id': f'{color}_{shape}',
                    'color': color,
                    'shape': shape,
                    'confidence': confidence,
                    'u': u_f,
                    'v': v_f,
                    'depth_m': depth,
                    'area_px': area,
                    'position_is_center': center_valid,
                    'center_estimate_valid': (center_valid if cube_size_param > 0 else None),
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'position': {
                        'x': point[0],
                        'y': point[1],
                        'z': point[2],
                    },
                })

        detections.sort(key=lambda d: d['depth_m'])
        try:
            self._publish(detections, rgb, publish_localization=localize, publish_debug=not localize)
        except TypeError:
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
            return None

        values = values.astype(np.float32)
        if self.depth_image.dtype == np.uint16:
            values = values / 1000.0

        values = values[np.isfinite(values)]
        values = values[(values > 0.05) & (values < float(self.get_parameter('max_depth_m').value))]
        if values.size == 0:
            return None
        return float(np.median(values))

    def _intrinsics(self) -> Tuple[float, float, float, float]:
        if (self.camera_info is not None
                and self.camera_info.k[0] > 0.0 and self.camera_info.k[4] > 0.0
                and all(math.isfinite(value) for value in self.camera_info.k)):
            if (self.rgb_image is not None and self.camera_info.width and self.camera_info.height
                    and (self.camera_info.height, self.camera_info.width) != self.rgb_image.shape[:2]):
                return 0.0, 0.0, 0.0, 0.0
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )

        if not bool(self.get_parameter('allow_fallback_intrinsics').value):
            return 0.0, 0.0, 0.0, 0.0

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

    def _cube_center(self, contour, median_depth):
        side = float(self.get_parameter('known_cube_size_m').value)
        if side <= 0:
            return None
        x,y,w,h = cv2.boundingRect(contour)
        rows, cols = self.depth_image.shape[:2]
        if x <= 0 or y <= 0 or x+w >= cols or y+h >= rows:
            return None  # clipped silhouettes cannot determine the full center
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('cube_alignment_frame').value, self.rgb_frame,
                rclpy.time.Time.from_msg(self.last_stamp))
        except Exception:
            return None
        q = transform.transform.rotation
        xq,yq,zq,wq = q.x,q.y,q.z,q.w
        rotation = np.array([
            [1-2*(yq*yq+zq*zq),2*(xq*yq-zq*wq),2*(xq*zq+yq*wq)],
            [2*(xq*yq+zq*wq),1-2*(xq*xq+zq*zq),2*(yq*zq-xq*wq)],
            [2*(xq*zq-yq*wq),2*(yq*zq+xq*wq),1-2*(xq*xq+yq*yq)]])
        yaw = float(self.get_parameter('known_cube_yaw').value)
        c,t = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c,t,0],[-t,c,0],[0,0,1]]) @ rotation
        mask = np.zeros((rows,cols),dtype=np.uint8)
        cv2.drawContours(mask,[contour],-1,255,-1)
        mask = cv2.erode(mask,np.ones((3,3),dtype=np.uint8))
        v,u = np.nonzero(mask)
        depth = self.depth_image[v,u].astype(float)
        if self.depth_image.dtype == np.uint16:
            depth *= .001
        valid = np.isfinite(depth) & (depth > .05) & (np.abs(depth-median_depth) < 2*side)
        u,v,depth = u[valid],v[valid],depth[valid]
        stride = max(1,len(depth)//2000)
        u,v,depth = u[::stride],v[::stride],depth[::stride]
        fx,fy,cx,cy = self._intrinsics()
        if fx <= 0 or fy <= 0:
            return None
        points = np.column_stack(((u-cx)*depth/fx,(v-cy)*depth/fy,depth))
        return known_cube_center(points,rotation,side)

    def _classify_shape(self, contour: np.ndarray, color: str) -> str:
        if color == 'purple':
            return 'station'

        try:
            known_shape = str(self.get_parameter('known_object_shape').value).strip()
            if known_shape:
                return known_shape
        except Exception:
            pass

        if color == 'red':
            return 'cube'
        if color == 'green':
            return 'cylinder'
        if color == 'blue':
            return 'rectangular_block'

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        if circularity > 0.72:
            return 'cylinder'
        if len(approx) == 4 and 0.8 <= aspect <= 1.25:
            return 'cube'
        return 'rectangular_block'

    def _publish(self, detections: List[Dict], debug_image: np.ndarray,
                 publish_localization: bool = True, publish_debug: bool = True) -> None:
        frame_id = self.rgb_frame or self.depth_frame or 'cam_1'
        stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()

        if publish_localization:
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

        if publish_debug and bool(self.get_parameter('publish_debug_image').value):
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
            'purple': (180, 50, 160),
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
