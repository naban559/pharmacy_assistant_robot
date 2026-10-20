"""AMMS Task Manager — explicit state-machine for A/B/C/D pick-and-place.

Architecture:

    SYSTEM_STARTUP → WAIT_FOR_LOCALIZATION → WAIT_FOR_COMMAND
        → (A/B/C) → CAPTURE_INITIAL_POSE → NAVIGATE_TO_PICKUP → SEARCH_FOR_OBJECT
            → PICK → NAVIGATE_TO_PLACE → PLACE → RETURN_TO_INITIAL_POSE → WAIT_FOR_COMMAND
        → (D)    → NAVIGATE_TO_PLACE_STATION → WAIT_FOR_COMMAND

Commands arrive via /amms/goto_a, /amms/goto_b, /amms/goto_c, /amms/goto_d
Trigger services.  The operator CLI (amms_command.py) calls those services
from a separate terminal.
"""

import json
import math
import os
import threading
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers geometry transforms
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from k12_pick_place.ik import PlanarArmIK
from k12_pick_place.station_docking import docking_command
from k12_pick_place.gazebo_observer import GazeboObserver, resolve_entities, transform_matrix


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class State(Enum):
    SYSTEM_STARTUP = auto()
    WAIT_FOR_LOCALIZATION = auto()
    WAIT_FOR_COMMAND = auto()
    CAPTURE_INITIAL_POSE = auto()
    NAVIGATE_TO_PICKUP = auto()
    SEARCH_FOR_OBJECT = auto()
    VALIDATE_OBJECT = auto()
    PICK_OBJECT = auto()
    VERIFY_PICK = auto()
    NAVIGATE_TO_PLACE = auto()
    COMPUTE_PLACE_POSE = auto()
    PLACE_OBJECT = auto()
    VERIFY_RELEASE = auto()
    RETURN_TO_INITIAL_POSE = auto()
    TASK_COMPLETE = auto()
    TASK_FAILED = auto()
    REPORT_ARRIVAL = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class AmmsTaskManager(Node):

    # A/B/C → navigation waypoint name
    _ABC_WAYPOINTS: Dict[str, str] = {
        'A': 'pickup_point_1_red',
        'B': 'pickup_point_2_green',
        'C': 'pickup_point_3_blue',
    }
    _ABC_LABELS: Dict[str, str] = {
        'A': 'Pick Point 1 - RED',
        'B': 'Pick Point 2 - GREEN',
        'C': 'Pick Point 3 - BLUE',
        'D': 'Place Station',
    }

    def __init__(self):
        super().__init__('amms_task_manager')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('use_navigation', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pick_target_topic', '/amms/pick_target')
        self.declare_parameter('task_state_topic', '/amms/task_state')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('backup_action_name', 'backup')
        self.declare_parameter('pick_service', '/amms/pick_object')
        self.declare_parameter('place_service', '/amms/place_object')
        self.declare_parameter('fold_arm_service', '/amms/fold_arm')
        self.declare_parameter('scan_topic', '/scan')

        # Waypoints (read from YAML)
        self.declare_parameter('waypoint_names', [
            'home', 'pickup_point_1_red', 'pickup_point_2_green',
            'pickup_point_3_blue', 'pick_clear', 'place_approach',
            'place_station_approach', 'place_station', 'place_clear',
        ])
        self.waypoint_names = [str(n) for n in self.get_parameter('waypoint_names').value]
        for name in self.waypoint_names:
            self.declare_parameter(f'waypoint_{name}', [0.0, 0.0, 0.0])

        # Navigation
        self.declare_parameter('nav_timeout_sec', 90.0)
        self.declare_parameter('nav_retry_count', 3)
        self.declare_parameter('nav_retry_wait_sec', 0.5)
        self.declare_parameter('nav_arrival_xy_tolerance_m', 0.08)
        self.declare_parameter('nav_arrival_yaw_tolerance_rad', 0.15)
        self.declare_parameter('docking_arrival_xy_tolerance_m', 0.06)
        self.declare_parameter('docking_arrival_yaw_tolerance_rad', 0.10)
        self.declare_parameter('post_navigation_settle_sec', 0.5)
        self.declare_parameter('clear_costmaps_on_nav_failure', True)
        self.declare_parameter('local_clear_costmap_service',
                               '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('global_clear_costmap_service',
                               '/global_costmap/clear_entirely_global_costmap')

        # Backup
        self.declare_parameter('post_pick_backup_enabled', True)
        self.declare_parameter('post_pick_backup_distance', 0.5)
        self.declare_parameter('post_pick_backup_speed', 0.06)
        self.declare_parameter('post_pick_backup_timeout_sec', 12.0)
        self.declare_parameter('post_place_backup_enabled', True)
        self.declare_parameter('post_place_backup_distance', 0.35)
        self.declare_parameter('post_place_backup_speed', 0.10)
        self.declare_parameter('post_place_backup_timeout_sec', 10.0)
        self.declare_parameter('nav2_backup_max_segment_distance', 0.20)

        # Perception
        self.declare_parameter('detection_timeout_sec', 45.0)
        self.declare_parameter('detection_freshness_sec', 0.5)
        self.declare_parameter('fresh_detection_after_move_timeout_sec', 10.0)
        self.declare_parameter('min_detection_confidence', 0.25)
        self.declare_parameter('object_target_frame', 'arm_base_link')
        self.declare_parameter('object_reachable_min_radius', 0.14)
        self.declare_parameter('object_reachable_max_radius', 0.9)
        self.declare_parameter('object_reachable_min_z', -0.31)
        self.declare_parameter('object_reachable_max_z', 0.23)
        self.declare_parameter('object_grasp_forward_offset', 0.0)
        self.declare_parameter('object_grasp_z_bias', 0.010)
        self.declare_parameter('object_grasp_surface_to_center_offset', 0.0125)
        self.declare_parameter('object_nav_standoff_distance', 0.39)
        self.declare_parameter('max_reposition_attempts', 3)
        self.declare_parameter('max_object_failures', 4)
        self.declare_parameter('pick_retry_count', 3)
        self.declare_parameter('pick_retry_wait_sec', 0.3)
        self.declare_parameter('require_reachable_before_pick_service', True)
        self.declare_parameter('pre_pick_detection_required', True)

        # Place station alignment
        self.declare_parameter('enable_final_alignment', False)
        self.declare_parameter('place_position_arm_base', [0.505, 0.0, 0.026])
        self.declare_parameter('place_slot_reach_adjustments_x',
                               [0.0, -0.026, -0.053, -0.079])

        # Purple station visual alignment & creep parameters
        self.declare_parameter('purple_station_detection_enabled', True)
        self.declare_parameter('purple_station_rgb_topic',
                               '/camera/depth_camera/image')
        self.declare_parameter('purple_hsv_lower', [115, 50, 30])
        self.declare_parameter('purple_hsv_upper', [165, 255, 255])
        self.declare_parameter('purple_min_area_px', 200.0)
        self.declare_parameter('purple_detection_timeout_sec', 25.0)

        # Creep & visual servo parameters
        self.declare_parameter('creep_linear_speed', 0.04)
        self.declare_parameter('creep_angular_gain', 0.5)
        self.declare_parameter('creep_stop_lidar_m', 0.275)  # 20cm chassis standoff (derived: 0.20m + 0.24m - 0.165m)
        self.declare_parameter('creep_stop_depth_m', 0.245)  # 20cm chassis standoff (derived: 0.20m + 0.24m - 0.195m)
        self.declare_parameter('creep_max_distance_m', 1.20)
        self.declare_parameter('creep_timeout_sec', 45.0)
        self.declare_parameter('creep_lidar_forward_half_angle', 0.20)
        self.declare_parameter('creep_depth_topic', '/camera/depth_camera/depth_image')
        self.declare_parameter('creep_camera_frame', 'camera_1_optical_frame')
        self.declare_parameter('place_station_live_pose_topic', '/amms/place_station_pose')
        self.declare_parameter('tracker_cmd_vel_topic', '/cmd_vel_tracker')

        # Startup
        self.declare_parameter('startup_timeout_sec', 300.0)
        self.declare_parameter('startup_poll_period_sec', 0.25)
        self.declare_parameter('localization_timeout_sec', 3.0)
        self.declare_parameter('obstacle_log_min_range', 0.45)
        self.declare_parameter('service_timeout_sec', 90.0)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('nav_action_name').value)
        self.backup_client = ActionClient(
            self, BackUp, self.get_parameter('backup_action_name').value)
        self.ik = PlanarArmIK()
        self.pick_client = self.create_client(
            Trigger, self.get_parameter('pick_service').value)
        self.place_client = self.create_client(
            Trigger, self.get_parameter('place_service').value)
        self.fold_client = self.create_client(
            Trigger, self.get_parameter('fold_arm_service').value)
        self.local_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('local_clear_costmap_service').value)
        self.global_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('global_clear_costmap_service').value)

        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(
            String, self.get_parameter('task_state_topic').value, state_qos)
        self.pick_target_pub = self.create_publisher(
            String, self.get_parameter('pick_target_topic').value, 10)
        self.tracker_cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('tracker_cmd_vel_topic').value, 10)
        self.live_place_pose_pub = self.create_publisher(
            PointStamped, self.get_parameter('place_station_live_pose_topic').value, 10)

        self.create_subscription(
            String, self.get_parameter('detections_topic').value,
            self._on_detections, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)

        # RGB & Depth image subscriptions for purple-station visual alignment & creep
        self._bridge = CvBridge()
        self._purple_image: Optional[np.ndarray] = None
        self._purple_image_lock = threading.Lock()
        self._depth_image: Optional[np.ndarray] = None
        self._depth_image_stamp: float = 0.0
        self._depth_image_lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None
        self._scan_lock = threading.Lock()
        self._live_place_pose: Optional[Tuple[float, float, float]] = None

        _img_qos = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Image,
            self.get_parameter('purple_station_rgb_topic').value,
            self._on_rgb_image,
            _img_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter('creep_depth_topic').value,
            self._on_depth_image,
            _img_qos,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.observer = GazeboObserver(self)

        # ------------------------------------------------------------------
        # Command services (A/B/C/D via separate terminal CLI)
        # ------------------------------------------------------------------
        self._command_event = threading.Event()
        self._pending_command: Optional[str] = None
        self._mission_busy = False
        self._goto_services = {}
        for letter in ('a', 'b', 'c', 'd'):
            svc = self.create_service(
                Trigger, f'/amms/goto_{letter}',
                lambda req, resp, l=letter.upper(): self._handle_goto(l, req, resp))
            self._goto_services[letter] = svc

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._state = State.SYSTEM_STARTUP
        self._state_pub_timer = self.create_timer(1.0, self._publish_state)
        self._start_timer = self.create_timer(0.5, self._maybe_start)

        self.latest_detection_time = 0.0
        self.latest_detection_count = 0
        self.latest_payload: Optional[Dict] = None
        self.latest_obstacle_range: Optional[float] = None
        self.last_obstacle_log_time = 0.0
        self.base_motion_active = False
        self.completed_object_ids: set = set()
        self.failed_object_counts: Dict[str, int] = {}

        self._initial_pose: Optional[Tuple[float, float, float]] = None
        self._current_command: Optional[str] = None
        self._mission_thread: Optional[threading.Thread] = None
        self._started = False
        self._purple_image_stamp: float = 0.0

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _maybe_start(self) -> None:
        if self._started:
            return
        self._started = True
        self._mission_thread = threading.Thread(
            target=self._mission_loop, daemon=True)
        self._mission_thread.start()

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state.name
        self.state_pub.publish(msg)

    def _set_state(self, state: State) -> None:
        self._state = state
        self.get_logger().info(f'State → {state.name}')
        self._publish_state()

    # ------------------------------------------------------------------
    # Command service callback
    # ------------------------------------------------------------------

    def _handle_goto(self, letter: str, request, response):
        if self._mission_busy or self._pending_command is not None:
            response.success = False
            response.message = f'Busy executing mission. Cannot accept {letter} now.'
            self.get_logger().warn(response.message)
            return response
        if self._state != State.WAIT_FOR_COMMAND or getattr(self, 'navigation_fault', False):
            response.success = False
            response.message = 'Robot is not ready for a new mission.'
            return response
        if letter not in ('A', 'B', 'C', 'D'):
            response.success = False
            response.message = f'Invalid command: {letter}'
            return response
        self._pending_command = letter
        self._command_event.set()
        response.success = True
        response.message = f'Command {letter} accepted.'
        self.get_logger().info(f'Service accepted command: {letter}')
        return response

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        objects = payload.get('objects', [])
        self.latest_payload = payload
        self.latest_detection_count = len(objects)
        if objects:
            self.latest_detection_time = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = msg
            self._scan_wall_time = time.monotonic()
        valid = [float(v) for v in msg.ranges
                 if math.isfinite(v) and msg.range_min <= float(v) <= msg.range_max]
        if valid:
            self.latest_obstacle_range = min(valid)

    def _on_depth_image(self, msg: Image) -> None:
        """Store latest depth frame thread-safely for visual-servo creep."""
        try:
            depth_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().debug(f'Depth bridge error: {exc}')
            return
        with self._depth_image_lock:
            self._depth_image = depth_frame
            self._depth_image_stamp = time.monotonic()

    def _on_rgb_image(self, msg: Image) -> None:
        """Store latest RGB frame thread-safely for purple-station detection.

        Camera topic map (from amms_gazebo.launch.py ros_gz_image_bridge):
          /camera/depth_camera/image        → sensor_msgs/Image, encoding=rgb8  (COLOR ✔)
          /camera/depth_camera/depth_image  → sensor_msgs/Image, encoding=32FC1 (DEPTH)

        ros_gz_image bridges the Gazebo R8G8B8 color stream as *rgb8* (not bgr8).
        We request 'rgb8' from CvBridge then convert to BGR so cv2.cvtColor
        (COLOR_BGR2HSV) produces correct hue values.
        """
        try:
            # Request rgb8 explicitly; then flip to BGR for OpenCV
            rgb_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            self.get_logger().debug(f'RGB bridge error: {exc}')
            return
        with self._purple_image_lock:
            self._purple_image = frame
            self._purple_image_stamp = time.monotonic()

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _current_base_pose(self) -> Optional[Tuple[float, float, float]]:
        frame_id = self.get_parameter('frame_id').value
        base_frame = self.get_parameter('base_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                frame_id, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.25))
        except Exception:
            return None
        rot = t.transform.rotation
        yaw = math.atan2(
            2.0 * (rot.w * rot.z + rot.x * rot.y),
            1.0 - 2.0 * (rot.y ** 2 + rot.z ** 2))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _physical_base_pose(self):
        pose = self.observer.get_pose('four_wheel', max_age_sec=.5)
        if pose is None:
            return None
        x, y, z, w = pose['orientation']
        return (*pose['position'][:2], math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

    def _physical_station_goal(self):
        """Register the world station waypoint into Nav2's current map frame."""
        x, y, yaw = self._waypoint('place_station')
        matrix = self.observer.world_to_frame(self, 'map')
        point = matrix @ np.array([x, y, 0., 1.])
        heading = matrix[:3, :3] @ np.array([math.cos(yaw), math.sin(yaw), 0.])
        return self._pose_stamped((point[0], point[1], math.atan2(heading[1], heading[0])))

    def _verify_localization(self) -> None:
        frame_id = self.get_parameter('frame_id').value
        base_frame = self.get_parameter('base_frame').value
        timeout = float(self.get_parameter('localization_timeout_sec').value)
        try:
            self.tf_buffer.lookup_transform(
                frame_id, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=timeout))
        except Exception as exc:
            raise RuntimeError(
                f'Localization {frame_id}→{base_frame} unavailable: {exc}') from exc

    def _transform_fixed_point(
        self, frame_id: str, position: Tuple[float, float, float],
        target_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        pt = PointStamped()
        pt.header.frame_id = frame_id
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = position
        try:
            t = self.tf_buffer.transform(
                pt, target_frame, timeout=Duration(seconds=0.25))
        except Exception:
            return None
        return (t.point.x, t.point.y, t.point.z)

    # ------------------------------------------------------------------
    # Waypoint helpers
    # ------------------------------------------------------------------

    def _waypoint(self, name: str) -> Tuple[float, float, float]:
        values = list(self.get_parameter(f'waypoint_{name}').value)
        return float(values[0]), float(values[1]), float(values[2])

    def _pose_stamped(self, xyz_yaw: Tuple[float, float, float]) -> PoseStamped:
        x, y, yaw = xyz_yaw
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _navigate_to_waypoint(self, name: str, stop_when_object_id=None, stop_on_visible=False) -> None:
        if not bool(self.get_parameter('use_navigation').value):
            return
        pose = (self._physical_station_goal() if name == 'place_station'
                else self._pose_stamped(self._waypoint(name)))
        self.get_logger().info(f'Nav → {name}: '
                               f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        self._navigate_to_pose(pose, name, stop_when_object_id, stop_on_visible)

    def _navigate_to_pose(
        self, pose: PoseStamped, label: str, stop_when_object_id=None, stop_on_visible=False,
        check_placement_platform_pos=None
    ) -> None:
        if label == 'place_station' and hasattr(self, 'observer'):
            # Nav2 owns the route to staging. Once its action has completed,
            # station-relative feedback owns this short, bounded docking move.
            # A frozen map goal cannot compensate changing map/physical drift.
            self._dock_to_station()
            self._verify_arrival(pose, label)
            self.get_logger().info(f'Nav goal {label} succeeded (physical docking).')
            return
        retry_count = max(1, int(self.get_parameter('nav_retry_count').value))
        retry_wait = max(0.0, float(self.get_parameter('nav_retry_wait_sec').value))
        timeout = float(self.get_parameter('nav_timeout_sec').value)
        for attempt in range(1, retry_count + 1):
            self.get_logger().info(f'Nav goal {label} attempt {attempt}/{retry_count}')
            try:
                if getattr(self, 'navigation_fault', False):
                    raise RuntimeError('Previous navigation did not stop; restart after checking the base')
                self._navigate_to_pose_once(
                    pose, label, timeout, stop_when_object_id, stop_on_visible, check_placement_platform_pos)
                if label in ('return_to_initial', 'place_station'):
                    self._stop_base()
                    time.sleep(float(self.get_parameter('post_navigation_settle_sec').value))
                    if label == 'place_station' and hasattr(self, 'observer'):
                        if not self._verify_and_align_heading(
                                self._waypoint('place_station')[2], physical=True):
                            raise RuntimeError('Physical station heading alignment failed')
                    self._verify_arrival(pose, label)
                self.get_logger().info(f'Nav goal {label} succeeded.')
                return
            except RuntimeError as exc:
                if getattr(self, 'navigation_fault', False):
                    raise
                self.get_logger().warn(
                    f'Nav goal {label} attempt {attempt}/{retry_count} failed: {exc}')
                if attempt < retry_count:
                    self._clear_nav_costmaps(label)
                    time.sleep(retry_wait)
        raise RuntimeError(f'Nav goal {label} failed after {retry_count} attempts')

    def _docking_clearance(self, base):
        """Check physical station clearance and the full fresh laser footprint."""
        station = self.observer.get_pose('placing_station', max_age_sec=.5)
        if station is None:
            raise RuntimeError('Physical station pose unavailable during docking')
        # Conservative full wheel rectangle, including a 25 mm station margin.
        extent_y = .255*abs(math.sin(base[2])) + .17*abs(math.cos(base[2]))
        station_front = station['position'][1] + .15
        if base[1]-extent_y < station_front+.025:
            raise RuntimeError('Docking footprint reached station clearance boundary')
        with self._scan_lock:
            scan = self._latest_scan
            received = getattr(self, '_scan_wall_time', 0.)
        if scan is None or time.monotonic()-received > .5:
            raise RuntimeError('Fresh lidar required for local docking')
        ranges = np.asarray(scan.ranges)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= min(scan.range_max, .8))
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        points = np.vstack((ranges[valid]*np.cos(angles[valid]),
                            ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum()), np.ones(valid.sum())))
        transform = transform_matrix(self.tf_buffer.lookup_transform(
            'base_footprint', scan.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=.25)))
        points = transform @ points
        # 45 mm beyond the wheels covers the low-speed stop distance and drift.
        if np.any((np.abs(points[0]) < .300) & (np.abs(points[1]) < .215)):
            raise RuntimeError('Lidar obstacle inside docking stop envelope')

    def _docking_pivot_x(self):
        # Measured pure-turn ICR agrees with the front axle: 194.997 mm in
        # the validation trace. Read its geometry from TF, not a map offset.
        positions = []
        for wheel in ('left_front_1','right_front_1'):
            t = self.tf_buffer.lookup_transform('base_footprint', wheel, rclpy.time.Time(),
                                                timeout=Duration(seconds=.5))
            positions.append(t.transform.translation.x)
        if abs(positions[0]-positions[1]) > .001 or not .15 < sum(positions)/2 < .25:
            raise RuntimeError('Unexpected front-axle geometry; docking model must be revalidated')
        return sum(positions)/2

    def _dock_to_station(self):
        goal_x, goal_y, goal_yaw = self._waypoint('place_station')
        pivot_x = self._docking_pivot_x()
        deadline = time.monotonic()+120.
        self._stop_base()
        self.base_motion_active = True
        last_log = 0.
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                base = self._physical_base_pose()
                if base is None:
                    raise RuntimeError('Fresh physical base pose unavailable during docking')
                if abs(base[0]) > .5 or not -2.35 < base[1] < -1.2:
                    raise RuntimeError(f'Base outside the bounded station approach region: {base}')
                self._docking_clearance(base)
                velocity, omega, distance, heading = docking_command(
                    base, (goal_x,goal_y,goal_yaw), pivot_x)
                if distance <= .008 and abs(heading) <= .025:
                    self._stop_base()
                    time.sleep(.5)
                    if not self._verify_and_align_heading(goal_yaw, physical=True, timeout=20.):
                        raise RuntimeError('Final physical docking heading failed')
                    self._stop_base()
                    time.sleep(.5)
                    self._docking_clearance(self._physical_base_pose())
                    return
                # Do not orbit the target or reverse toward the table after overshoot.
                if base[1] < goal_y-.015:
                    raise RuntimeError('Physical docking overshot its target; stopping with cube held')
                twist = Twist()
                twist.angular.z = omega
                twist.linear.x = velocity
                self.tracker_cmd_vel_pub.publish(twist)
                if time.monotonic()-last_log > 1.:
                    self.get_logger().info(
                        f'Physical docking: base={base}, distance={distance:.4f}m '
                        f'heading_error={heading:.4f}rad pivot_x={pivot_x:.3f}m command=({twist.linear.x:.3f},{twist.angular.z:.3f})')
                    last_log = time.monotonic()
                time.sleep(.05)
            raise RuntimeError('Physical docking timed out')
        finally:
            self._stop_base()

    def _navigate_to_pose_once(
        self, pose: PoseStamped, label: str, timeout: float, stop_when_object_id=None, stop_on_visible=False,
        check_placement_platform_pos=None,
    ) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('Nav2 NavigateToPose action server unavailable.')
        goal = NavigateToPose.Goal()
        goal.pose = pose
        tree = ('approach_grasp.xml' if label in
                ('place_station', 'reachable grasp approach', 'reachable place approach')
                else 'navigate.xml')
        goal.behavior_tree = os.path.join(
            get_package_share_directory('k12_pick_place'), 'config', tree)
        send_future = self.nav_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 10.0):
            self.navigation_fault = True
            send_future.add_done_callback(lambda done: done.result().cancel_goal_async()
                                          if done.result() and done.result().accepted else None)
            raise RuntimeError(f'Timed out sending nav goal {label}.')
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Nav goal {label} rejected.')
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout
        self.base_motion_active = True
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if result_future.done():
                    result = result_future.result()
                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                        raise RuntimeError(
                            f'Nav goal {label} failed status={result.status}.')
                    return
                if stop_when_object_id is not None:
                    candidate = self._select_valid_detection(not stop_on_visible, object_id=stop_when_object_id)
                    near_stage = True
                    if stop_on_visible:
                        base = self._current_base_pose()
                        near_stage = base is not None and math.hypot(
                            base[0]-pose.pose.position.x, base[1]-pose.pose.position.y) <= 0.40
                    if candidate is not None and near_stage:
                        self._stop_navigation_goal(goal_handle, result_future)
                        self.get_logger().info(
                            'Cube visible near pickup staging point; stopping before final waypoint rotation.'
                            if stop_on_visible else 'Cube admits a complete grasp path; stopping fine approach.')
                        return
                if label in ('place_station', 'reachable place approach'):
                    base = self._physical_base_pose()
                    if base is not None:
                        gx, gy, yaw_goal = self._waypoint('place_station')
                        yaw_err = abs(_normalize_angle(base[2] - yaw_goal))
                        dist_to_goal = math.hypot(base[0] - gx, base[1] - gy)
                        # A safety boundary always fails; it is never an arrival shortcut.
                        if base[1] <= -2.35:
                            self._stop_navigation_goal(goal_handle, result_future)
                            raise RuntimeError('Physical placing approach reached safety boundary')
                        if dist_to_goal <= 0.025:
                            self._stop_navigation_goal(goal_handle, result_future)
                            self.get_logger().info(
                                f'Placing station approach complete: base reached goal waypoint ({base[0]:.2f}, {base[1]:.2f}) '
                                f'and yaw {base[2]:.2f} rad (dist={dist_to_goal:.3f}m, yaw_err={yaw_err:.3f}rad); stopping base.')
                            return
                    lidar_range = self._min_forward_lidar_range()
                    if lidar_range is not None and lidar_range <= 0.12:
                        self._stop_navigation_goal(goal_handle, result_future)
                        raise RuntimeError(
                            f'Placing approach reached obstacle safety range {lidar_range:.2f}m <= 0.12m without completing arrival!')
                time.sleep(0.1)
            self._stop_navigation_goal(goal_handle, result_future)
            raise RuntimeError(f'Nav goal {label} timed out.')
        finally:
            self.base_motion_active = False

    def _verify_arrival(self, pose: PoseStamped, label: str) -> None:
        base = self._current_base_pose()
        if base is None:
            raise RuntimeError(f'Cannot verify arrival at {label}: localization unavailable')
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        gx, gy = pose.pose.position.x, pose.pose.position.y
        if label == 'place_station' and hasattr(self, 'observer'):
            base = self._physical_base_pose()
            if base is None:
                raise RuntimeError('Physical docking pose unavailable')
            gx, gy, yaw = self._waypoint('place_station')
        distance = math.hypot(base[0]-gx, base[1]-gy)
        heading = abs(_normalize_angle(base[2]-yaw))
        prefix = 'docking' if label in ('place_station', 'reachable place approach') else 'nav'
        xy_tol = float(self.get_parameter(f'{prefix}_arrival_xy_tolerance_m').value)
        yaw_tol = float(self.get_parameter(f'{prefix}_arrival_yaw_tolerance_rad').value)
        if distance > xy_tol or heading > yaw_tol:
            raise RuntimeError(f'{label} outside the measured arrival tolerance: '
                               f'position error={distance:.3f} m (tol={xy_tol:.3f}), '
                               f'yaw error={heading:.3f} rad (tol={yaw_tol:.3f})')
        self.get_logger().info(
            f'Arrival verified at {label}: position=({base[0]:.3f}, {base[1]:.3f}) '
            f'yaw={base[2]:.2f} rad (distance error={distance:.3f} m, heading error={heading:.3f} rad).')

    def _stop_navigation_goal(self, goal_handle, result_future):
        self._stop_base()
        cancel_future = goal_handle.cancel_goal_async()
        if (not self._wait_for_future(cancel_future, 3.0)
                or not self._wait_for_future(result_future, 5.0)):
            self.navigation_fault = True
            self._stop_base()
            raise RuntimeError('Navigation cancellation was not acknowledged; no arm motion allowed')
        result = result_future.result()
        if result is None or result.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
            raise RuntimeError('Navigation did not finish cleanly during approach cancellation')
        self._stop_base()

    def _clear_nav_costmaps(self, label: str) -> None:
        if not bool(self.get_parameter('clear_costmaps_on_nav_failure').value):
            return
        for client, svc in (
            (self.local_clear_costmap_client,
             self.get_parameter('local_clear_costmap_service').value),
            (self.global_clear_costmap_client,
             self.get_parameter('global_clear_costmap_service').value),
        ):
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(ClearEntireCostmap.Request())
                self._wait_for_future(future, 2.0)

    # ------------------------------------------------------------------
    # Backup
    # ------------------------------------------------------------------

    def _backup(self, distance_param: str, speed_param: str,
                timeout_param: str, label: str) -> None:
        if not bool(self.get_parameter('use_navigation').value):
            return
        distance = abs(float(self.get_parameter(distance_param).value))
        speed = abs(float(self.get_parameter(speed_param).value))
        timeout = max(1.0, float(self.get_parameter(timeout_param).value))
        if distance <= 0 or speed <= 0:
            return
        if not self.backup_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'BackUp action unavailable for {label}.')
        goal = BackUp.Goal()
        goal.target.x = -distance
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = speed
        goal.time_allowance.sec = int(timeout)
        send_future = self.backup_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            self.navigation_fault = True
            send_future.add_done_callback(lambda done: done.result().cancel_goal_async()
                                          if done.result() and done.result().accepted else None)
            raise RuntimeError(f'Timed out sending {label}.')
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{label} rejected.')
        self.base_motion_active = True
        try:
            result_future = goal_handle.get_result_async()
            # Nav2's allowance uses simulation time; a slow simulator needs a
            # longer wall-clock wait. Never start another motion while backing up.
            wall_timeout = max(timeout + 2.0, float(self.get_parameter('nav_timeout_sec').value))
            if not self._wait_for_future(result_future, wall_timeout):
                self._stop_navigation_goal(goal_handle, result_future)
                raise RuntimeError(f'{label} timed out.')
            result = result_future.result()
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'{label} failed: status={getattr(result, "status", None)}')
        finally:
            self.base_motion_active = False
            self._stop_base()

    # ------------------------------------------------------------------
    # Service / action helpers
    # ------------------------------------------------------------------

    def _call_trigger(self, client: object, label: str) -> None:
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'Service for {label} unavailable.')
        future = client.call_async(Trigger.Request())
        timeout = float(self.get_parameter('service_timeout_sec').value)
        if not self._wait_for_future(future, timeout):
            raise RuntimeError(f'Service call {label} timed out.')
        result = future.result()
        if result is None or not result.success:
            msg = '' if result is None else result.message
            raise RuntimeError(f'Service {label} failed: {msg}')

    def _try_call_trigger(self, client: object, label: str) -> bool:
        try:
            self._call_trigger(client, label)
            return True
        except RuntimeError as exc:
            self.get_logger().warn(f'{label} skipped: {exc}')
            return False

    def _fold_before_driving(self) -> None:
        deadline = time.monotonic() + 30.0
        while rclpy.ok():
            try:
                self._call_trigger(self.fold_client, 'fold arm')
                return
            except RuntimeError as exc:
                if 'Manipulator is already executing' not in str(exc) or time.monotonic() >= deadline:
                    raise
                time.sleep(0.25)
        raise RuntimeError('Shutdown while waiting for folded arm')

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    # ------------------------------------------------------------------
    # Perception helpers
    # ------------------------------------------------------------------

    def _detection_candidates(self, payload: Dict) -> List[Dict]:
        target_frame = self.get_parameter('object_target_frame').value
        map_frame = self.get_parameter('frame_id').value
        map_positions = {}
        for obj, pos in self._transform_objects(payload, map_frame):
            map_positions[self._object_id(obj)] = pos
        candidates = []
        for obj, pos in self._transform_objects(payload, target_frame):
            oid = self._object_id(obj)
            grasp = self._grasp_pose_from_object(pos, obj)
            reachable = self._object_within_workspace(grasp)
            confidence = float(obj.get('confidence', 1.0))
            candidates.append({
                'object': obj, 'object_id': oid, 'confidence': confidence,
                'arm_position': pos, 'map_position': map_positions.get(oid),
                'grasp': grasp, 'reachable': reachable,
                'distance': math.hypot(grasp[0], grasp[1]),
            })
        return candidates

    def _select_valid_detection(
        self, require_reachable: bool, object_id: Optional[str] = None,
        min_detection_time: float = 0.0,
    ) -> Optional[Dict]:
        freshness = float(self.get_parameter('detection_freshness_sec').value)
        if (self.latest_payload is None
                or self.latest_detection_count <= 0
                or time.monotonic() - self.latest_detection_time > freshness
                or self.latest_detection_time < min_detection_time):
            return None
        candidates = self._detection_candidates(self.latest_payload)
        if object_id is not None:
            candidates = [c for c in candidates
                          if c['object_id'] == object_id
                          or str(c.get('object', {}).get('color', '')).lower() == object_id.lower()
                          or object_id.lower().startswith(str(c.get('object', {}).get('color', 'xxx')).lower())]
        valid = [c for c in candidates
                 if self._candidate_is_valid(c, require_reachable)]
        if not valid:
            return None
        valid.sort(key=lambda c: (not c['reachable'], c['distance']))
        return valid[0]

    def _wait_for_valid_detection(
        self, require_reachable: bool,
        timeout_override: Optional[float] = None,
        object_id: Optional[str] = None,
        min_detection_time: float = 0.0,
    ) -> Optional[Dict]:
        start = time.monotonic()
        timeout = (timeout_override if timeout_override is not None
                   else float(self.get_parameter('detection_timeout_sec').value))
        while rclpy.ok() and time.monotonic() - start < timeout:
            c = self._select_valid_detection(
                require_reachable, object_id, min_detection_time)
            if c is not None:
                return c
            time.sleep(0.1)
        return None

    def _update_candidate_for_current_pose(self, candidate: Optional[Dict]) -> Optional[Dict]:
        if candidate is None:
            return None
        map_pos = candidate.get('map_position')
        if map_pos is None:
            return candidate
        target_frame = self.get_parameter('object_target_frame').value
        pt = PointStamped()
        pt.header.frame_id = self.get_parameter('frame_id').value
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(map_pos[0])
        pt.point.y = float(map_pos[1])
        pt.point.z = float(map_pos[2])
        try:
            t = self.tf_buffer.transform(pt, target_frame, timeout=Duration(seconds=0.5))
            arm_pos = (t.point.x, t.point.y, t.point.z)
            grasp = self._grasp_pose_from_object(arm_pos, candidate.get('object'))
            reachable = self._object_within_workspace(grasp)
            updated = dict(candidate)
            updated['arm_position'] = arm_pos
            updated['grasp'] = grasp
            updated['reachable'] = reachable
            return updated
        except Exception as exc:
            self.get_logger().warn(f'Could not transform map_position to {target_frame}: {exc}')
            return candidate

    def _acquire_reachable_object(self, seed=None):
        target = seed or self._wait_for_valid_detection(False)
        if target is None:
            return None
        attempts = max(0, int(self.get_parameter('max_reposition_attempts').value))
        for attempt in range(attempts+1):
            target = self._update_candidate_for_current_pose(target)
            if self._candidate_is_valid(target, True):
                return target
            if target is None:
                return None
            self.get_logger().warn(
                f'{target.get("object_id", "object")} detected but grasp rejected: '
                f'arm target={target.get("grasp")}, reachable={target.get("reachable")}, '
                f'confidence={target.get("confidence")}.')
            if attempt == attempts or not bool(self.get_parameter('use_navigation').value):
                return None
            base = self._current_base_pose()
            obj = target.get('map_position')
            if base is None or obj is None:
                return None
            # Face the measured cube to keep its full silhouette in view.
            # Wrist alignment is selected before pre-grasp, then preserved.
            yaw = math.atan2(obj[1]-base[1], obj[0]-base[0])
            standoff = float(self.get_parameter('object_nav_standoff_distance').value)
            pose = PoseStamped()
            pose.header.frame_id = self.get_parameter('frame_id').value
            pose.header.stamp = rclpy.time.Time().to_msg()
            pose.pose.position.x = obj[0] - standoff*math.cos(yaw)
            pose.pose.position.y = min(2.35, obj[1] - standoff*math.sin(yaw))
            pose.pose.orientation.z = math.sin(yaw/2)
            pose.pose.orientation.w = math.cos(yaw/2)
            self.get_logger().info(f'Repositioning for complete vertical grasp, attempt {attempt+1}')
            self._navigate_to_pose(pose, 'reachable grasp approach',
                                   stop_when_object_id=self._object_id(target['object']))
            self._stop_base()
            stopped_at = time.monotonic()
            fresh = self._wait_for_valid_detection(
                False, object_id=self._object_id(target['object']),
                min_detection_time=stopped_at, timeout_override=10.0)
            if fresh is not None:
                target = fresh
            else:
                self.get_logger().warn('No fresh 3D detection after approach; pickup aborted.')
                return None
        return None

    def _candidate_is_valid(self, c: Dict, require_reachable: bool) -> bool:
        if c is None:
            return False
        oid = c.get('object_id', '')
        if oid in self.completed_object_ids:
            return False
        if self.failed_object_counts.get(oid, 0) >= int(
                self.get_parameter('max_object_failures').value):
            return False
        if float(c.get('confidence', 0)) < float(
                self.get_parameter('min_detection_confidence').value):
            return False
        if c.get('map_position') is None:
            return False
        if require_reachable and c.get('object', {}).get('center_estimate_valid') is False:
            if not c.get('reachable', False):
                return False
        if require_reachable and not c.get('reachable', False):
            return False
        return True

    def _confirm_fresh_target(self, object_id: str, fallback_candidate: Optional[Dict] = None) -> Optional[Dict]:
        fresh = self._wait_for_valid_detection(
            True, object_id=object_id,
            min_detection_time=time.monotonic(),
            timeout_override=float(
                self.get_parameter('fresh_detection_after_move_timeout_sec').value))
        if fresh is not None:
            return fresh
        return None

    def _record_object_failure(self, object_id: str) -> None:
        self.failed_object_counts[object_id] = \
            self.failed_object_counts.get(object_id, 0) + 1

    @staticmethod
    def _object_id(obj: Dict) -> str:
        oid = str(obj.get('id', '')).strip()
        return oid if oid else f'{obj.get("color", "unk")}_{obj.get("shape", "obj")}'

    def _grasp_pose_from_object(
        self, pos: Tuple[float, float, float], obj: Optional[Dict] = None,
    ) -> Tuple[float, float, float]:
        x, y, z = pos
        fwd = max(0.0, float(self.get_parameter('object_grasp_forward_offset').value))
        center = self._center_offset_for_object(obj)
        r = max(math.hypot(x, y), 1e-6)
        net = center - fwd
        return (x + net * x / r, y + net * y / r,
                z + float(self.get_parameter('object_grasp_z_bias').value))

    def _center_offset_for_object(self, obj: Optional[Dict]) -> float:
        if obj is not None and obj.get('position_is_center', False):
            return 0.0
        return max(0.0, float(
            self.get_parameter('object_grasp_surface_to_center_offset').value))

    def _object_within_workspace(self, pt: Tuple[float, float, float]) -> bool:
        x, y, z = pt
        r = math.hypot(x, y)
        return (
            float(self.get_parameter('object_reachable_min_radius').value) <= r
            <= float(self.get_parameter('object_reachable_max_radius').value)
            and float(self.get_parameter('object_reachable_min_z').value) <= z
            <= float(self.get_parameter('object_reachable_max_z').value)
            and self.ik.pick_plan(pt) is not None)

    def _transform_map_point_to_arm_base(self, pt_map: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        target_frame = self.get_parameter('object_target_frame').value
        frame_id = self.get_parameter('frame_id').value
        if frame_id == target_frame:
            return pt_map
        pt = PointStamped()
        pt.header.frame_id = frame_id
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x, pt.point.y, pt.point.z = float(pt_map[0]), float(pt_map[1]), float(pt_map[2])
        try:
            t = self.tf_buffer.transform(pt, target_frame, timeout=Duration(seconds=0.5))
            return (t.point.x, t.point.y, t.point.z)
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.5))
                tr = t.transform.translation
                rot = t.transform.rotation
                qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
                r00 = 1.0 - 2.0 * (qy*qy + qz*qz)
                r01 = 2.0 * (qx*qy - qz*qw)
                r02 = 2.0 * (qx*qz + qy*qw)
                r10 = 2.0 * (qx*qy + qz*qw)
                r11 = 1.0 - 2.0 * (qx*qx + qz*qz)
                r12 = 2.0 * (qy*qz - qx*qw)
                r20 = 2.0 * (qx*qz - qy*qw)
                r21 = 2.0 * (qy*qz + qx*qw)
                r22 = 1.0 - 2.0 * (qx*qx + qy*qy)
                px, py, pz = float(pt_map[0]), float(pt_map[1]), float(pt_map[2])
                return (
                    r00 * px + r01 * py + r02 * pz + tr.x,
                    r10 * px + r11 * py + r12 * pz + tr.y,
                    r20 * px + r21 * py + r22 * pz + tr.z,
                )
            except Exception as exc:
                self.get_logger().warn(f'TF transform {frame_id} -> {target_frame} failed: {exc}')
                return None

    def _is_placement_reachable(self, platform_pos: Tuple[float, float, float]) -> bool:
        """Check if platform placement pose is reachable by arm IK from current base pose."""
        try:
            # Target release height in map frame: platform top (0.115) + cube half height (0.0125) + clearance (0.004) = 0.1315
            # TCP in map frame: + 0.010 grasp_z_bias = 0.1415
            min_r = float(self.get_parameter('object_reachable_min_radius').value)
            max_r = float(self.get_parameter('object_reachable_max_radius').value)
            min_z = float(self.get_parameter('object_reachable_min_z').value)
            max_z = float(self.get_parameter('object_reachable_max_z').value)
            pitches_to_try = (math.pi / 2,)

            # Require the exact platform centre and the same downward orientation as execution.
            candidate_offsets = [(0.0, 0.0)]
            for dx, dy in candidate_offsets:
                target_map = (float(platform_pos[0]) + dx, float(platform_pos[1]) + dy, float(platform_pos[2]) + .0265)
                pt_arm = self._transform_map_point_to_arm_base(target_map)
                if pt_arm is None:
                    continue
                r = math.hypot(pt_arm[0], pt_arm[1])
                if not (min_r <= r <= max_r and min_z <= pt_arm[2] <= max_z):
                    continue
                # Full trajectory IK check (approach, descend, lift/retreat)
                plan = self.ik.pick_plan(pt_arm, approach=0.025, lift=0.025, pitches=pitches_to_try)
                if plan is not None:
                    self.get_logger().info(
                        f'Placement capability check: target_map=({target_map[0]:.3f}, {target_map[1]:.3f}, {target_map[2]:.3f}) '
                        f'-> pt_arm=({pt_arm[0]:.3f}, {pt_arm[1]:.3f}, {pt_arm[2]:.3f}): complete IK trajectory is valid!')
                    return True
            return False
        except Exception as exc:
            self.get_logger().warn(f'Placement reachability check failed with exception: {exc}')
            return False

    def _transform_objects(
        self, payload: Dict, target_frame: str,
    ) -> List[Tuple[Dict, Tuple[float, float, float]]]:
        frame_id = payload.get('header', {}).get('frame_id', '')
        objects = payload.get('objects', [])
        if not frame_id or not objects:
            return []
        result = []
        for obj in objects:
            pos = obj.get('position', {})
            pt = PointStamped()
            pt.header.frame_id = frame_id
            stamp = payload.get('header', {}).get('stamp', {})
            pt.header.stamp = rclpy.time.Time(
                seconds=int(stamp.get('sec', 0)),
                nanoseconds=int(stamp.get('nanosec', 0))).to_msg()
            pt.point.x = float(pos.get('x', 0))
            pt.point.y = float(pos.get('y', 0))
            pt.point.z = float(pos.get('z', 0))
            try:
                t = self.tf_buffer.transform(
                    pt, target_frame, timeout=Duration(seconds=0.25))
            except Exception:
                continue
            result.append((obj, (t.point.x, t.point.y, t.point.z)))
        return result

    def _publish_pick_target(self, candidate: Dict) -> None:
        grasp = tuple(float(v) for v in candidate['grasp'])
        arm_pos = tuple(float(v) for v in candidate.get('arm_position', grasp))
        payload = {
            'object_id': candidate.get('object_id', ''),
            'object': candidate.get('object', {}),
            'target_frame': self.get_parameter('object_target_frame').value,
            'arm_base_position': list(grasp),
            'object_arm_base_position': list(arm_pos),
            'map_position': (None if candidate.get('map_position') is None
                             else [float(v) for v in candidate['map_position']]),
            'confidence': float(candidate.get('confidence', 1.0)),
            'reachable': bool(candidate.get('reachable', False)),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pick_target_pub.publish(msg)

    # ------------------------------------------------------------------
    # Place alignment
    # ------------------------------------------------------------------

    def _find_placing_platform(self, target_color: str = 'red') -> Tuple[float, float, float]:
        """Measure the matching platform using the explicit Gazebo simulation backend.

        Returns (x, y, z) in map frame.
        Publishes the detected platform position to live_place_pose_pub.
        """
        # Explicit simulation observation backend. The cube detector assumes a
        # 25 mm cube and cannot estimate the centre of a 60 mm flat platform.
        # Never label a fixed world prior or cube bounding box as a platform detection.
        _, platform, _ = resolve_entities(target_color)
        measured = self.observer.get_pose(platform, max_age_sec=.5)
        if measured is None:
            raise RuntimeError(f'Fresh physical platform observation unavailable: {platform}')
        from k12_pick_place.gazebo_observer import pose_matrix
        surface = (pose_matrix(measured['position'], measured['orientation']) @
                   np.array([0., 0., .0075, 1.]))[:3]
        result = self.observer.world_point_in_frame(self, surface, 'map')
        self.get_logger().info(
            f'Platform observation backend=GazeboSim entity={platform} '
            f'world_surface={surface.tolist()} registered_map_surface={result}')

        pt_msg = PointStamped()
        pt_msg.header.frame_id = 'map'
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.point.x = result[0]
        pt_msg.point.y = result[1]
        pt_msg.point.z = result[2]
        self.live_place_pose_pub.publish(pt_msg)
        return result

    def _align_to_place_station(self, target_color: str = 'red') -> Tuple[float, float, float]:
        return self._find_placing_platform(target_color)

    # ------------------------------------------------------------------
    # Purple station visual detection and lateral alignment
    # ------------------------------------------------------------------

    def _detect_purple_station_centre(
        self,
    ) -> Optional[Tuple[int, int, int, int]]:
        """Detect the purple placing station in the latest RGB frame.

        Returns (cx_px, cy_px, img_width, img_height) of the largest purple
        region found, or None if no frame is available or no purple detected.
        """
        with self._purple_image_lock:
            frame = self._purple_image
            stamp = self._purple_image_stamp

        if frame is None:
            return None
        # Reject stale frames (older than 0.5 s)
        if time.monotonic() - stamp > 0.5:
            return None

        lower = self.get_parameter('purple_hsv_lower').value
        upper = self.get_parameter('purple_hsv_upper').value
        min_area = float(self.get_parameter('purple_min_area_px').value)

        # Staleness guard: reject frames older than 1.0 s
        # (camera runs at 30 Hz; 1.0 s is generous to handle thread scheduling)
        if time.monotonic() - stamp > 1.0:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < min_area:
            return None

        moments = cv2.moments(best)
        if abs(moments['m00']) < 1e-6:
            return None

        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        h, w = frame.shape[:2]
        return (cx, cy, w, h)

    def _align_to_purple_station(self) -> None:
        """Detect the purple placing station visually.

        Does NOT drive the base.  Simply waits for a valid purple detection,
        logs the centroid offset, and returns so the arm can place.
        If purple is never detected, logs a warning and returns anyway —
        the pre-computed arm reach handles placement.
        """
        if not bool(self.get_parameter('purple_station_detection_enabled').value):
            return

        det_timeout = float(self.get_parameter('purple_detection_timeout_sec').value)
        self.get_logger().info('Purple detection: waiting for visual confirmation…')

        deadline = time.monotonic() + det_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            detection = self._detect_purple_station_centre()
            if detection is not None:
                cx_px, cy_px, img_w, img_h = detection
                err_px = cx_px - img_w / 2.0
                self.get_logger().info(
                    f'Purple station detected: centroid=({cx_px}, {cy_px}) '
                    f'img_center_x={img_w / 2:.0f} lateral_error={err_px:+.0f}px')
                return
            time.sleep(0.1)

        self.get_logger().warn(
            'Purple station not detected within timeout — '
            'proceeding with pre-computed arm reach.')

    # ------------------------------------------------------------------
    # Visual-servo creep & placement pose extraction
    # ------------------------------------------------------------------

    def _median_depth_at_pixel(self, u: int, v: int, depth_frame: np.ndarray) -> Optional[float]:
        if depth_frame is None or u is None or v is None:
            return None
        h, w = depth_frame.shape[:2]
        u_min, u_max = max(0, u - 5), min(w, u + 6)
        v_min, v_max = max(0, v - 5), min(h, v + 6)
        patch = depth_frame[v_min:v_max, u_min:u_max]
        if patch.size == 0:
            return None
        valid = patch[
            np.isfinite(patch) & (patch > 0.05) & (patch < 20.0)
        ]
        if len(valid) < 5:
            return None
        return float(np.median(valid))

    def _min_forward_lidar_range(self) -> Optional[float]:
        with self._scan_lock:
            scan = self._latest_scan
        if scan is None:
            return None
        half_angle = float(self.get_parameter('creep_lidar_forward_half_angle').value)
        valid = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < 0.05 or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(_normalize_angle(angle)) <= half_angle:
                valid.append(float(r))
        return min(valid) if valid else None

    def _project_to_arm_base(
        self, u: Optional[int], v: Optional[int], depth: Optional[float],
    ) -> Optional[Tuple[float, float, float]]:
        if u is None or v is None or depth is None or depth <= 0:
            return None
        img_w, img_h = 640, 480
        fov = 1.3962634
        fx = (img_w / 2.0) / math.tan(fov / 2.0)
        fy = fx
        cx = img_w / 2.0
        cy = img_h / 2.0

        x_opt = (u - cx) * depth / fx
        y_opt = (v - cy) * depth / fy
        z_opt = depth

        pt = PointStamped()
        pt.header.frame_id = self.get_parameter('creep_camera_frame').value
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(x_opt)
        pt.point.y = float(y_opt)
        pt.point.z = float(z_opt)

        target_frame = self.get_parameter('object_target_frame').value
        try:
            t = self.tf_buffer.transform(pt, target_frame, timeout=Duration(seconds=0.25))
            return (t.point.x, t.point.y, t.point.z)
        except Exception as exc:
            self.get_logger().warn(f'TF transform camera -> {target_frame} failed: {exc}')
            return None

    def _creep_to_place_station(self) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
        linear_speed = float(self.get_parameter('creep_linear_speed').value)
        angular_gain = float(self.get_parameter('creep_angular_gain').value)
        stop_lidar_m = float(self.get_parameter('creep_stop_lidar_m').value)
        stop_depth_m = float(self.get_parameter('creep_stop_depth_m').value)
        max_dist_m = float(self.get_parameter('creep_max_distance_m').value)
        timeout_sec = float(self.get_parameter('creep_timeout_sec').value)

        start_time = time.monotonic()
        distance_driven = 0.0
        stop_twist = Twist()
        purple_lost_start = None

        cx_px = cy_px = img_w = img_h = None
        last_good_depth = None
        has_vision_fix = False

        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            if elapsed > timeout_sec or distance_driven >= max_dist_m:
                self.tracker_cmd_vel_pub.publish(stop_twist)
                self.get_logger().warn(
                    f'[creep] Aborting: elapsed={elapsed:.1f}s, distance={distance_driven:.2f}m')
                return False, None

            # Primary stop: Lidar forward range
            fwd_range = self._min_forward_lidar_range()
            if fwd_range is not None and fwd_range <= stop_lidar_m:
                self.tracker_cmd_vel_pub.publish(stop_twist)
                live_pose = (self._project_to_arm_base(cx_px, cy_px, last_good_depth)
                             if has_vision_fix else None)
                self.get_logger().info(
                    f'[creep] Lidar stop at {fwd_range:.3f} m (<= {stop_lidar_m:.3f}); vision_fix={has_vision_fix}')
                return True, live_pose

            # Purple station visual detection for heading correction
            detection = self._detect_purple_station_centre()
            if detection is None:
                self.tracker_cmd_vel_pub.publish(stop_twist)
                now = time.monotonic()
                if purple_lost_start is None:
                    purple_lost_start = now
                elif now - purple_lost_start > 2.0:
                    self.get_logger().warn('[creep] Lost purple station for > 2.0s')
                    return False, None
                time.sleep(0.05)
                continue
            purple_lost_start = None
            cx_px, cy_px, img_w, img_h = detection
            has_vision_fix = True

            # Secondary stop: Camera depth cross-check
            with self._depth_image_lock:
                depth_frame = self._depth_image
                depth_stamp = self._depth_image_stamp

            depth = None
            if depth_frame is not None and (time.monotonic() - depth_stamp <= 0.5):
                depth = self._median_depth_at_pixel(cx_px, cy_px, depth_frame)

            if depth is not None:
                last_good_depth = depth
            if depth is not None and depth <= stop_depth_m:
                self.tracker_cmd_vel_pub.publish(stop_twist)
                live_pose = self._project_to_arm_base(cx_px, cy_px, depth)
                self.get_logger().info(
                    f'[creep] Camera depth stop at {depth:.3f} m (<= {stop_depth_m:.3f})')
                return True, live_pose

            # Steering correction + step forward
            err_norm = (cx_px - img_w / 2.0) / float(img_w)
            angular_z = -angular_gain * err_norm
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_z
            self.tracker_cmd_vel_pub.publish(twist)
            time.sleep(0.05)
            distance_driven += linear_speed * 0.05

    # ------------------------------------------------------------------
    # Main mission loop
    # ------------------------------------------------------------------

    def _mission_loop(self) -> None:
        try:
            self._wait_for_startup_readiness()
            self._set_state(State.WAIT_FOR_LOCALIZATION)
            self._verify_localization()
            self.get_logger().info('Startup complete. Waiting for commands via /amms/goto_{a,b,c,d}')

            while rclpy.ok():
                self._set_state(State.WAIT_FOR_COMMAND)
                self._command_event.clear()
                self._command_event.wait(timeout=1.0)
                if not rclpy.ok():
                    break

                raw = self._pending_command
                self._pending_command = None
                if raw is None:
                    continue

                self._current_command = raw
                self._mission_busy = True
                try:
                    if raw in self._ABC_WAYPOINTS:
                        self._run_pick_place_mission(raw)
                    else:
                        self._run_place_only_mission()
                except Exception as exc:
                    self._stop_base()
                    self._set_state(State.TASK_FAILED)
                    self.get_logger().error(f'Mission failed: {exc}')
                finally:
                    self.base_motion_active = False
                    self._mission_busy = False
                    self._stop_base()
        finally:
            self._stop_base()

    # ------------------------------------------------------------------
    # Pick-and-place mission (A/B/C)
    # ------------------------------------------------------------------

    def _run_pick_place_mission(self, cmd: str) -> None:
        label = self._ABC_LABELS[cmd]
        waypoint = self._ABC_WAYPOINTS[cmd]

        self.get_logger().info(f'[{cmd}] {label}')

        # Capture initial pose
        self._set_state(State.CAPTURE_INITIAL_POSE)
        self._initial_pose = self._current_base_pose()
        if self._initial_pose is not None:
            self.get_logger().info(
                f'Initial pose: ({self._initial_pose[0]:.2f}, '
                f'{self._initial_pose[1]:.2f}, yaw={self._initial_pose[2]:.2f})')
        else:
            raise RuntimeError('Could not capture initial pose; mission cannot guarantee return')

        # Fold arm
        self._fold_before_driving()

        # Navigate to pickup zone
        self._set_state(State.NAVIGATE_TO_PICKUP)
        self.get_logger().info(f'Navigating to {label}...')
        expected_id = {'A': 'red_cube', 'B': 'green_cube', 'C': 'blue_cube'}[cmd]
        self._navigate_to_waypoint(waypoint, stop_when_object_id=expected_id, stop_on_visible=False)
        self._stop_base()
        stopped_at = time.monotonic()

        # Search for object
        self._set_state(State.SEARCH_FOR_OBJECT)
        self._verify_localization()
        self.get_logger().info('Searching for object...')
        seed = self._wait_for_valid_detection(False, object_id=expected_id,
                                              min_detection_time=stopped_at, timeout_override=10.0)
        target = self._acquire_reachable_object(seed) if seed is not None else None
        if target is None:
            self.get_logger().warn(f'No reachable grasp acquired at {label}. Retrying perception and approach...')
            time.sleep(1.0)
            seed = self._wait_for_valid_detection(False, object_id=expected_id,
                                                  min_detection_time=time.monotonic(), timeout_override=10.0)
            target = self._acquire_reachable_object(seed) if seed is not None else None

        if target is None:
            self.get_logger().warn(f'No reachable grasp acquired at {label} after retries; see detection/approach diagnostics.')
            self.get_logger().info('Returning to initial pose...')
            self._return_to_initial()
            return

        oid = self._object_id(target['object'])
        self.get_logger().info(
            f'Object detected: {oid} color={target["object"].get("color", "?")} '
            f'arm_pos=({target["arm_position"][0]:.3f}, '
            f'{target["arm_position"][1]:.3f}, '
            f'{target["arm_position"][2]:.3f})')

        # Validate
        self._set_state(State.VALIDATE_OBJECT)
        target = self._update_candidate_for_current_pose(target)
        if not self._candidate_is_valid(target, require_reachable=True):
            self._record_object_failure(oid)
            raise RuntimeError('Object failed final grasp validation')

        # Confirm fresh detection
        confirmed = self._confirm_fresh_target(oid, fallback_candidate=target)
        if confirmed is None:
            self.get_logger().warn('Fresh detection confirmation failed.')
            self._record_object_failure(oid)
            self.get_logger().info('Returning to initial pose...')
            self._return_to_initial()
            return

        self.get_logger().info('Object reachable.')

        # Pick
        self._set_state(State.PICK_OBJECT)
        self._stop_base()
        self._verify_localization()
        self._publish_pick_target(confirmed)
        self.get_logger().info('Picking...')
        try:
            self._call_trigger(self.pick_client, 'pick object')
        except RuntimeError as exc:
            self._record_object_failure(oid)
            self.get_logger().error(f'Pick failed: {exc}')
            self._try_call_trigger(self.fold_client, 'recover arm')
            self.get_logger().info('Returning to initial pose...')
            self._return_to_initial()
            return

        # Verify pick
        self._set_state(State.VERIFY_PICK)
        self.get_logger().info('Pick successful.')

        # Retract and backup
        self._fold_before_driving()
        if bool(self.get_parameter('post_pick_backup_enabled').value):
            self._backup('post_pick_backup_distance', 'post_pick_backup_speed',
                         'post_pick_backup_timeout_sec', 'post-pick backup')

        # Stage 1: Navigate to place station approach waypoint
        self._set_state(State.NAVIGATE_TO_PLACE)
        self.get_logger().info('Navigating to Place Station Approach...')
        self._live_place_pose = None  # explicit reset before attempt
        self._navigate_to_waypoint('place_station_approach')
        self._stop_base()
        time.sleep(float(self.get_parameter('post_navigation_settle_sec').value))

        # Camera-based alignment & matching platform detection
        target_color = {'A': 'red', 'B': 'green', 'C': 'blue'}.get(cmd, 'red')
        self.get_logger().info(f'Searching for purple placing station and matching {target_color} platform...')
        platform_pos = self._find_placing_platform(target_color)

        # Stage 2: Approach the fixed placing station waypoint (0.00, -2.25, -1.5708)
        approach_pose = self._physical_station_goal()
        self.get_logger().info(
            f'Physical station docking from Nav2 staging; diagnostic map goal '
            f'({approach_pose.pose.position.x:.2f}, {approach_pose.pose.position.y:.2f}, yaw=-1.57)...')

        self._navigate_to_pose(approach_pose, 'place_station')
        self._stop_base()
        time.sleep(float(self.get_parameter('post_navigation_settle_sec').value))

        # Navigation verifies physical position and performs one heading alignment
        # after cancellation, before accepting arrival. No duplicate rotation here.

        # Refresh platform alignment from verified close-up pose
        platform_pos = self._find_placing_platform(target_color)

        # Step 4: Use actual TF to verify that the complete placing trajectory is reachable
        if not self._is_placement_reachable(platform_pos):
            base = self._current_base_pose()
            self.get_logger().error(
                f'Placement aborted: target {target_color} platform at {platform_pos} not reachable via IK '
                f'from current base pose {base}. Holding position safely without moving forward or returning home.')
            self._set_state(State.TASK_FAILED)
            return

        # Step 5: Execute placement immediately
        self._set_state(State.COMPUTE_PLACE_POSE)
        self._verify_localization()
        self.get_logger().info('Placement trajectory validated via live TF. Initiating placement...')
        self._set_state(State.PLACE_OBJECT)

        try:
            self._call_trigger(self.place_client, 'place object')
        except RuntimeError as exc:
            self.get_logger().error(
                f'Place execution failed: {exc}. Safely folding arm and holding position in place.')
            self._try_call_trigger(self.fold_client, 'recover arm')
            self._set_state(State.TASK_FAILED)
            return

        # Verify release
        self._set_state(State.VERIFY_RELEASE)
        self.completed_object_ids.add(oid)
        self.get_logger().info('Placement complete.')

        # Retract and backup
        self._fold_before_driving()
        if bool(self.get_parameter('post_place_backup_enabled').value):
            self._backup('post_place_backup_distance', 'post_place_backup_speed',
                         'post_place_backup_timeout_sec', 'post-place backup')

        # Return to initial pose
        self._set_state(State.RETURN_TO_INITIAL_POSE)
        self.get_logger().info('Returning to initial pose...')
        self._return_to_initial()

        self._set_state(State.TASK_COMPLETE)
        self.get_logger().info(f'Task {cmd} complete. Robot returned to initial pose.')

    # ------------------------------------------------------------------
    # Place-only mission (D)
    # ------------------------------------------------------------------

    def _run_place_only_mission(self) -> None:
        label = self._ABC_LABELS['D']
        self.get_logger().info(f'[D] {label}')
        self.get_logger().info(f'Navigating to {label} approach...')
        self._set_state(State.NAVIGATE_TO_PLACE)
        self._live_place_pose = None  # explicit reset
        self._navigate_to_waypoint('place_station_approach')
        self._stop_base()

        self.get_logger().info('Navigating to the reachable placement pose...')
        self._navigate_to_waypoint('place_station')
        self._stop_base()
        self._set_state(State.REPORT_ARRIVAL)
        self.get_logger().info(f'Arrived at {label}.')

    # ------------------------------------------------------------------
    # Return to initial
    # ------------------------------------------------------------------

    def _return_to_initial(self) -> None:
        if self._initial_pose is None:
            raise RuntimeError('Initial pose was not captured; return cannot be verified')
        self._fold_before_driving()
        self._navigate_to_pose(
            self._pose_stamped(self._initial_pose), 'return_to_initial')

    def _stop_base(self) -> None:
        self.base_motion_active = False
        self.tracker_cmd_vel_pub.publish(Twist())

    def _creep_forward(self, distance: float = 0.03, speed: float = 0.04) -> None:
        duration = distance / max(0.01, speed)
        deadline = time.monotonic() + duration
        twist = Twist()
        twist.linear.x = float(speed)
        while rclpy.ok() and time.monotonic() < deadline:
            self.tracker_cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.tracker_cmd_vel_pub.publish(Twist())
        time.sleep(0.2)

    def _verify_and_align_heading(
        self, target_yaw: float = -1.570796, tolerance: float = 0.03, timeout: float = 12.0,
        physical: bool = False
    ) -> bool:
        """Closed-loop in-place rotation to achieve and verify target heading.

        Returns True if heading is verified within tolerance, False otherwise.
        Never treats a timeout or aborted rotation as successful.
        """
        self._stop_base()
        time.sleep(0.1)
        base = (self._physical_base_pose() if physical else self._current_base_pose())
        if base is None:
            self.get_logger().error('Heading alignment failed: localization unavailable.')
            return False

        err = _normalize_angle(target_yaw - base[2])
        if abs(err) <= tolerance:
            self.get_logger().info(
                f'Heading already aligned: yaw={base[2]:.3f} rad (err={err:.3f} rad <= {tolerance:.3f}).')
            return True

        self.get_logger().info(
            f'Aligning heading to {target_yaw:.3f} rad (current yaw={base[2]:.3f} rad, initial err={err:.3f} rad)...')
        deadline = time.monotonic() + timeout
        consecutive_aligned_count = 0

        while rclpy.ok() and time.monotonic() < deadline:
            base = (self._physical_base_pose() if physical else self._current_base_pose())
            if base is None:
                break
            if physical:
                self._docking_clearance(base)
            err = _normalize_angle(target_yaw - base[2])
            if abs(err) <= tolerance:
                consecutive_aligned_count += 1
                self.tracker_cmd_vel_pub.publish(Twist())
                if consecutive_aligned_count >= 3:
                    self.get_logger().info(
                        f'Heading alignment verified: yaw={base[2]:.3f} rad (err={err:.3f} rad <= {tolerance:.3f}).')
                    return True
                time.sleep(0.05)
                continue

            consecutive_aligned_count = 0
            sign = 1.0 if err > 0 else -1.0
            speed = max(0.06, min(0.30, abs(1.5 * err)))
            twist = Twist()
            twist.angular.z = sign * speed
            self.tracker_cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self._stop_base()
        time.sleep(0.1)
        final_base = (self._physical_base_pose() if physical else self._current_base_pose())
        final_err = abs(_normalize_angle(target_yaw - final_base[2])) if final_base else 999.0
        if final_err <= tolerance:
            self.get_logger().info(f'Heading alignment verified after settling: err={final_err:.3f} rad.')
            return True

        self.get_logger().error(
            f'Heading alignment timed out after {timeout:.1f}s: final err={final_err:.3f} rad > tolerance {tolerance:.3f}.')
        return False

    def _align_yaw(self, target_yaw: float = -1.570796, tolerance: float = 0.03, timeout: float = 12.0) -> bool:
        return self._verify_and_align_heading(target_yaw, tolerance, timeout)

    # ------------------------------------------------------------------
    # Startup readiness
    # ------------------------------------------------------------------

    def _wait_for_startup_readiness(self) -> None:
        self._set_state(State.SYSTEM_STARTUP)
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        deadline = time.monotonic() + timeout
        self.get_logger().info('Waiting for startup readiness...')
        while rclpy.ok() and time.monotonic() < deadline:
            if self._check_readiness():
                self.get_logger().info('Startup readiness satisfied.')
                return
            time.sleep(float(
                self.get_parameter('startup_poll_period_sec').value))
        raise RuntimeError('Startup readiness timed out.')

    def _check_readiness(self) -> bool:
        # Check lifecycle nodes
        for node_name in [
            '/map_server', '/amcl', '/controller_server',
            '/smoother_server', '/planner_server', '/behavior_server',
            '/bt_navigator', '/velocity_smoother',
        ]:
            if self.count_publishers(f'{node_name}/transition_event') == 0:
                return False
        # Check TF
        try:
            self.tf_buffer.lookup_transform(
                self.get_parameter('frame_id').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return False
        # Check services
        for client in [self.pick_client, self.place_client, self.fold_client]:
            if not client.wait_for_service(timeout_sec=0.05):
                return False
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AmmsTaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
