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
import threading
import time
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers geometry transforms
import tf2_ros
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
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
            'place_station', 'place_clear',
        ])
        self.waypoint_names = [str(n) for n in self.get_parameter('waypoint_names').value]
        for name in self.waypoint_names:
            self.declare_parameter(f'waypoint_{name}', [0.0, 0.0, 0.0])

        # Navigation
        self.declare_parameter('nav_timeout_sec', 90.0)
        self.declare_parameter('nav_retry_count', 3)
        self.declare_parameter('nav_retry_wait_sec', 0.5)
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
        self.declare_parameter('object_grasp_z_bias', 0.0)
        self.declare_parameter('object_grasp_surface_to_center_offset', 0.0)
        self.declare_parameter('object_nav_standoff_distance', 0.60)
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

        # Purple station visual alignment
        self.declare_parameter('purple_station_detection_enabled', True)
        self.declare_parameter('purple_station_rgb_topic',
                               '/camera/depth_camera/image')
        self.declare_parameter('purple_hsv_lower', [115, 50, 30])
        self.declare_parameter('purple_hsv_upper', [165, 255, 255])
        self.declare_parameter('purple_min_area_px', 200.0)
        self.declare_parameter('purple_detection_timeout_sec', 25.0)

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
        self.create_subscription(
            String, self.get_parameter('detections_topic').value,
            self._on_detections, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)

        # RGB image subscription for purple-station visual alignment
        self._bridge = CvBridge()
        self._purple_image: Optional[np.ndarray] = None
        self._purple_image_lock = threading.Lock()
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

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

    # ------------------------------------------------------------------
    # Command service callback
    # ------------------------------------------------------------------

    def _handle_goto(self, letter: str, request, response):
        if self._mission_busy:
            response.success = False
            response.message = f'Busy executing mission. Cannot accept {letter} now.'
            self.get_logger().warn(response.message)
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
        valid = [float(v) for v in msg.ranges
                 if math.isfinite(v) and msg.range_min <= float(v) <= msg.range_max]
        if valid:
            self.latest_obstacle_range = min(valid)

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

    def _navigate_to_waypoint(self, name: str) -> None:
        if not bool(self.get_parameter('use_navigation').value):
            return
        pose = self._pose_stamped(self._waypoint(name))
        self.get_logger().info(f'Nav → {name}: '
                               f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        self._navigate_to_pose(pose, name)

    def _navigate_to_pose(self, pose: PoseStamped, label: str) -> None:
        retry_count = max(1, int(self.get_parameter('nav_retry_count').value))
        retry_wait = max(0.0, float(self.get_parameter('nav_retry_wait_sec').value))
        timeout = float(self.get_parameter('nav_timeout_sec').value)
        for attempt in range(1, retry_count + 1):
            self.get_logger().info(f'Nav goal {label} attempt {attempt}/{retry_count}')
            try:
                self._navigate_to_pose_once(pose, label, timeout)
                self.get_logger().info(f'Nav goal {label} succeeded.')
                return
            except RuntimeError as exc:
                self.get_logger().warn(
                    f'Nav goal {label} attempt {attempt}/{retry_count} failed: {exc}')
                if attempt < retry_count:
                    self._clear_nav_costmaps(label)
                    time.sleep(retry_wait)
        raise RuntimeError(f'Nav goal {label} failed after {retry_count} attempts')

    def _navigate_to_pose_once(
        self, pose: PoseStamped, label: str, timeout: float,
    ) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('Nav2 NavigateToPose action server unavailable.')
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 10.0):
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
                time.sleep(0.1)
            goal_handle.cancel_goal_async()
            raise RuntimeError(f'Nav goal {label} timed out.')
        finally:
            self.base_motion_active = False

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
            self.get_logger().warn(f'BackUp action unavailable for {label}.')
            return
        goal = BackUp.Goal()
        goal.target.x = -distance
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = speed
        goal.time_allowance.sec = int(timeout)
        send_future = self.backup_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            return
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return
        self.base_motion_active = True
        try:
            result_future = goal_handle.get_result_async()
            self._wait_for_future(result_future, timeout + 2.0)
        finally:
            self.base_motion_active = False

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
            candidates = [c for c in candidates if c['object_id'] == object_id]
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

    def _acquire_reachable_object(
        self, seed: Optional[Dict] = None,
    ) -> Optional[Dict]:
        if seed is not None and self._candidate_is_valid(seed, True):
            return seed
        target = self._wait_for_valid_detection(True, timeout_override=2.0)
        if target is not None:
            return target
        return self._wait_for_valid_detection(False)

    def _candidate_is_valid(self, c: Dict, require_reachable: bool) -> bool:
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
        if require_reachable and not c.get('reachable', False):
            return False
        return True

    def _confirm_fresh_target(self, object_id: str) -> Optional[Dict]:
        return self._wait_for_valid_detection(
            True, object_id=object_id,
            min_detection_time=time.monotonic(),
            timeout_override=float(
                self.get_parameter('fresh_detection_after_move_timeout_sec').value))

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
            and self.ik.solve(pt) is not None)

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
            pt.header.stamp = rclpy.time.Time().to_msg()
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

    def _align_to_place_station(self) -> None:
        """No-op: robot already stopped at place_station waypoint."""
        return

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
            self.get_logger().warn('Could not capture initial pose.')

        # Fold arm
        self._try_call_trigger(self.fold_client, 'fold arm')

        # Navigate to pickup zone
        self._set_state(State.NAVIGATE_TO_PICKUP)
        self.get_logger().info(f'Navigating to {label}...')
        self._navigate_to_waypoint(waypoint)
        self._stop_base()

        # Search for object
        self._set_state(State.SEARCH_FOR_OBJECT)
        self._verify_localization()
        self.get_logger().info('Searching for object...')
        target = self._acquire_reachable_object()
        if target is None:
            self.get_logger().warn(f'No valid object detected at {label}.')
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
        if not self._candidate_is_valid(target, require_reachable=True):
            self.get_logger().warn('Object not reachable by arm.')
            self._record_object_failure(oid)
            self.get_logger().info('Returning to initial pose...')
            self._return_to_initial()
            return

        # Confirm fresh detection
        confirmed = self._confirm_fresh_target(oid)
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
        self.completed_object_ids.add(oid)
        self.get_logger().info('Pick successful.')

        # Retract and backup
        self._try_call_trigger(self.fold_client, 'fold arm')
        if bool(self.get_parameter('post_pick_backup_enabled').value):
            self._backup('post_pick_backup_distance', 'post_pick_backup_speed',
                         'post_pick_backup_timeout_sec', 'post-pick backup')

        # Navigate to place station
        self._set_state(State.NAVIGATE_TO_PLACE)
        self.get_logger().info('Navigating to Place Station...')
        self._navigate_to_waypoint('place_station')
        self._stop_base()

        # Visual confirmation: detect purple station (no base movement)
        self.get_logger().info('Detecting purple placing station...')
        self._align_to_purple_station()
        self._stop_base()

        # Place
        self._set_state(State.COMPUTE_PLACE_POSE)
        self._verify_localization()
        self.get_logger().info('Computing placement pose...')

        self._set_state(State.PLACE_OBJECT)
        self.get_logger().info('Placing...')
        try:
            self._call_trigger(self.place_client, 'place object')
        except RuntimeError as exc:
            self.get_logger().error(f'Place failed: {exc}')
            self._try_call_trigger(self.fold_client, 'recover arm')
            self.get_logger().info('Returning to initial pose...')
            self._return_to_initial()
            return

        # Verify release
        self._set_state(State.VERIFY_RELEASE)
        self.get_logger().info('Placement complete.')

        # Retract and backup
        self._try_call_trigger(self.fold_client, 'fold arm')
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
        self.get_logger().info(f'Navigating to {label}...')
        self._set_state(State.NAVIGATE_TO_PLACE)
        self._navigate_to_waypoint('place_station')
        self._stop_base()
        self._set_state(State.REPORT_ARRIVAL)
        self.get_logger().info(f'Arrived at {label}.')

    # ------------------------------------------------------------------
    # Return to initial
    # ------------------------------------------------------------------

    def _return_to_initial(self) -> None:
        if self._initial_pose is not None:
            try:
                self._navigate_to_pose(
                    self._pose_stamped(self._initial_pose),
                    'return_to_initial')
                return
            except RuntimeError as exc:
                self.get_logger().warn(f'Return to initial failed: {exc}')
        # Fallback to home waypoint
        try:
            self._navigate_to_waypoint('home')
        except RuntimeError:
            pass

    def _stop_base(self) -> None:
        self.base_motion_active = False

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
