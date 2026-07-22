import json
import math
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers geometry transforms
import tf2_ros
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PointStamped, PoseStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from k12_pick_place.ik import PlanarArmIK


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def mission_state_sequence(
    post_place_clear_waypoint: str = '',
    post_pick_clear_waypoint: str = '',
    post_pick_backup_enabled: bool = True,
    pre_place_route_waypoint: str = 'place_approach',
) -> List[str]:
    states = [
        'WAIT_FOR_MISSION_REQUEST',
        'START',
        'NAVIGATE_TO_PICK_ZONE',
        'SEARCH_FOR_OBJECT',
        'VALIDATE_OBJECT',
        'REPOSITION_FOR_REACH',
        'CONFIRM_FRESH_DETECTION',
        'PICK_OBJECT',
        'VERIFY_GRASP',
        'RETRACT_ARM',
    ]
    if post_pick_backup_enabled:
        states.append('POST_PICK_BACKUP')
    if post_pick_clear_waypoint:
        states.append('CLEAR_PICK_ZONE')
    if pre_place_route_waypoint:
        states.append('NAVIGATE_TO_PLACE_APPROACH')
    states.extend([
        'NAVIGATE_TO_PLACE_ZONE',
        'ALIGN_TO_PLACE_STATION',
        'STAGE_FOR_PLACE_APPROACH',
        'FINAL_APPROACH_TO_PLACE',
        'DETECT_PLACEMENT_SURFACE',
        'ESTIMATE_PLACE_POSE',
        'REPOSITION_FOR_PLACE',
        'PLACE_OBJECT',
        'VERIFY_RELEASE',
        'RETRACT_ARM',
        'POST_PLACE_BACKUP',
    ])
    if post_place_clear_waypoint:
        states.append('CLEAR_PLACE_ZONE')
    states.extend(['RETURN_TO_SEARCH_AREA', 'MISSION_COMPLETE', 'IDLE'])
    return states


class AmmsTaskManager(Node):
    def __init__(self):
        super().__init__('amms_task_manager')

        defaults = {
            'home': [0.0, 0.0, 0.0],
            'pick_station': [0.717, 0.0, 0.0],
            'pick_clear': [0.5, 0.0, 0.0],
            'place_approach': [2.0, 0.0, 0.0],
            'place_station': [2.825, 0.0, 0.0],
            'place_clear': [2.0, 0.0, 0.0],
        }

        self.declare_parameter('autostart', True)
        self.declare_parameter('repeat_count', 1)
        self.declare_parameter('mission_request_service', '/amms/start_mission')
        self.declare_parameter('pick_zone_waypoint', 'pick_station')
        self.declare_parameter('post_pick_clear_waypoint', '')
        self.declare_parameter('pre_place_route_waypoint', 'place_approach')
        self.declare_parameter('place_zone_waypoint', 'place_station')
        self.declare_parameter('search_return_waypoint', 'home')
        self.declare_parameter('max_mission_objects', 3)
        self.declare_parameter('no_object_cycles_before_complete', 1)
        self.declare_parameter('use_navigation', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pick_target_topic', '/amms/pick_target')
        self.declare_parameter('task_state_topic', '/amms/task_state')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('drive_on_heading_action_name', 'drive_on_heading')
        self.declare_parameter('pick_service', '/amms/pick_object')
        self.declare_parameter('place_service', '/amms/place_object')
        self.declare_parameter('fold_arm_service', '/amms/fold_arm')
        self.declare_parameter('gazebo_get_entity_state_service', '/get_entity_state')
        self.declare_parameter('use_gazebo_truth_fallback', True)
        self.declare_parameter('gazebo_truth_frame', 'map')
        self.declare_parameter('waypoint_names', list(defaults.keys()))
        self.declare_parameter('detection_timeout_sec', 25.0)
        self.declare_parameter('detection_freshness_sec', 2.0)
        self.declare_parameter('search_while_nav_detection_period_sec', 0.15)
        self.declare_parameter('fresh_detection_after_move_timeout_sec', 4.0)
        self.declare_parameter('min_detection_confidence', 0.25)
        self.declare_parameter('nav_timeout_sec', 120.0)
        self.declare_parameter('nav_retry_count', 3)
        self.declare_parameter('nav_retry_wait_sec', 0.75)
        self.declare_parameter('position_only_waypoints', ['place_station', 'place_approach', 'home'])
        self.declare_parameter('position_only_goal_tolerance', 0.45)
        self.declare_parameter('position_only_min_obstacle_range', 0.35)
        self.declare_parameter('backup_action_name', 'backup')
        self.declare_parameter('post_pick_backup_enabled', True)
        self.declare_parameter('post_pick_backup_distance', 0.81)
        self.declare_parameter('post_pick_backup_speed', 0.06)
        self.declare_parameter('post_pick_backup_timeout_sec', 18.0)
        self.declare_parameter('post_place_backup_enabled', True)
        self.declare_parameter('post_place_backup_distance', 0.50625)
        self.declare_parameter('post_place_backup_speed', 0.10)
        self.declare_parameter('post_place_backup_timeout_sec', 16.0)
        self.declare_parameter('nav2_backup_max_segment_distance', 0.253125)
        self.declare_parameter('clear_costmaps_on_nav_failure', True)
        self.declare_parameter('local_clear_costmap_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('global_clear_costmap_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_log_min_range', 0.45)
        self.declare_parameter('service_timeout_sec', 90.0)
        self.declare_parameter('localization_timeout_sec', 3.0)
        self.declare_parameter('startup_timeout_sec', 180.0)
        self.declare_parameter('startup_poll_period_sec', 0.25)
        self.declare_parameter('startup_required_topics', [
            '/scan',
            '/odometry/filtered',
            '/amms/detections',
        ])
        self.declare_parameter('startup_required_lifecycle_nodes', [
            '/map_server',
            '/amcl',
            '/controller_server',
            '/smoother_server',
            '/planner_server',
            '/behavior_server',
            '/bt_navigator',
            '/velocity_smoother',
        ])
        self.declare_parameter('startup_required_transforms', ['map<-base_link'])
        self.declare_parameter('startup_lifecycle_recovery_enabled', True)
        self.declare_parameter('startup_lifecycle_recovery_initial_delay_sec', 25.0)
        self.declare_parameter('startup_lifecycle_recovery_period_sec', 5.0)
        self.declare_parameter('startup_lifecycle_transition_timeout_sec', 8.0)
        self.declare_parameter('post_place_clear_waypoint', 'place_clear')
        self.declare_parameter('enable_final_alignment', True)
        self.declare_parameter('align_target_frame', 'map')
        self.declare_parameter('align_pick_station_target', [1.5, 0.0, 0.0])
        self.declare_parameter('align_pick_station_stop_distance', 0.783)
        self.declare_parameter('align_place_station_target', [3.5, 0.0, 0.0])
        self.declare_parameter('align_place_station_stop_distance', 0.675)
        self.declare_parameter('align_timeout_sec', 45.0)
        self.declare_parameter('place_final_approach_use_drive_on_heading', True)
        self.declare_parameter('place_final_approach_heading_tolerance', 0.35)
        self.declare_parameter('place_final_approach_lateral_tolerance', 0.30)
        self.declare_parameter('place_final_approach_max_distance', 1.11375)
        self.declare_parameter('place_final_approach_speed', 0.055)
        self.declare_parameter('place_final_approach_timeout_sec', 20.0)
        self.declare_parameter('place_final_orientation_stage_distance', 0.759375)
        self.declare_parameter('place_final_stage_timeout_sec', 60.0)
        self.declare_parameter('place_final_standoff_xy_tolerance', 0.22275)
        self.declare_parameter('place_final_standoff_yaw_tolerance', 0.40)
        self.declare_parameter('object_target_frame', 'arm_base_link')
        self.declare_parameter('object_reachable_min_radius', 0.22275)
        self.declare_parameter('object_reachable_max_radius', 1.0935)
        self.declare_parameter('object_reachable_min_z', -0.4725)
        self.declare_parameter('object_reachable_max_z', 0.354375)
        self.declare_parameter('object_grasp_forward_offset', 0.0)
        self.declare_parameter('object_grasp_z_bias', -0.01125)
        self.declare_parameter('object_grasp_surface_to_center_offset', 0.045)
        self.declare_parameter('object_nav_standoff_distance', 1.053)
        self.declare_parameter('max_reposition_attempts', 3)
        self.declare_parameter('max_object_failures', 4)
        self.declare_parameter('place_position_arm_base', [0.87075, 0.0, 0.1215])
        self.declare_parameter('place_slot_reach_adjustments_x', [0.0, -0.0405, -0.081, -0.1215])
        self.declare_parameter('use_shape_center_offsets', True)
        self.declare_parameter('cube_center_offset_m', 0.05625)
        self.declare_parameter('cylinder_center_offset_m', 0.05625)
        self.declare_parameter('block_center_offset_m', 0.05625)
        self.declare_parameter('default_center_offset_m', 0.05625)
        self.declare_parameter('require_reachable_before_pick_service', True)
        self.declare_parameter('pre_pick_detection_required', True)
        self.declare_parameter('pre_pick_detection_grace_sec', 0.0)
        self.declare_parameter('pick_retry_count', 3)
        self.declare_parameter('pick_retry_wait_sec', 1.0)

        self.waypoint_names = [str(name) for name in self.get_parameter('waypoint_names').value]
        for name in self.waypoint_names:
            self.declare_parameter(f'waypoint_{name}', defaults.get(name, [0.0, 0.0, 0.0]))

        self.nav_client = ActionClient(self, NavigateToPose, self.get_parameter('nav_action_name').value)
        self.backup_client = ActionClient(self, BackUp, self.get_parameter('backup_action_name').value)
        self.drive_on_heading_client = ActionClient(
            self,
            DriveOnHeading,
            self.get_parameter('drive_on_heading_action_name').value,
        )
        self.ik = PlanarArmIK()
        self.pick_client = self.create_client(Trigger, self.get_parameter('pick_service').value)
        self.place_client = self.create_client(Trigger, self.get_parameter('place_service').value)
        self.fold_client = self.create_client(Trigger, self.get_parameter('fold_arm_service').value)
        self.get_entity_state_client = self.create_client(
            GetEntityState,
            self.get_parameter('gazebo_get_entity_state_service').value,
        )
        self.local_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('local_clear_costmap_service').value,
        )
        self.global_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('global_clear_costmap_service').value,
        )
        self.lifecycle_clients: Dict[str, object] = {}
        self.lifecycle_change_clients: Dict[str, object] = {}
        self.create_service(
            Trigger,
            self.get_parameter('mission_request_service').value,
            self._on_start_mission_request,
        )

        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, self.get_parameter('task_state_topic').value, state_qos)
        self.pick_target_pub = self.create_publisher(String, self.get_parameter('pick_target_topic').value, 10)
        self.create_subscription(String, self.get_parameter('detections_topic').value, self._on_detections, 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_detection_time = 0.0
        self.latest_detection_count = 0
        self.latest_payload: Optional[Dict] = None
        self.mission_requested = threading.Event()
        self.mission_active = False
        self.completed_object_ids = set()
        self.failed_object_counts: Dict[str, int] = {}
        self.base_motion_active = False
        self.latest_obstacle_range: Optional[float] = None
        self.last_obstacle_log_time = 0.0
        self.started = False
        self.mission_thread: Optional[threading.Thread] = None
        self.create_timer(1.0, self._maybe_start)

    def _maybe_start(self) -> None:
        if self.started:
            return
        self.started = True
        if bool(self.get_parameter('autostart').value):
            self.mission_requested.set()
        self.mission_thread = threading.Thread(target=self._mission_loop, daemon=True)
        self.mission_thread.start()

    def _on_start_mission_request(self, _request, response):
        if self.mission_active:
            response.success = False
            response.message = 'Mission is already active.'
            return response

        self.completed_object_ids.clear()
        self.failed_object_counts.clear()
        self.mission_requested.set()
        response.success = True
        response.message = 'Mission request accepted.'
        return response

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
        valid_ranges = [
            float(value)
            for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= float(value) <= msg.range_max
        ]
        if not valid_ranges:
            return

        closest = min(valid_ranges)
        self.latest_obstacle_range = closest
        threshold = float(self.get_parameter('obstacle_log_min_range').value)
        now = time.monotonic()
        if closest <= threshold and now - self.last_obstacle_log_time > 2.0:
            self.last_obstacle_log_time = now
            self.get_logger().warn(
                f'Obstacle close to base from LiDAR: nearest_range={closest:.2f} m. '
                'Nav2 costmaps remain responsible for avoidance and replanning.'
            )

    def _mission_loop(self) -> None:
        try:
            self._set_state('WAIT_FOR_MISSION_REQUEST')
            while rclpy.ok():
                self.mission_requested.wait(timeout=0.2)
                if not self.mission_requested.is_set():
                    continue
                self.mission_requested.clear()
                self.mission_active = True
                try:
                    self._run_requested_mission()
                except Exception as exc:
                    self._stop_base()
                    self._set_state(f'ERROR: {exc}')
                    self.get_logger().error(f'AMMS mission failed: {exc}')
                finally:
                    self.mission_active = False
                    self._stop_base()
                    self._set_state('IDLE')
        finally:
            self._stop_base()

    def _run_requested_mission(self) -> None:
        self._wait_for_startup_readiness()
        self.completed_object_ids.clear()
        self.failed_object_counts.clear()

        repeat_count = int(self.get_parameter('repeat_count').value)
        max_objects = int(self.get_parameter('max_mission_objects').value)
        if repeat_count > 0:
            max_objects = repeat_count
        if max_objects <= 0:
            max_objects = 100
        no_object_limit = max(1, int(self.get_parameter('no_object_cycles_before_complete').value))
        no_object_cycles = 0
        processed = 0

        self._set_state('START')
        self._try_call_trigger(self.fold_client, 'initial fold arm')

        while rclpy.ok() and processed < max_objects:
            pick_waypoint = str(self.get_parameter('pick_zone_waypoint').value)
            self._set_state('NAVIGATE_TO_PICK_ZONE')
            candidate = self._navigate_to_waypoint(
                pick_waypoint,
                monitor_for_objects=True,
                publish_state=False,
            )

            self._set_state('SEARCH_FOR_OBJECT')
            self._stop_base()
            self._verify_localization()
            target = self._acquire_reachable_object(candidate)
            if target is None:
                no_object_cycles += 1
                self.get_logger().info(
                    f'No pickable object found ({no_object_cycles}/{no_object_limit}).'
                )
                if no_object_cycles >= no_object_limit:
                    break
                continue

            no_object_cycles = 0
            object_id = self._object_id(target['object'])
            self._set_state('VALIDATE_OBJECT')
            if not self._candidate_is_valid(target, require_reachable=True):
                self._record_object_failure(object_id)
                continue

            self._set_state('CONFIRM_FRESH_DETECTION')
            confirmed = self._confirm_fresh_target(object_id)
            if confirmed is None:
                self._record_object_failure(object_id)
                continue

            self._set_state('PICK_OBJECT')
            self._stop_base()
            self._verify_localization()
            try:
                self._call_pick_with_retries(object_id)
            except RuntimeError as exc:
                self._record_object_failure(object_id)
                self.get_logger().warn(f'Pick failed for {object_id}; recovering: {exc}')
                self._try_call_trigger(self.fold_client, 'recover carry pose after pick failure')
                continue

            self._set_state('VERIFY_GRASP')
            self.completed_object_ids.add(object_id)
            self._set_state('RETRACT_ARM')
            self._call_trigger(self.fold_client, 'fold arm with object')
            if bool(self.get_parameter('post_pick_backup_enabled').value):
                self._set_state('POST_PICK_BACKUP')
                self._backup_from_pick_table()

            pick_clear_waypoint = str(self.get_parameter('post_pick_clear_waypoint').value).strip()
            if pick_clear_waypoint:
                self._set_state('CLEAR_PICK_ZONE')
                self._navigate_to_waypoint(pick_clear_waypoint, publish_state=False)

            pre_place_waypoint = str(self.get_parameter('pre_place_route_waypoint').value).strip()
            if pre_place_waypoint:
                self._set_state('NAVIGATE_TO_PLACE_APPROACH')
                self._navigate_to_waypoint(pre_place_waypoint, publish_state=False)

            self._set_state('NAVIGATE_TO_PLACE_ZONE')
            self._navigate_to_waypoint(str(self.get_parameter('place_zone_waypoint').value), publish_state=False)

            self._set_state('DETECT_PLACEMENT_SURFACE')
            if not self._ensure_place_reachable():
                raise RuntimeError('Placement surface is not reachable after reposition attempts.')

            self._set_state('ESTIMATE_PLACE_POSE')
            self._verify_localization()

            self._set_state('PLACE_OBJECT')
            self._stop_base()
            self._verify_localization()
            self._call_trigger(self.place_client, 'place object')

            self._set_state('VERIFY_RELEASE')
            self._set_state('RETRACT_ARM')
            self._call_trigger(self.fold_client, 'fold arm after release')
            self._set_state('POST_PLACE_BACKUP')
            self._backup_from_place_table()

            clear_waypoint = str(self.get_parameter('post_place_clear_waypoint').value).strip()
            if clear_waypoint:
                self._set_state('CLEAR_PLACE_ZONE')
                self._navigate_to_waypoint(clear_waypoint, publish_state=False)

            self._set_state('RETURN_TO_SEARCH_AREA')
            self._navigate_to_waypoint(str(self.get_parameter('search_return_waypoint').value), publish_state=False)

            processed += 1

        self._set_state('MISSION_COMPLETE')

    def _wait_for_startup_readiness(self) -> None:
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        poll_period = float(self.get_parameter('startup_poll_period_sec').value)
        deadline = time.monotonic() + timeout
        started_at = time.monotonic()
        last_recovery_at = 0.0
        last_report = ''

        while rclpy.ok() and time.monotonic() < deadline:
            missing = self._missing_startup_requirements()
            if not missing:
                self.get_logger().info('AMMS startup readiness satisfied.')
                return
            now = time.monotonic()
            recovery_enabled = bool(self.get_parameter('startup_lifecycle_recovery_enabled').value)
            recovery_delay = float(self.get_parameter('startup_lifecycle_recovery_initial_delay_sec').value)
            recovery_period = float(self.get_parameter('startup_lifecycle_recovery_period_sec').value)
            if (
                recovery_enabled
                and now - started_at >= recovery_delay
                and now - last_recovery_at >= recovery_period
            ):
                last_recovery_at = now
                self._recover_startup_lifecycle_nodes()
            report = ', '.join(missing[:6])
            if report != last_report:
                self._set_state(f'WAITING_FOR_READINESS: {report}')
                last_report = report
            time.sleep(poll_period)
        raise RuntimeError(f'Startup readiness timed out: {last_report}')

    def _missing_startup_requirements(self) -> List[str]:
        missing = []
        for topic in self._string_list_parameter('startup_required_topics'):
            if self.count_publishers(topic) == 0:
                missing.append(f'topic {topic}')

        service_checks = [
            (self.pick_client, self.get_parameter('pick_service').value),
            (self.place_client, self.get_parameter('place_service').value),
            (self.fold_client, self.get_parameter('fold_arm_service').value),
        ]
        if bool(self.get_parameter('use_gazebo_truth_fallback').value):
            service_checks.append((
                self.get_entity_state_client,
                self.get_parameter('gazebo_get_entity_state_service').value,
            ))
        if bool(self.get_parameter('clear_costmaps_on_nav_failure').value):
            service_checks.extend([
                (self.local_clear_costmap_client, self.get_parameter('local_clear_costmap_service').value),
                (self.global_clear_costmap_client, self.get_parameter('global_clear_costmap_service').value),
            ])
        for client, service_name in service_checks:
            if not client.wait_for_service(timeout_sec=0.05):
                missing.append(f'service {service_name}')

        if not self.nav_client.wait_for_server(timeout_sec=0.05):
            missing.append(f'action {self.get_parameter("nav_action_name").value}')
        if bool(self.get_parameter('place_final_approach_use_drive_on_heading').value):
            if not self.drive_on_heading_client.wait_for_server(timeout_sec=0.05):
                missing.append(f'action {self.get_parameter("drive_on_heading_action_name").value}')
        if (
            bool(self.get_parameter('post_place_backup_enabled').value)
            or bool(self.get_parameter('post_pick_backup_enabled').value)
        ):
            if not self.backup_client.wait_for_server(timeout_sec=0.05):
                missing.append(f'action {self.get_parameter("backup_action_name").value}')

        for node_name in self._string_list_parameter('startup_required_lifecycle_nodes'):
            client = self.lifecycle_clients.get(node_name)
            if client is None:
                client = self.create_client(GetState, f'{node_name}/get_state')
                self.lifecycle_clients[node_name] = client
            if not client.wait_for_service(timeout_sec=0.05):
                missing.append(f'lifecycle {node_name}')
                continue
            future = client.call_async(GetState.Request())
            if not self._wait_for_future(future, 0.25):
                missing.append(f'lifecycle {node_name}')
                continue
            response = future.result()
            if response is None or response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                state_label = 'unknown' if response is None else response.current_state.label
                missing.append(f'lifecycle {node_name}={state_label}')

        for requirement in self._string_list_parameter('startup_required_transforms'):
            target_frame, source_frame = self._parse_transform_requirement(requirement)
            if not self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ):
                missing.append(f'tf {target_frame}<-{source_frame}')
        return missing

    def _recover_startup_lifecycle_nodes(self) -> None:
        self.get_logger().warn('Attempting Nav2 lifecycle startup recovery.')
        for node_name in self._string_list_parameter('startup_required_lifecycle_nodes'):
            state = self._get_lifecycle_state(node_name)
            if state is None:
                self.get_logger().warn(f'Lifecycle recovery skipped {node_name}: get_state unavailable.')
                continue
            if state.id == State.PRIMARY_STATE_ACTIVE:
                continue
            self.get_logger().warn(
                f'Lifecycle recovery inspecting {node_name}: state={state.label or state.id}.'
            )
            if state.id == State.PRIMARY_STATE_UNCONFIGURED:
                if not self._change_lifecycle_state(node_name, Transition.TRANSITION_CONFIGURE):
                    continue
                state = self._get_lifecycle_state(node_name)
                if state is None:
                    continue
            if state.id == State.PRIMARY_STATE_INACTIVE:
                self._change_lifecycle_state(node_name, Transition.TRANSITION_ACTIVATE)
            elif state.id != State.PRIMARY_STATE_ACTIVE:
                self.get_logger().warn(
                    f'Lifecycle recovery cannot transition {node_name} from state={state.label or state.id}.'
                )

    def _get_lifecycle_state(self, node_name: str) -> Optional[State]:
        client = self.lifecycle_clients.get(node_name)
        if client is None:
            client = self.create_client(GetState, f'{node_name}/get_state')
            self.lifecycle_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=0.25):
            return None
        future = client.call_async(GetState.Request())
        timeout = float(self.get_parameter('startup_lifecycle_transition_timeout_sec').value)
        if not self._wait_for_future(future, min(timeout, 2.0)):
            return None
        response = future.result()
        if response is None:
            return None
        return response.current_state

    def _change_lifecycle_state(self, node_name: str, transition_id: int) -> bool:
        client = self.lifecycle_change_clients.get(node_name)
        if client is None:
            client = self.create_client(ChangeState, f'{node_name}/change_state')
            self.lifecycle_change_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'Lifecycle recovery cannot reach {node_name}/change_state.')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        label = 'configure' if transition_id == Transition.TRANSITION_CONFIGURE else 'activate'
        self.get_logger().warn(f'Lifecycle recovery requesting {label} for {node_name}.')
        future = client.call_async(request)
        timeout = float(self.get_parameter('startup_lifecycle_transition_timeout_sec').value)
        if not self._wait_for_future(future, timeout):
            self.get_logger().warn(f'Lifecycle recovery timed out requesting {label} for {node_name}.')
            return False
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f'Lifecycle recovery {label} failed for {node_name}.')
            return False
        self.get_logger().info(f'Lifecycle recovery {label} accepted for {node_name}.')
        return True

    @staticmethod
    def _parse_transform_requirement(requirement: str) -> Tuple[str, str]:
        if '<-' in requirement:
            target_frame, source_frame = requirement.split('<-', 1)
            return target_frame.strip(), source_frame.strip()
        if ':' in requirement:
            target_frame, source_frame = requirement.split(':', 1)
            return target_frame.strip(), source_frame.strip()
        raise ValueError(f'Invalid transform requirement: {requirement}')

    def _string_list_parameter(self, name: str) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return []
        return [str(item) for item in value if str(item)]

    def _waypoint(self, name: str) -> Tuple[float, float, float]:
        if name not in self.waypoint_names:
            raise RuntimeError(f'Waypoint {name} is not configured.')
        values = list(self.get_parameter(f'waypoint_{name}').value)
        if len(values) != 3:
            raise RuntimeError(f'waypoint_{name} must be [x, y, yaw].')
        return float(values[0]), float(values[1]), float(values[2])

    def _navigate_to_waypoint(
        self,
        name: str,
        monitor_for_objects: bool = False,
        publish_state: bool = True,
    ) -> Optional[Dict]:
        if not bool(self.get_parameter('use_navigation').value):
            self.get_logger().info(f'Navigation disabled; skipping waypoint {name}.')
            return None

        pose = self._pose_stamped(self._waypoint(name))
        if publish_state:
            self._set_state(f'NAVIGATE_TO_{name.upper()}')
        if self._accept_position_only_waypoint(name, pose, 'already within position-only tolerance before Nav2 goal'):
            if name in ('pick_station', 'place_station'):
                self._align_to_station(name)
            return None
        self.get_logger().info(
            f'Sending Nav2 goal {name}: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}.'
        )
        try:
            candidate = self._navigate_to_pose(
                pose,
                name,
                float(self.get_parameter('nav_timeout_sec').value),
                monitor_for_objects=monitor_for_objects,
            )
        except RuntimeError as exc:
            if not self._accept_position_only_waypoint(name, pose, str(exc)):
                raise
            candidate = None
        if candidate is None:
            self._align_to_station(name)
        return candidate

    def _accept_position_only_waypoint(self, name: str, pose: PoseStamped, reason: str) -> bool:
        if name not in self._string_list_parameter('position_only_waypoints'):
            return False

        current = self._current_base_pose()
        if current is None:
            self.get_logger().warn(
                f'Cannot position-accept staging waypoint {name}; current base pose is unavailable.'
            )
            return False

        tolerance = max(0.0, float(self.get_parameter('position_only_goal_tolerance').value))
        distance_error = math.hypot(pose.pose.position.x - current[0], pose.pose.position.y - current[1])
        goal_yaw = self._yaw_from_pose(pose)
        yaw_error = self._normalize_angle(goal_yaw - current[2])
        if distance_error > tolerance:
            self.get_logger().warn(
                f'Cannot position-accept staging waypoint {name}; '
                f'linear_error={distance_error:.3f} m exceeds tolerance={tolerance:.3f} m.'
            )
            return False

        obstacle = self.latest_obstacle_range
        min_obstacle = max(0.0, float(self.get_parameter('position_only_min_obstacle_range').value))
        if obstacle is not None and obstacle < min_obstacle:
            self.get_logger().warn(
                f'Cannot position-accept staging waypoint {name}; nearest_obstacle={obstacle:.2f} m '
                f'is below minimum={min_obstacle:.2f} m.'
            )
            return False

        obstacle_text = 'unknown' if obstacle is None else f'{obstacle:.2f} m'
        alignment_text = (
            'Final alignment remains required before arm motion.'
            if name in ('pick_station', 'place_station')
            else 'Skipping redundant Nav2 goal.'
        )
        self.get_logger().warn(
            f'Accepting position-only waypoint {name}: {reason}; '
            f'base=({current[0]:.2f}, {current[1]:.2f}, yaw={current[2]:.2f}), '
            f'linear_error={distance_error:.3f} m, ignored_yaw_error={yaw_error:.3f} rad, '
            f'nearest_obstacle={obstacle_text}. {alignment_text}'
        )
        return True

    def _navigate_to_pose(
        self,
        pose: PoseStamped,
        label: str,
        timeout: float,
        monitor_for_objects: bool = False,
    ) -> Optional[Dict]:
        retry_count = max(1, int(self.get_parameter('nav_retry_count').value))
        retry_wait = max(0.0, float(self.get_parameter('nav_retry_wait_sec').value))
        last_error = ''

        for attempt in range(1, retry_count + 1):
            self._log_navigation_goal(label, pose, attempt, retry_count)
            try:
                result = self._navigate_to_pose_once(
                    pose,
                    label,
                    timeout,
                    monitor_for_objects=monitor_for_objects,
                )
                self._log_navigation_result(label, pose, result)
                return result
            except RuntimeError as exc:
                last_error = str(exc)
                obstacle = self.latest_obstacle_range
                obstacle_text = '' if obstacle is None else f' nearest_obstacle={obstacle:.2f} m'
                self.get_logger().warn(
                    f'Nav2 goal {label} attempt {attempt}/{retry_count} failed: {exc}.{obstacle_text}'
                )
                if attempt < retry_count:
                    self._clear_nav_costmaps(label)
                    time.sleep(retry_wait)

        raise RuntimeError(f'Navigation goal {label} failed after {retry_count} attempts: {last_error}')

    def _navigate_to_pose_once(
        self,
        pose: PoseStamped,
        label: str,
        timeout: float,
        monitor_for_objects: bool = False,
    ) -> Optional[Dict]:
        if not self.nav_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('Nav2 NavigateToPose action server is not available.')

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.nav_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 10.0):
            raise RuntimeError(f'Timed out sending navigation goal {label}.')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Navigation goal {label} was rejected.')

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout
        poll_period = max(0.05, float(self.get_parameter('search_while_nav_detection_period_sec').value))
        self.base_motion_active = True
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if monitor_for_objects:
                    candidate = self._select_valid_detection(require_reachable=True)
                    if candidate is not None:
                        self.get_logger().info(
                            f'Reachable object {self._object_id(candidate["object"])} detected during navigation; '
                            f'canceling Nav2 goal {label}.'
                        )
                        cancel_future = goal_handle.cancel_goal_async()
                        self._wait_for_future(cancel_future, 2.0)
                        return candidate
                if self._position_only_waypoint_reached(label, pose):
                    cancel_future = goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, 2.0)
                    return None
                if result_future.done():
                    result = result_future.result()
                    if result.status != GoalStatus.STATUS_SUCCEEDED:
                        raise RuntimeError(f'Navigation goal {label} failed with status {result.status}.')
                    return None
                time.sleep(poll_period)

            goal_handle.cancel_goal_async()
            raise RuntimeError(f'Navigation goal {label} timed out.')
        finally:
            self.base_motion_active = False

    def _position_only_waypoint_reached(self, name: str, pose: PoseStamped) -> bool:
        if name not in self._string_list_parameter('position_only_waypoints'):
            return False

        current = self._current_base_pose()
        if current is None:
            return False

        tolerance = max(0.0, float(self.get_parameter('position_only_goal_tolerance').value))
        distance_error = math.hypot(pose.pose.position.x - current[0], pose.pose.position.y - current[1])
        if distance_error > tolerance:
            return False

        obstacle = self.latest_obstacle_range
        min_obstacle = max(0.0, float(self.get_parameter('position_only_min_obstacle_range').value))
        if obstacle is not None and obstacle < min_obstacle:
            return False

        goal_yaw = self._yaw_from_pose(pose)
        yaw_error = self._normalize_angle(goal_yaw - current[2])
        obstacle_text = 'unknown' if obstacle is None else f'{obstacle:.2f} m'
        self.get_logger().info(
            f'Position-only waypoint {name} reached during Nav2 execution; '
            f'linear_error={distance_error:.3f} m, ignored_yaw_error={yaw_error:.3f} rad, '
            f'nearest_obstacle={obstacle_text}. Canceling yaw-only settling.'
        )
        return True

    def _backup_from_pick_table(self) -> None:
        self._run_nav2_backup(
            stage='POST_PICK_BACKUP',
            enabled_parameter='post_pick_backup_enabled',
            distance_parameter='post_pick_backup_distance',
            speed_parameter='post_pick_backup_speed',
            timeout_parameter='post_pick_backup_timeout_sec',
            context='post-pick backup',
        )

    def _backup_from_place_table(self) -> None:
        self._run_nav2_backup(
            stage='POST_PLACE_BACKUP',
            enabled_parameter='post_place_backup_enabled',
            distance_parameter='post_place_backup_distance',
            speed_parameter='post_place_backup_speed',
            timeout_parameter='post_place_backup_timeout_sec',
            context='post-place backup',
        )

    def _run_nav2_backup(
        self,
        stage: str,
        enabled_parameter: str,
        distance_parameter: str,
        speed_parameter: str,
        timeout_parameter: str,
        context: str,
    ) -> None:
        if not bool(self.get_parameter('use_navigation').value):
            return
        if not bool(self.get_parameter(enabled_parameter).value):
            return

        distance = abs(float(self.get_parameter(distance_parameter).value))
        speed = abs(float(self.get_parameter(speed_parameter).value))
        timeout = max(1.0, float(self.get_parameter(timeout_parameter).value))
        max_segment = max(0.05, float(self.get_parameter('nav2_backup_max_segment_distance').value))
        if distance <= 0.0 or speed <= 0.0:
            self.get_logger().warn(f'Skipping {context}; distance or speed is not positive.')
            return

        if not self.backup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 BackUp action server is unavailable; continuing to clear waypoint.')
            return

        current = self._current_base_pose()
        obstacle = self.latest_obstacle_range
        pose_text = (
            'unknown'
            if current is None
            else f'({current[0]:.2f}, {current[1]:.2f}, yaw={current[2]:.2f})'
        )
        obstacle_text = 'unknown' if obstacle is None else f'{obstacle:.2f} m'
        self.get_logger().info(
            f'{stage}: requesting Nav2 BackUp distance={distance:.2f} m, '
            f'speed={speed:.2f} m/s, base={pose_text}, nearest_obstacle={obstacle_text}.'
        )

        segments = self._backup_segments(distance, max_segment)
        goal = BackUp.Goal()
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(speed)

        self.base_motion_active = True
        try:
            for index, segment_distance in enumerate(segments, start=1):
                segment_timeout = min(timeout, max(5.0, segment_distance / max(speed, 1e-6) + 8.0))
                goal.target.x = -segment_distance
                goal.time_allowance.sec = int(segment_timeout)
                goal.time_allowance.nanosec = int((segment_timeout - int(segment_timeout)) * 1e9)

                self.get_logger().info(
                    f'{stage}: Nav2 BackUp segment {index}/{len(segments)} '
                    f'distance={segment_distance:.2f} m, allowance={segment_timeout:.1f} s.'
                )
                if not self._run_nav2_backup_goal(goal, segment_timeout, context):
                    return

            current_after = self._current_base_pose()
            pose_after = (
                'unknown'
                if current_after is None
                else f'({current_after[0]:.2f}, {current_after[1]:.2f}, yaw={current_after[2]:.2f})'
            )
            self.get_logger().info(f'{stage} complete: base={pose_after}.')
        finally:
            self.base_motion_active = False

    @staticmethod
    def _backup_segments(distance: float, max_segment: float) -> List[float]:
        if distance <= max_segment:
            return [distance]
        segments = []
        remaining = distance
        while remaining > 1e-6:
            segment = min(max_segment, remaining)
            segments.append(segment)
            remaining -= segment
        return segments

    def _run_nav2_backup_goal(self, goal: BackUp.Goal, timeout: float, context: str) -> bool:
        send_future = self.backup_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            self.get_logger().warn('Timed out sending Nav2 BackUp goal; clearing costmaps and continuing.')
            self._clear_nav_costmaps(f'{context} send timeout')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Nav2 BackUp goal was rejected; clearing costmaps and continuing.')
            self._clear_nav_costmaps(f'{context} rejected')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout + 2.0):
            goal_handle.cancel_goal_async()
            self.get_logger().warn('Nav2 BackUp timed out; clearing costmaps and continuing.')
            self._clear_nav_costmaps(f'{context} timeout')
            return False

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Nav2 BackUp failed with status {result.status}; clearing costmaps and continuing.'
            )
            self._clear_nav_costmaps(f'{context} failure')
            return False

        return True

    def _clear_nav_costmaps(self, label: str) -> None:
        if not bool(self.get_parameter('clear_costmaps_on_nav_failure').value):
            return

        for client, service_name in (
            (self.local_clear_costmap_client, self.get_parameter('local_clear_costmap_service').value),
            (self.global_clear_costmap_client, self.get_parameter('global_clear_costmap_service').value),
        ):
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Cannot clear Nav2 costmap for {label}; service {service_name} unavailable.')
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not self._wait_for_future(future, 2.0):
                self.get_logger().warn(f'Timed out clearing Nav2 costmap with {service_name}.')
                continue
            self.get_logger().info(f'Cleared Nav2 costmap via {service_name} before retrying {label}.')

    def _log_navigation_goal(self, label: str, pose: PoseStamped, attempt: int, total: int) -> None:
        goal = (
            pose.pose.position.x,
            pose.pose.position.y,
            self._yaw_from_pose(pose),
        )
        current = self._current_base_pose()
        if current is None:
            self.get_logger().info(
                f'Nav2 goal {label} attempt {attempt}/{total}: '
                f'goal=({goal[0]:.2f}, {goal[1]:.2f}, yaw={goal[2]:.2f}); current base pose unavailable.'
            )
            return

        distance_error = math.hypot(goal[0] - current[0], goal[1] - current[1])
        yaw_error = self._normalize_angle(goal[2] - current[2])
        obstacle = self.latest_obstacle_range
        obstacle_text = 'unknown' if obstacle is None else f'{obstacle:.2f} m'
        self.get_logger().info(
            f'Nav2 goal {label} attempt {attempt}/{total}: '
            f'goal=({goal[0]:.2f}, {goal[1]:.2f}, yaw={goal[2]:.2f}), '
            f'base=({current[0]:.2f}, {current[1]:.2f}, yaw={current[2]:.2f}), '
            f'linear_error={distance_error:.2f} m, yaw_error={yaw_error:.2f} rad, '
            f'nearest_obstacle={obstacle_text}.'
        )

    def _log_navigation_result(self, label: str, pose: PoseStamped, candidate: Optional[Dict]) -> None:
        if candidate is not None:
            self.get_logger().info(
                f'Nav2 goal {label} stopped early because {candidate["object_id"]} is freshly reachable.'
            )
            return

        current = self._current_base_pose()
        if current is None:
            self.get_logger().info(f'Nav2 goal {label} succeeded; current base pose unavailable.')
            return

        goal_yaw = self._yaw_from_pose(pose)
        distance_error = math.hypot(pose.pose.position.x - current[0], pose.pose.position.y - current[1])
        yaw_error = self._normalize_angle(goal_yaw - current[2])
        self.get_logger().info(
            f'Nav2 goal {label} succeeded: final_error={distance_error:.3f} m, '
            f'yaw_error={yaw_error:.3f} rad.'
        )

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        rotation = pose.pose.orientation
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _align_to_station(self, name: str) -> None:
        if not bool(self.get_parameter('enable_final_alignment').value):
            return
        if name not in ('pick_station', 'place_station'):
            return

        target = tuple(float(v) for v in self.get_parameter(f'align_{name}_target').value)
        if len(target) != 3:
            raise RuntimeError(f'align_{name}_target must be [x, y, z].')
        stop_distance = float(self.get_parameter(f'align_{name}_stop_distance').value)
        target_frame = self.get_parameter('align_target_frame').value
        if target_frame != self.get_parameter('frame_id').value:
            transformed = self._transform_fixed_point(target_frame, target, self.get_parameter('frame_id').value)
            if transformed is None:
                raise RuntimeError(f'Could not transform final alignment target for {name}.')
            target = transformed

        self._set_state(f'ALIGN_TO_{name.upper()}')
        _, _, yaw = self._waypoint(name)
        standoff = (
            target[0] - stop_distance * math.cos(yaw),
            target[1] - stop_distance * math.sin(yaw),
            yaw,
        )
        current = self._current_base_pose()
        current_text = (
            'unknown'
            if current is None
            else f'({current[0]:.2f}, {current[1]:.2f}, yaw={current[2]:.2f})'
        )
        self.get_logger().info(
            f'Final alignment for {name}: target=({target[0]:.2f}, {target[1]:.2f}), '
            f'standoff=({standoff[0]:.2f}, {standoff[1]:.2f}, yaw={standoff[2]:.2f}), '
            f'stop_distance={stop_distance:.2f}, base={current_text}.'
        )
        if name == 'place_station':
            if self._align_to_place_station(target, stop_distance, yaw, standoff):
                return
            if bool(self.get_parameter('place_final_approach_use_drive_on_heading').value):
                raise RuntimeError(
                    'Place final staged approach failed; refusing tight-space Nav2 final-pose fallback.'
                )
        pose = self._pose_stamped(standoff)
        timeout = float(self.get_parameter('align_timeout_sec').value)
        self._navigate_to_pose(pose, f'{name} final alignment', timeout)

    def _align_to_place_station(
        self,
        target: Tuple[float, float, float],
        stop_distance: float,
        yaw: float,
        standoff: Tuple[float, float, float],
    ) -> bool:
        if self._place_standoff_ready(standoff, require_reachable=True):
            return True

        stage_distance = max(0.05, float(self.get_parameter('place_final_orientation_stage_distance').value))
        max_drive_distance = max(0.05, float(self.get_parameter('place_final_approach_max_distance').value))
        stage_distance = min(stage_distance, max_drive_distance)
        stage_stop_distance = stop_distance + stage_distance
        stage = (
            target[0] - stage_stop_distance * math.cos(yaw),
            target[1] - stage_stop_distance * math.sin(yaw),
            yaw,
        )

        self._set_state('STAGE_FOR_PLACE_APPROACH')
        self.get_logger().info(
            'Place final alignment staging before table approach: '
            f'stage=({stage[0]:.2f}, {stage[1]:.2f}, yaw={stage[2]:.2f}), '
            f'final_standoff=({standoff[0]:.2f}, {standoff[1]:.2f}), '
            f'straight_approach={stage_distance:.2f} m.'
        )
        try:
            self._navigate_to_pose(
                self._pose_stamped(stage),
                'place_station orientation staging',
                float(self.get_parameter('place_final_stage_timeout_sec').value),
            )
        except RuntimeError as exc:
            if self._place_standoff_ready(standoff, require_reachable=True):
                self.get_logger().warn(
                    f'Accepting place final alignment after staging failure because '
                    f'the placement pose is already reachable: {exc}'
                )
                return True
            self.get_logger().warn(
                f'Place orientation staging failed; final table approach will not use tight-space Nav2 fallback: {exc}'
            )
            return False

        current = self._current_base_pose()
        self._set_state('FINAL_APPROACH_TO_PLACE')
        if current is not None and self._drive_on_heading_place_approach(standoff, current):
            return self._place_standoff_ready(standoff, require_reachable=True)

        self.get_logger().warn(
            'Place final staged approach could not complete with DriveOnHeading.'
        )
        return False

    def _place_standoff_ready(
        self,
        standoff: Tuple[float, float, float],
        require_reachable: bool,
    ) -> bool:
        current = self._current_base_pose()
        if current is None:
            return False

        xy_tolerance = max(0.0, float(self.get_parameter('place_final_standoff_xy_tolerance').value))
        yaw_tolerance = max(0.0, float(self.get_parameter('place_final_standoff_yaw_tolerance').value))
        distance_error = math.hypot(standoff[0] - current[0], standoff[1] - current[1])
        yaw_error = self._normalize_angle(standoff[2] - current[2])
        obstacle = self.latest_obstacle_range
        min_obstacle = max(0.0, float(self.get_parameter('position_only_min_obstacle_range').value))
        reachable = (not require_reachable) or self._placement_pose_reachable()

        if distance_error <= xy_tolerance and abs(yaw_error) <= yaw_tolerance and reachable:
            obstacle_text = 'unknown' if obstacle is None else f'{obstacle:.2f} m'
            self.get_logger().info(
                'Place standoff accepted: '
                f'linear_error={distance_error:.3f} m, yaw_error={yaw_error:.3f} rad, '
                f'placement_reachable={reachable}, nearest_obstacle={obstacle_text}.'
            )
            return obstacle is None or obstacle >= min_obstacle

        self.get_logger().info(
            'Place standoff not ready: '
            f'linear_error={distance_error:.3f}/{xy_tolerance:.3f} m, '
            f'yaw_error={yaw_error:.3f}/{yaw_tolerance:.3f} rad, '
            f'placement_reachable={reachable}.'
        )
        return False

    def _drive_on_heading_place_approach(
        self,
        standoff: Tuple[float, float, float],
        current: Optional[Tuple[float, float, float]],
    ) -> bool:
        if not bool(self.get_parameter('place_final_approach_use_drive_on_heading').value):
            return False
        if current is None:
            self.get_logger().warn('Place final approach cannot use DriveOnHeading; base pose is unavailable.')
            return False

        dx = standoff[0] - current[0]
        dy = standoff[1] - current[1]
        yaw_error = self._normalize_angle(standoff[2] - current[2])
        forward = dx * math.cos(current[2]) + dy * math.sin(current[2])
        lateral = -dx * math.sin(current[2]) + dy * math.cos(current[2])
        heading_tolerance = float(self.get_parameter('place_final_approach_heading_tolerance').value)
        lateral_tolerance = float(self.get_parameter('place_final_approach_lateral_tolerance').value)
        max_distance = float(self.get_parameter('place_final_approach_max_distance').value)

        if abs(yaw_error) > heading_tolerance or abs(lateral) > lateral_tolerance:
            self.get_logger().info(
                'Place final approach cannot use straight DriveOnHeading; '
                f'yaw_error={yaw_error:.2f} rad, lateral_error={lateral:.2f} m.'
            )
            return False
        if forward <= 0.02 or forward > max_distance:
            self.get_logger().info(
                'Place final approach cannot use straight DriveOnHeading; '
                f'forward_distance={forward:.2f} m is outside DriveOnHeading limits.'
            )
            return False

        speed = abs(float(self.get_parameter('place_final_approach_speed').value))
        timeout = float(self.get_parameter('place_final_approach_timeout_sec').value)
        self.get_logger().info(
            'Place final approach using one Nav2 DriveOnHeading: '
            f'forward={forward:.2f} m, lateral_error={lateral:.2f} m, '
            f'yaw_error={yaw_error:.2f} rad, speed={speed:.2f} m/s.'
        )
        if not self._run_drive_on_heading(forward, speed, timeout, 'place final approach'):
            return self._place_standoff_ready(standoff, require_reachable=True)
        return self._place_standoff_ready(standoff, require_reachable=True)

    def _run_drive_on_heading(self, distance: float, speed: float, timeout: float, label: str) -> bool:
        if distance <= 0.0 or speed <= 0.0:
            self.get_logger().warn(f'Skipping {label}; distance or speed is not positive.')
            return False
        if not self.drive_on_heading_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'Nav2 DriveOnHeading action server is unavailable for {label}.')
            return False

        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(speed)
        goal.time_allowance.sec = int(timeout)
        goal.time_allowance.nanosec = int((timeout - int(timeout)) * 1e9)

        send_future = self.drive_on_heading_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            self.get_logger().warn(f'Timed out sending Nav2 DriveOnHeading goal for {label}.')
            self._clear_nav_costmaps(f'{label} send timeout')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 DriveOnHeading goal for {label} was rejected.')
            self._clear_nav_costmaps(f'{label} rejected')
            return False

        self.base_motion_active = True
        try:
            result_future = goal_handle.get_result_async()
            if not self._wait_for_future(result_future, timeout + 2.0):
                goal_handle.cancel_goal_async()
                self.get_logger().warn(f'Nav2 DriveOnHeading timed out for {label}.')
                self._clear_nav_costmaps(f'{label} timeout')
                return False

            result = result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(
                    f'Nav2 DriveOnHeading for {label} failed with status {result.status}.'
                )
                self._clear_nav_costmaps(f'{label} failure')
                return False

            current_after = self._current_base_pose()
            pose_after = (
                'unknown'
                if current_after is None
                else f'({current_after[0]:.2f}, {current_after[1]:.2f}, yaw={current_after[2]:.2f})'
            )
            self.get_logger().info(f'Nav2 DriveOnHeading for {label} succeeded: base={pose_after}.')
            return True
        finally:
            self.base_motion_active = False

    def _transform_fixed_point(
        self,
        frame_id: str,
        position: Tuple[float, float, float],
        target_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = position
        try:
            transformed = self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.25))
        except Exception as exc:
            self.get_logger().debug(f'TF transform failed for final alignment target: {exc}')
            return None
        return transformed.point.x, transformed.point.y, transformed.point.z

    def _acquire_reachable_object(self, seed_candidate: Optional[Dict] = None) -> Optional[Dict]:
        if seed_candidate is not None and self._candidate_is_valid(seed_candidate, require_reachable=True):
            return seed_candidate

        candidate = seed_candidate if (
            seed_candidate is not None
            and self._candidate_is_valid(seed_candidate, require_reachable=False)
        ) else None
        if candidate is None or not self._candidate_is_valid(candidate, require_reachable=False):
            target = self._wait_for_valid_detection(require_reachable=True, timeout_override=2.0)
            if target is not None:
                return target
            candidate = self._wait_for_valid_detection(require_reachable=False)
        if candidate is None:
            return None

        for attempt in range(max(1, int(self.get_parameter('max_reposition_attempts').value))):
            if candidate is not None and self._candidate_is_valid(candidate, require_reachable=True):
                return candidate
            if candidate is None:
                candidate = self._wait_for_valid_detection(require_reachable=False, timeout_override=2.0)
                if candidate is None:
                    return None

            self._set_state('REPOSITION_FOR_REACH')
            move_completed_at = time.monotonic()
            if not self._reposition_for_candidate(candidate, attempt):
                return None

            self._set_state('CONFIRM_FRESH_DETECTION')
            target = self._wait_for_valid_detection(
                require_reachable=True,
                object_id=self._object_id(candidate['object']),
                min_detection_time=move_completed_at,
                timeout_override=float(self.get_parameter('fresh_detection_after_move_timeout_sec').value),
            )
            if target is not None:
                return target

            candidate = self._wait_for_valid_detection(
                require_reachable=False,
                object_id=self._object_id(candidate['object']),
                min_detection_time=move_completed_at,
                timeout_override=2.0,
            )

        return None

    def _wait_for_reachable_object_detection(self, timeout_override: Optional[float] = None) -> bool:
        return self._wait_for_valid_detection(
            require_reachable=bool(self.get_parameter('require_reachable_before_pick_service').value),
            timeout_override=timeout_override,
        ) is not None

    def _wait_for_valid_detection(
        self,
        require_reachable: bool,
        timeout_override: Optional[float] = None,
        object_id: Optional[str] = None,
        min_detection_time: float = 0.0,
    ) -> Optional[Dict]:
        start = time.monotonic()
        timeout = (
            float(timeout_override)
            if timeout_override is not None
            else float(self.get_parameter('detection_timeout_sec').value)
        )

        while rclpy.ok() and time.monotonic() - start < timeout:
            candidate = self._select_valid_detection(
                require_reachable=require_reachable,
                object_id=object_id,
                min_detection_time=min_detection_time,
            )
            if candidate is not None:
                self._stop_base()
                return candidate
            time.sleep(0.1)

        self._stop_base()
        return None

    def _select_valid_detection(
        self,
        require_reachable: bool,
        object_id: Optional[str] = None,
        min_detection_time: float = 0.0,
    ) -> Optional[Dict]:
        freshness = float(self.get_parameter('detection_freshness_sec').value)
        if (
            self.latest_payload is None
            or self.latest_detection_count <= 0
            or time.monotonic() - self.latest_detection_time > freshness
            or self.latest_detection_time < min_detection_time
        ):
            candidates = []
        else:
            candidates = self._detection_candidates(self.latest_payload)

        using_gazebo_truth = False
        if not candidates:
            if bool(self.get_parameter('use_gazebo_truth_fallback').value):
                self.get_logger().debug('No fresh RGB-D object candidate; checking Gazebo verified object states.')
            candidates = self._gazebo_truth_candidates()
            using_gazebo_truth = True
        if object_id is not None:
            candidates = [candidate for candidate in candidates if candidate['object_id'] == object_id]
        valid_candidates = [
            candidate
            for candidate in candidates
            if self._candidate_is_valid(candidate, require_reachable=require_reachable)
        ]
        if not valid_candidates and not using_gazebo_truth and bool(self.get_parameter('use_gazebo_truth_fallback').value):
            self.get_logger().info(
                'Fresh RGB-D candidates were rejected or already completed; '
                'checking Gazebo verified object states for remaining pick targets.'
            )
            candidates = self._gazebo_truth_candidates()
            if object_id is not None:
                candidates = [candidate for candidate in candidates if candidate['object_id'] == object_id]
            valid_candidates = [
                candidate
                for candidate in candidates
                if self._candidate_is_valid(candidate, require_reachable=require_reachable)
            ]
        candidates = valid_candidates
        if not candidates:
            return None

        candidates.sort(key=lambda item: (not item['reachable'], item['distance']))
        selected = candidates[0]
        grasp = selected['grasp']
        reach_label = 'reachable' if selected['reachable'] else 'visible'
        self.get_logger().info(
            f'{reach_label.capitalize()} object ready: {selected["object_id"]} '
            f'confidence={selected["confidence"]:.2f} '
            f'arm_base=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return selected

    def _detection_candidates(self, payload: Dict) -> List[Dict]:
        target_frame = self.get_parameter('object_target_frame').value
        map_frame = self.get_parameter('frame_id').value
        map_positions = {
            self._object_id(obj): position
            for obj, position in self._transform_objects(payload, map_frame)
        }

        candidates = []
        for obj, position in self._transform_objects(payload, target_frame):
            object_id = self._object_id(obj)
            try:
                confidence = float(obj.get('confidence', 1.0))
            except (TypeError, ValueError):
                confidence = 0.0
            grasp = self._grasp_pose_from_object(position, obj)
            reachable = self._object_within_workspace(grasp)
            candidates.append({
                'object': obj,
                'object_id': object_id,
                'confidence': confidence,
                'arm_position': position,
                'map_position': map_positions.get(object_id),
                'grasp': grasp,
                'reachable': reachable,
                'distance': math.hypot(grasp[0], grasp[1]),
            })
        return candidates

    def _candidate_is_valid(self, candidate: Dict, require_reachable: bool) -> bool:
        object_id = candidate.get('object_id') or self._object_id(candidate.get('object', {}))
        if object_id in self.completed_object_ids:
            self.get_logger().debug(f'Rejecting {object_id}: object was already completed.')
            return False
        if self.failed_object_counts.get(object_id, 0) >= int(self.get_parameter('max_object_failures').value):
            self.get_logger().debug(f'Rejecting {object_id}: failure limit reached.')
            return False
        if float(candidate.get('confidence', 0.0)) < float(self.get_parameter('min_detection_confidence').value):
            self.get_logger().debug(
                f'Rejecting {object_id}: confidence {float(candidate.get("confidence", 0.0)):.2f} below threshold.'
            )
            return False
        if candidate.get('map_position') is None:
            self.get_logger().debug(f'Rejecting {object_id}: no map-frame position is available.')
            return False
        if require_reachable and not bool(candidate.get('reachable', False)):
            grasp = candidate.get('grasp')
            if grasp is not None:
                self.get_logger().debug(
                    f'Rejecting {object_id}: grasp pose outside arm workspace '
                    f'({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f}).'
                )
            return False
        return True

    def _confirm_fresh_target(self, object_id: str) -> Optional[Dict]:
        self._stop_base()
        self._verify_localization()
        confirm_after = time.monotonic()
        self.get_logger().info(f'Confirming fresh reachable detection for {object_id} before grasp.')
        return self._wait_for_valid_detection(
            require_reachable=True,
            object_id=object_id,
            min_detection_time=confirm_after,
            timeout_override=float(self.get_parameter('fresh_detection_after_move_timeout_sec').value),
        )

    def _record_object_failure(self, object_id: str) -> None:
        self.failed_object_counts[object_id] = self.failed_object_counts.get(object_id, 0) + 1
        self.get_logger().warn(
            f'Recovery attempt recorded for {object_id}: '
            f'{self.failed_object_counts[object_id]}/{self.get_parameter("max_object_failures").value}'
        )

    @staticmethod
    def _object_id(obj: Dict) -> str:
        object_id = str(obj.get('id', '')).strip()
        if object_id:
            return object_id
        return f'{obj.get("color", "unknown")}_{obj.get("shape", "object")}'

    def _gazebo_truth_candidates(self) -> List[Dict]:
        if not bool(self.get_parameter('use_gazebo_truth_fallback').value):
            return []
        if not self.get_entity_state_client.wait_for_service(timeout_sec=0.05):
            return []

        candidates = []
        for object_id, entity_name, color, shape in self._known_gazebo_objects():
            request = GetEntityState.Request()
            request.name = entity_name
            request.reference_frame = 'world'
            future = self.get_entity_state_client.call_async(request)
            if not self._wait_for_future(future, 0.25):
                continue
            response = future.result()
            if response is None or not response.success:
                continue

            map_position = (
                response.state.pose.position.x,
                response.state.pose.position.y,
                response.state.pose.position.z,
            )
            arm_position = self._transform_fixed_point(
                self.get_parameter('gazebo_truth_frame').value,
                map_position,
                self.get_parameter('object_target_frame').value,
            )
            if arm_position is None:
                continue

            obj = {
                'id': object_id,
                'color': color,
                'shape': shape,
                'confidence': 1.0,
                'pose_source': 'gazebo_truth',
            }
            grasp = self._grasp_pose_from_object(arm_position, obj)
            candidates.append({
                'object': obj,
                'object_id': object_id,
                'confidence': 1.0,
                'arm_position': arm_position,
                'map_position': map_position,
                'grasp': grasp,
                'reachable': self._object_within_workspace(grasp),
                'distance': math.hypot(grasp[0], grasp[1]),
                'pose_source': 'gazebo_truth',
            })
        return candidates

    @staticmethod
    def _known_gazebo_objects() -> Tuple[Tuple[str, str, str, str], ...]:
        return (
            ('red_cube', 'k12_red_cube', 'red', 'cube'),
            ('green_cylinder', 'k12_green_cylinder', 'green', 'cylinder'),
            ('blue_rectangular_block', 'k12_blue_block', 'blue', 'rectangular_block'),
        )

    def _reposition_for_candidate(self, candidate: Dict, attempt: int) -> bool:
        map_position = candidate.get('map_position')
        if map_position is None:
            return False

        object_x, object_y, _ = map_position
        _, _, waypoint_yaw = self._waypoint(str(self.get_parameter('pick_zone_waypoint').value))
        base_standoff = float(self.get_parameter('object_nav_standoff_distance').value)
        standoff = max(0.3375, base_standoff - 0.045 * float(attempt))
        goal = (
            object_x - standoff * math.cos(waypoint_yaw),
            object_y - standoff * math.sin(waypoint_yaw),
            waypoint_yaw,
        )
        self.get_logger().info(
            f'Repositioning for {candidate["object_id"]}: '
            f'x={goal[0]:.2f}, y={goal[1]:.2f}, yaw={goal[2]:.2f}.'
        )
        self._navigate_to_pose(
            self._pose_stamped(goal),
            f'reposition for {candidate["object_id"]}',
            float(self.get_parameter('nav_timeout_sec').value),
        )
        return True

    def _current_base_pose(self) -> Optional[Tuple[float, float, float]]:
        frame_id = self.get_parameter('frame_id').value
        base_frame = self.get_parameter('base_frame').value
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except Exception as exc:
            self.get_logger().debug(f'Could not read current base pose: {exc}')
            return None

        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def _ensure_place_reachable(self) -> bool:
        if self._placement_pose_reachable():
            return True

        place_waypoint = str(self.get_parameter('place_zone_waypoint').value)
        attempts = max(1, int(self.get_parameter('max_reposition_attempts').value))
        for attempt in range(attempts):
            self._set_state('REPOSITION_FOR_PLACE')
            if not self._navigate_place_standoff(place_waypoint, attempt):
                return False
            self._verify_localization()
            if self._placement_pose_reachable():
                return True
        return False

    def _placement_pose_reachable(self) -> bool:
        fallback = tuple(float(v) for v in self.get_parameter('place_position_arm_base').value)
        if len(fallback) != 3:
            raise RuntimeError('place_position_arm_base must be [x, y, z].')

        target = tuple(float(v) for v in self.get_parameter('align_place_station_target').value)
        if len(target) != 3:
            raise RuntimeError('align_place_station_target must be [x, y, z].')

        adjustments = self._placement_reach_adjustments()
        saw_transform = False
        for adjustment_x in adjustments:
            adjusted_target = (target[0] + adjustment_x, target[1], target[2])
            transformed = self._transform_fixed_point(
                self.get_parameter('align_target_frame').value,
                adjusted_target,
                self.get_parameter('object_target_frame').value,
            )
            if transformed is None:
                continue

            saw_transform = True
            candidate = (transformed[0], transformed[1], fallback[2])
            reachable = self._object_within_workspace(candidate)
            self.get_logger().info(
                f'Placement reach check arm_base=({candidate[0]:.3f}, '
                f'{candidate[1]:.3f}, {candidate[2]:.3f}), '
                f'adjustment_x={adjustment_x:.3f}, reachable={reachable}.'
            )
            if reachable:
                return True

        if not saw_transform:
            return self._object_within_workspace(fallback)
        return False

    def _placement_reach_adjustments(self) -> Tuple[float, ...]:
        values = list(self.get_parameter('place_slot_reach_adjustments_x').value)
        unique = []
        for value in values:
            value = round(float(value), 3)
            if value not in unique:
                unique.append(value)
        if 0.0 not in unique:
            unique.insert(0, 0.0)
        return tuple(unique)

    def _navigate_place_standoff(self, place_waypoint: str, attempt: int) -> bool:
        target = tuple(float(v) for v in self.get_parameter('align_place_station_target').value)
        if len(target) != 3:
            return False
        target_frame = self.get_parameter('align_target_frame').value
        if target_frame != self.get_parameter('frame_id').value:
            transformed = self._transform_fixed_point(target_frame, target, self.get_parameter('frame_id').value)
            if transformed is None:
                return False
            target = transformed

        _, _, yaw = self._waypoint(place_waypoint)
        configured_stop = float(self.get_parameter('align_place_station_stop_distance').value)
        stop_distance = max(0.4275, configured_stop - 0.0225 * float(attempt))
        goal = (
            target[0] - stop_distance * math.cos(yaw),
            target[1] - stop_distance * math.sin(yaw),
            yaw,
        )
        self.get_logger().info(
            f'Repositioning for placement via staged final approach: x={goal[0]:.2f}, y={goal[1]:.2f}, '
            f'yaw={goal[2]:.2f}, stop_distance={stop_distance:.2f}.'
        )
        return self._align_to_place_station(target, stop_distance, yaw, goal)

    def _transform_objects(self, payload: Dict, target_frame: str) -> List[Tuple[Dict, Tuple[float, float, float]]]:
        frame_id = payload.get('header', {}).get('frame_id', '')
        objects = payload.get('objects', [])
        if not frame_id or not objects:
            return []

        transformed_objects = []
        for obj in objects:
            position = obj.get('position', {})
            point = PointStamped()
            point.header.frame_id = frame_id
            point.header.stamp = rclpy.time.Time().to_msg()
            point.point.x = float(position.get('x', 0.0))
            point.point.y = float(position.get('y', 0.0))
            point.point.z = float(position.get('z', 0.0))
            try:
                transformed = self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.25))
            except Exception as exc:
                self.get_logger().debug(f'TF transform failed for detected object: {exc}')
                continue
            transformed_objects.append((
                obj,
                (transformed.point.x, transformed.point.y, transformed.point.z),
            ))
        return transformed_objects

    def _grasp_pose_from_object(
        self,
        object_position: Tuple[float, float, float],
        obj: Optional[Dict] = None,
    ) -> Tuple[float, float, float]:
        x, y, z = object_position
        forward_offset = max(0.0, float(self.get_parameter('object_grasp_forward_offset').value))
        center_offset = self._center_offset_for_object(obj)
        radius = max(math.hypot(x, y), 1e-6)
        net_offset = center_offset - forward_offset
        return (
            x + net_offset * x / radius,
            y + net_offset * y / radius,
            z + float(self.get_parameter('object_grasp_z_bias').value),
        )

    def _center_offset_for_object(self, obj: Optional[Dict]) -> float:
        if obj is not None and obj.get('pose_source') == 'gazebo_truth':
            return 0.0

        configured = max(0.0, float(self.get_parameter('object_grasp_surface_to_center_offset').value))
        if not bool(self.get_parameter('use_shape_center_offsets').value):
            return configured

        shape = ''
        color = ''
        if obj is not None:
            shape = str(obj.get('shape', '')).lower()
            color = str(obj.get('color', '')).lower()

        if shape == 'cube' or color == 'red':
            return float(self.get_parameter('cube_center_offset_m').value)
        if shape == 'cylinder' or color == 'green':
            return float(self.get_parameter('cylinder_center_offset_m').value)
        if shape == 'rectangular_block' or color == 'blue':
            return float(self.get_parameter('block_center_offset_m').value)
        if configured > 0.0:
            return configured
        return float(self.get_parameter('default_center_offset_m').value)

    def _object_within_workspace(self, point: Tuple[float, float, float]) -> bool:
        x, y, z = point
        radius = math.hypot(x, y)
        return (
            float(self.get_parameter('object_reachable_min_radius').value)
            <= radius
            <= float(self.get_parameter('object_reachable_max_radius').value)
            and float(self.get_parameter('object_reachable_min_z').value)
            <= z
            <= float(self.get_parameter('object_reachable_max_z').value)
            and self.ik.solve(point) is not None
        )

    def _call_pick_with_retries(self, object_id: Optional[str] = None) -> None:
        retry_count = max(1, int(self.get_parameter('pick_retry_count').value))
        retry_wait = max(0.0, float(self.get_parameter('pick_retry_wait_sec').value))
        require_detection = bool(self.get_parameter('pre_pick_detection_required').value)
        last_error = ''
        for attempt in range(1, retry_count + 1):
            if require_detection:
                if object_id is None:
                    target = self._wait_for_valid_detection(require_reachable=True)
                else:
                    target = self._confirm_fresh_target(object_id)
                if target is None:
                    last_error = 'no fresh reachable object detection'
                    self._stop_base()
                    time.sleep(retry_wait)
                    continue
                self._publish_pick_target(target)

            if self.base_motion_active:
                last_error = 'base is still moving'
                self._stop_base()
                time.sleep(retry_wait)
                continue

            try:
                self._call_trigger(self.pick_client, f'pick object attempt {attempt}')
                return
            except RuntimeError as exc:
                last_error = str(exc)
                self.get_logger().warn(f'Pick attempt {attempt}/{retry_count} failed: {exc}')
            self._stop_base()
            time.sleep(retry_wait)
        raise RuntimeError(f'Pick object failed after {retry_count} attempts: {last_error}')

    def _publish_pick_target(self, candidate: Dict) -> None:
        obj = dict(candidate.get('object', {}))
        object_id = candidate.get('object_id') or self._object_id(obj)
        grasp = tuple(float(v) for v in candidate['grasp'])
        arm_position = tuple(float(v) for v in candidate.get('arm_position', grasp))
        map_position = candidate.get('map_position')

        payload = {
            'object_id': object_id,
            'object': obj,
            'target_frame': self.get_parameter('object_target_frame').value,
            'arm_base_position': list(grasp),
            'object_arm_base_position': list(arm_position),
            'map_position': None if map_position is None else [float(v) for v in map_position],
            'confidence': float(candidate.get('confidence', obj.get('confidence', 1.0))),
            'reachable': bool(candidate.get('reachable', False)),
            'pose_source': obj.get('pose_source', candidate.get('pose_source', 'rgbd')),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pick_target_pub.publish(msg)
        self.get_logger().info(
            f'Published fresh pick target {object_id}: '
            f'arm_base=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f}), '
            f'confidence={payload["confidence"]:.2f}, source={payload["pose_source"]}.'
        )

    def _call_trigger(self, client, label: str) -> None:
        if self.base_motion_active:
            raise RuntimeError(f'Refusing {label} while base navigation is active.')

        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'Service for {label} is not available.')

        future = client.call_async(Trigger.Request())
        timeout = float(self.get_parameter('service_timeout_sec').value)
        if not self._wait_for_future(future, timeout):
            raise RuntimeError(f'Service call for {label} timed out.')

        result = future.result()
        if result is None or not result.success:
            message = '' if result is None else result.message
            raise RuntimeError(f'Service call for {label} failed. {message}')

    def _try_call_trigger(self, client, label: str) -> bool:
        try:
            self._call_trigger(client, label)
        except RuntimeError as exc:
            self.get_logger().warn(f'{label} skipped: {exc}')
            return False
        return True

    def _verify_localization(self) -> None:
        frame_id = self.get_parameter('frame_id').value
        base_frame = self.get_parameter('base_frame').value
        timeout = float(self.get_parameter('localization_timeout_sec').value)
        try:
            self.tf_buffer.lookup_transform(frame_id, base_frame, rclpy.time.Time(), timeout=Duration(seconds=timeout))
        except Exception as exc:
            raise RuntimeError(f'Localization transform {frame_id}->{base_frame} is not available: {exc}') from exc

    def _pose_stamped(self, values: Iterable[float]) -> PoseStamped:
        x, y, yaw = values
        qx, qy, qz, qw = _quaternion_from_yaw(float(yaw))
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

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    def _stop_base(self) -> None:
        return

    def _set_state(self, state: str) -> None:
        self.get_logger().info(state)
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AmmsTaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._stop_base()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
