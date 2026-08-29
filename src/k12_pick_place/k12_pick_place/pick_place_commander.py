import json
import math
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers geometry transforms
import tf2_ros
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, GetLinkProperties, SetEntityState, SetLinkProperties
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from k12_pick_place.ik import PlanarArmIK, distance


class PickPlaceCommander(Node):
    def __init__(self):
        super().__init__('k12_pick_place_commander')

        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pick_target_topic', '/amms/pick_target')
        self.declare_parameter('prefer_external_pick_target', True)
        self.declare_parameter('external_pick_target_timeout_sec', 3.0)
        self.declare_parameter('arm_controller_topic', '/arm_position_controller/joint_trajectory')
        self.declare_parameter('arm_action_name', '/arm_position_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_controller_topic', '/gripper_position_controller/commands')
        self.declare_parameter('target_frame', 'arm_base_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('place_table_frame', 'map')
        self.declare_parameter('place_table_position', [0.0, -2.18, 0.0])
        self.declare_parameter('auto_start', True)
        self.declare_parameter('run_once', True)
        self.declare_parameter('object_timeout_sec', 8.0)
        self.declare_parameter('track_timeout_sec', 60.0)
        self.declare_parameter('min_detection_confidence', 0.25)
        self.declare_parameter('service_requires_fresh_detection', True)
        self.declare_parameter('service_pick_target_timeout_sec', 35.0)
        self.declare_parameter('require_stable_detection', True)
        self.declare_parameter('stable_detection_count', 2)
        self.declare_parameter('stable_detection_position_tolerance_m', 0.06)
        self.declare_parameter('stable_detection_timeout_sec', 3.0)
        self.declare_parameter('approach_distance', 0.09871875)
        self.declare_parameter('lift_height', 0.1184625)
        self.declare_parameter('grasp_z_bias', -0.0073125)
        self.declare_parameter('grasp_forward_offset', 0.0)
        self.declare_parameter('grasp_surface_to_center_offset', 0.02925)
        self.declare_parameter('use_shape_center_offsets', True)
        self.declare_parameter('cube_center_offset_m', 0.05625)
        self.declare_parameter('cylinder_center_offset_m', 0.05625)
        self.declare_parameter('block_center_offset_m', 0.05625)
        self.declare_parameter('default_center_offset_m', 0.05625)
        self.declare_parameter('grasp_close_settle_sec', 0.8)
        self.declare_parameter('open_gripper_width', 0.039)
        self.declare_parameter('closed_gripper_width', 0.0014625)
        self.declare_parameter('use_follow_joint_trajectory_action', True)
        self.declare_parameter('trajectory_action_server_timeout_sec', 15.0)
        self.declare_parameter('trajectory_goal_send_timeout_sec', 15.0)
        self.declare_parameter('trajectory_result_margin_sec', 8.0)
        self.declare_parameter('allow_trajectory_topic_fallback', True)
        self.declare_parameter('use_gazebo_grasp_assist', True)
        self.declare_parameter('require_verified_grasp_attachment', True)
        self.declare_parameter('grasp_assist_backend', 'gazebo_state')
        self.declare_parameter('gazebo_set_entity_state_service', '/set_entity_state')
        self.declare_parameter('gazebo_get_entity_state_service', '/get_entity_state')
        self.declare_parameter('gazebo_get_link_properties_service', '/get_link_properties')
        self.declare_parameter('gazebo_set_link_properties_service', '/set_link_properties')
        self.declare_parameter('use_gazebo_truth_fallback', True)
        self.declare_parameter('gazebo_truth_frame', 'map')
        self.declare_parameter('disable_gravity_while_holding', True)
        self.declare_parameter('use_live_grasp_offset', True)
        self.declare_parameter('held_object_offset_end_effector', [0.2120625, 0.0, 0.0])
        self.declare_parameter('held_object_reference_frame', 'k12_amms::end_effector')
        self.declare_parameter('held_object_orientation_follows_gripper', True)
        self.declare_parameter('max_live_grasp_offset_m', 0.950625)
        self.declare_parameter('release_object_z_bias', 0.0)
        self.declare_parameter('release_upright_orientation', True)
        self.declare_parameter('snap_release_to_table_center', True)
        self.declare_parameter('place_table_release_height', 0.32)
        self.declare_parameter('place_table_top_z', 0.30)
        self.declare_parameter('release_clearance', 0.020)
        self.declare_parameter('release_cube_height', 0.07425)
        self.declare_parameter('release_cylinder_height', 0.07425)
        self.declare_parameter('release_block_height', 0.07425)
        self.declare_parameter('place_table_release_xy_offset', [0.0, 0.0])
        self.declare_parameter('place_table_release_red_cube_xy_offset', [0.0, 0.0])
        self.declare_parameter('place_table_release_green_cylinder_xy_offset', [-0.09, 0.15])
        self.declare_parameter('place_table_release_blue_block_xy_offset', [-0.09, -0.15])
        self.declare_parameter('place_slot_reach_adjustments_x', [0.0, -0.026325, -0.05265, -0.078975])
        self.declare_parameter('release_settle_before_open_sec', 0.15)
        self.declare_parameter('hold_object_during_place_open', True)
        self.declare_parameter('place_contact_settle_sec', 0.60)
        self.declare_parameter('place_lift_height', 0.078975)
        self.declare_parameter('place_retreat_lift_height', 0.078975)
        self.declare_parameter('place_retreat_steps', 4)
        self.declare_parameter('place_retreat_step_duration_sec', 0.50)
        self.declare_parameter('held_object_update_period_sec', 0.02)
        self.declare_parameter('held_object_request_timeout_sec', 0.03)
        self.declare_parameter('held_object_update_wait_sec', 0.02)
        self.declare_parameter('held_gripper_republish_sec', 0.05)
        self.declare_parameter('pick_service', '/amms/pick_object')
        self.declare_parameter('place_service', '/amms/place_object')
        self.declare_parameter('fold_arm_service', '/amms/fold_arm')
        self.declare_parameter('home_joints', [0.0, -1.35, 2.20, -0.85, 0.0])
        self.declare_parameter('carry_joints', [0.0, -1.10, 1.85, -0.75, 0.0])
        self.declare_parameter('place_position_arm_base', [0.505, 0.0, 0.026])
        self.declare_parameter('reachable_min_radius', 0.1447875)
        self.declare_parameter('reachable_max_radius', 0.658125)
        self.declare_parameter('reachable_min_z', -0.307125)
        self.declare_parameter('reachable_max_z', 0.23034375)
        self.declare_parameter('allow_best_effort_grasp', True)
        self.declare_parameter('best_effort_max_radius', 0.78975)

        self.joint_names = [
            'arm_base_yaw_joint',
            'arm_shoulder_pitch_joint',
            'arm_elbow_pitch_joint',
            'arm_wrist_pitch_joint',
            'arm_wrist_roll_joint',
        ]
        self.ik = PlanarArmIK()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.command_callback_group = ReentrantCallbackGroup()

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter('arm_controller_topic').value,
            10,
        )
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('arm_action_name').value,
            callback_group=self.command_callback_group,
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('gripper_controller_topic').value,
            10,
        )
        self.entity_state_client = self.create_client(
            SetEntityState,
            self.get_parameter('gazebo_set_entity_state_service').value,
            callback_group=self.command_callback_group,
        )
        self.get_entity_state_client = self.create_client(
            GetEntityState,
            self.get_parameter('gazebo_get_entity_state_service').value,
            callback_group=self.command_callback_group,
        )
        self.get_link_properties_client = self.create_client(
            GetLinkProperties,
            self.get_parameter('gazebo_get_link_properties_service').value,
            callback_group=self.command_callback_group,
        )
        self.set_link_properties_client = self.create_client(
            SetLinkProperties,
            self.get_parameter('gazebo_set_link_properties_service').value,
            callback_group=self.command_callback_group,
        )
        self.create_subscription(String, self.get_parameter('detections_topic').value, self._on_detections, 10)
        self.create_subscription(String, self.get_parameter('pick_target_topic').value, self._on_pick_target, 10)
        # ADDED: joint_states feedback. Previously _open_gripper/_close_gripper
        # were fully open-loop -- publish a target width, wait a fixed 0.8s,
        # assume success. If physics-engine jitter, tiny solver asymmetry, or
        # an incidental self-collision made one finger lag, there was no way
        # to detect or correct it -- this is the "sometimes the right finger
        # doesn't open" symptom. latest_joint_positions lets the gripper
        # commands actually verify both fingers reached target before moving
        # on, and keep nudging a lagging one instead of silently continuing.
        self.latest_joint_positions: Dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_service(
            Trigger,
            self.get_parameter('pick_service').value,
            self._on_pick_request,
            callback_group=self.command_callback_group,
        )
        self.create_service(
            Trigger,
            self.get_parameter('place_service').value,
            self._on_place_request,
            callback_group=self.command_callback_group,
        )
        self.create_service(
            Trigger,
            self.get_parameter('fold_arm_service').value,
            self._on_fold_arm_request,
            callback_group=self.command_callback_group,
        )

        self.latest_payload: Optional[Dict] = None
        self.latest_time = 0.0
        self.latest_external_pick_target: Optional[Dict] = None
        self.latest_external_pick_target_time = 0.0
        self.tracked_target: Optional[Dict] = None
        self.tracked_time = 0.0
        self.stable_target_key = ''
        self.stable_target_position: Optional[Tuple[float, float, float]] = None
        self.stable_target_count = 0
        self.stable_target_time = 0.0
        self.hold_lock = threading.Lock()
        self.held_entity_name: Optional[str] = None
        self.held_object_offset: Optional[Tuple[float, float, float]] = None
        self.hold_request_pending = None
        self.hold_request_started = 0.0
        self.hold_request_reference_frame = ''
        self.hold_reference_frame_failed = False
        self.hold_request_failure_warned = False
        self.last_hold_position: Optional[Tuple[float, float, float]] = None
        self.last_hold_orientation: Optional[Tuple[float, float, float, float]] = None
        self.link_properties_cache: Dict[str, GetLinkProperties.Response] = {}
        self.hold_gripper_republish_suspended = False
        self.place_table_adjustment_x = 0.0
        self.executing = False
        self.completed = False
        self.create_timer(0.5, self._tick)
        self.create_timer(float(self.get_parameter('held_object_update_period_sec').value), self._update_held_object)
        self.create_timer(float(self.get_parameter('held_gripper_republish_sec').value), self._hold_gripper_closed)

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid detection JSON: {exc}')
            return
        self.latest_payload = payload
        self.latest_time = time.monotonic()
        if payload.get('objects', []):
            self._update_tracked_target(payload)

    def _on_pick_target(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid pick target JSON: {exc}')
            return
        self.latest_external_pick_target = payload
        self.latest_external_pick_target_time = time.monotonic()

    def _tick(self) -> None:
        if not bool(self.get_parameter('auto_start').value):
            return
        if self.executing:
            return
        if self.completed and bool(self.get_parameter('run_once').value):
            return
        if self.latest_payload is None:
            self._stop_base()
            return
        latest_age = time.monotonic() - self.latest_time
        if latest_age > float(self.get_parameter('object_timeout_sec').value) and self.tracked_target is None:
            self._stop_base()
            return

        self._update_tracked_target(self.latest_payload)
        target = self._select_reachable_target(self.latest_payload)
        if target is None:
            target = self._select_reachable_tracked_target()
        if target is not None:
            if not self._accept_stable_pick_target(target):
                self._stop_base()
                return
            self._stop_base()
            self.executing = True
            thread = threading.Thread(target=self._execute_sequence, args=(target,), daemon=True)
            thread.start()
            return

        self._reset_stable_pick_target()
        self._stop_base()

    def _select_current_pick_target(self) -> Optional[Dict]:
        target = self._select_external_pick_target()
        if target is not None:
            return target

        if self.latest_payload is None:
            target = self._select_gazebo_truth_target()
            if target is not None:
                return target
            if bool(self.get_parameter('service_requires_fresh_detection').value):
                return None
            return self._select_reachable_tracked_target()

        latest_age = time.monotonic() - self.latest_time
        if latest_age <= float(self.get_parameter('object_timeout_sec').value):
            self._update_tracked_target(self.latest_payload)
            target = self._select_reachable_target(self.latest_payload)
            if target is not None:
                return target
            target = self._select_gazebo_truth_target()
            if target is not None:
                return target
        if bool(self.get_parameter('service_requires_fresh_detection').value):
            return self._select_gazebo_truth_target()
        return self._select_reachable_tracked_target()

    def _select_external_pick_target(self) -> Optional[Dict]:
        if not bool(self.get_parameter('prefer_external_pick_target').value):
            return None
        payload = self.latest_external_pick_target
        if payload is None:
            return None
        age = time.monotonic() - self.latest_external_pick_target_time
        if age > float(self.get_parameter('external_pick_target_timeout_sec').value):
            return None

        obj = dict(payload.get('object', {}))
        object_id = str(payload.get('object_id', obj.get('id', ''))).strip()
        if object_id:
            obj['id'] = object_id
        if 'confidence' not in obj and 'confidence' in payload:
            obj['confidence'] = payload['confidence']
        if 'pose_source' not in obj and 'pose_source' in payload:
            obj['pose_source'] = payload['pose_source']
        if not self._detection_has_confidence(obj):
            return None

        grasp = self._position_from_payload(payload.get('arm_base_position'))
        if grasp is None:
            self.get_logger().warn(f'Rejecting external pick target {object_id}: missing arm_base_position.')
            return None
        object_position = self._position_from_payload(payload.get('object_arm_base_position')) or grasp

        plan = self._pick_plan_for_grasp(grasp)
        if plan is None:
            self.get_logger().warn(
                f'Rejecting external pick target {object_id}: '
                f'{self._pick_plan_rejection_reason(grasp)}'
            )
            return None

        target = self._target_from_pick_plan(obj, plan, object_position)
        self.get_logger().info(
            f'Selected externally confirmed {target.get("id", target.get("color", "object"))}: '
            f'grasp=({plan["grasp"][0]:.3f}, {plan["grasp"][1]:.3f}, {plan["grasp"][2]:.3f}), '
            f'pre_grasp=({plan["pre_grasp"][0]:.3f}, {plan["pre_grasp"][1]:.3f}, {plan["pre_grasp"][2]:.3f}), '
            f'lift=({plan["lift"][0]:.3f}, {plan["lift"][1]:.3f}, {plan["lift"][2]:.3f}).'
        )
        return target

    @staticmethod
    def _position_from_payload(value) -> Optional[Tuple[float, float, float]]:
        if value is None:
            return None
        try:
            if isinstance(value, dict):
                return float(value['x']), float(value['y']), float(value['z'])
            x, y, z = value
            return float(x), float(y), float(z)
        except (KeyError, TypeError, ValueError):
            return None

    def _wait_for_service_pick_target(self) -> Optional[Dict]:
        start = time.monotonic()
        timeout = float(self.get_parameter('service_pick_target_timeout_sec').value)
        self._reset_stable_pick_target()

        while rclpy.ok() and time.monotonic() - start < timeout:
            target = self._select_current_pick_target()
            if target is not None and self._accept_stable_pick_target(target):
                self._stop_base()
                return target
            if target is None:
                self._reset_stable_pick_target()

            self._sleep(0.1)

        self._stop_base()
        return None

    def _accept_stable_pick_target(self, target: Dict) -> bool:
        required = max(1, int(self.get_parameter('stable_detection_count').value))
        if not bool(self.get_parameter('require_stable_detection').value) or required <= 1:
            return True

        key = self._target_key(target)
        position = self._target_position(target)
        if position is None:
            return False

        now = time.monotonic()
        tolerance = max(0.0, float(self.get_parameter('stable_detection_position_tolerance_m').value))
        timeout = max(0.0, float(self.get_parameter('stable_detection_timeout_sec').value))
        same_target = (
            key == self.stable_target_key
            and self.stable_target_position is not None
            and now - self.stable_target_time <= timeout
            and distance((
                position[0] - self.stable_target_position[0],
                position[1] - self.stable_target_position[1],
                position[2] - self.stable_target_position[2],
            )) <= tolerance
        )

        self.stable_target_key = key
        self.stable_target_position = position
        self.stable_target_time = now
        self.stable_target_count = self.stable_target_count + 1 if same_target else 1

        if self.stable_target_count < required:
            self.get_logger().debug(
                f'Waiting for stable target {key}: '
                f'{self.stable_target_count}/{required} confirmations.'
            )
            return False
        return True

    def _reset_stable_pick_target(self) -> None:
        self.stable_target_key = ''
        self.stable_target_position = None
        self.stable_target_count = 0
        self.stable_target_time = 0.0

    @staticmethod
    def _target_key(target: Dict) -> str:
        object_id = str(target.get('id', '')).strip()
        if object_id:
            return object_id
        return f'{target.get("color", "")}:{target.get("shape", "")}'

    @staticmethod
    def _target_position(target: Dict) -> Optional[Tuple[float, float, float]]:
        values = target.get('arm_base_position', target.get('object_arm_base_position'))
        if values is None:
            return None
        try:
            x, y, z = values
        except (TypeError, ValueError):
            return None
        return float(x), float(y), float(z)

    def _detection_has_confidence(self, obj: Dict) -> bool:
        threshold = float(self.get_parameter('min_detection_confidence').value)
        try:
            confidence = float(obj.get('confidence', 1.0))
        except (TypeError, ValueError):
            confidence = 0.0
        if confidence >= threshold:
            return True
        self.get_logger().debug(
            f'Rejecting {obj.get("id", "object")} below confidence threshold: '
            f'{confidence:.2f} < {threshold:.2f}'
        )
        return False

    def _on_pick_request(self, _request, response):
        if self.executing:
            response.success = False
            response.message = 'Manipulator is already executing.'
            return response

        self.executing = True
        try:
            target = self._wait_for_service_pick_target()
            if target is None:
                response.success = False
                response.message = 'No reachable object target is available after fine approach.'
                return response
            self._execute_pick(target)
            response.success = True
            response.message = 'Pick complete; object is secured.'
        except Exception as exc:
            if self._holding_object():
                self._keep_object_held_after_error('pick failed')
            else:
                self._recover_after_failed_pick()
            response.success = False
            response.message = f'Pick failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.executing = False
        return response

    def _on_place_request(self, _request, response):
        if self.executing:
            response.success = False
            response.message = 'Manipulator is already executing.'
            return response

        self.executing = True
        try:
            self._execute_place()
            response.success = True
            response.message = 'Place complete.'
        except Exception as exc:
            self._keep_object_held_after_error('place failed')
            response.success = False
            response.message = f'Place failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.executing = False
        return response

    def _on_fold_arm_request(self, _request, response):
        if self.executing:
            response.success = False
            response.message = 'Manipulator is already executing.'
            return response

        self.executing = True
        try:
            self._stop_base()
            self._move_joints(self.get_parameter('carry_joints').value, 3.0, 'safe carry')
            response.success = True
            response.message = 'Arm folded into safe carry pose.'
        except Exception as exc:
            response.success = False
            response.message = f'Fold arm failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.executing = False
        return response

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
                self.get_logger().debug(f'TF transform failed for {obj.get("id", "object")}: {exc}')
                continue

            xyz = (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            )
            transformed_objects.append((obj, xyz))

        return transformed_objects

    def _update_tracked_target(self, payload: Dict) -> None:
        base_target = self._select_drive_target(payload)
        if base_target is None:
            return

        tracking_frame = self.get_parameter('tracking_frame').value
        selected_id = base_target['object'].get('id')
        for obj, xyz in self._transform_objects(payload, tracking_frame):
            if obj.get('id') != selected_id:
                continue
            self.tracked_target = {
                'object': dict(obj),
                'frame_id': tracking_frame,
                'position': xyz,
            }
            self.tracked_time = time.monotonic()
            return

    def _tracked_target_in_frame(self, target_frame: str) -> Optional[Tuple[Dict, Tuple[float, float, float]]]:
        if self.tracked_target is None:
            return None
        if time.monotonic() - self.tracked_time > float(self.get_parameter('track_timeout_sec').value):
            self.tracked_target = None
            return None

        point = PointStamped()
        point.header.frame_id = self.tracked_target['frame_id']
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = self.tracked_target['position']

        try:
            transformed = self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.25))
        except Exception as exc:
            self.get_logger().debug(f'TF transform failed for tracked object: {exc}')
            return None

        return (
            dict(self.tracked_target['object']),
            (transformed.point.x, transformed.point.y, transformed.point.z),
        )

    def _select_reachable_target(self, payload: Dict) -> Optional[Dict]:
        target_frame = self.get_parameter('target_frame').value
        candidates = []
        fallback_candidates = []
        for obj, xyz_raw in self._transform_objects(payload, target_frame):
            if not self._detection_has_confidence(obj):
                continue
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            plan = self._pick_plan_for_grasp(grasp)
            if plan is not None:
                candidates.append((distance(grasp), obj, plan, xyz_raw, False))
                continue
            self.get_logger().debug(
                f'Rejecting detected {obj.get("id", obj.get("color", "object"))}: '
                f'{self._pick_plan_rejection_reason(grasp)}'
            )

            fallback_plan = self._best_effort_pick_plan(grasp)
            if fallback_plan is not None:
                fallback_candidates.append((distance(grasp), obj, fallback_plan, xyz_raw, True))

        if not candidates:
            candidates = fallback_candidates
        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, obj, plan, xyz_raw, best_effort = candidates[0]
        obj = self._target_from_pick_plan(obj, plan, xyz_raw)
        grasp = plan['grasp']
        label = 'best-effort ' if best_effort else ''
        self.get_logger().info(
            f'Selected {label}{obj.get("color", "?")} {obj.get("shape", "object")} at '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return obj

    def _select_reachable_tracked_target(self) -> Optional[Dict]:
        tracked = self._tracked_target_in_frame(self.get_parameter('target_frame').value)
        if tracked is None:
            return None

        obj, xyz_raw = tracked
        if not self._detection_has_confidence(obj):
            return None
        grasp = self._grasp_pose_from_object(xyz_raw, obj)
        best_effort = False
        plan = self._pick_plan_for_grasp(grasp)
        if plan is None:
            self.get_logger().debug(
                f'Rejecting tracked {obj.get("id", obj.get("color", "object"))}: '
                f'{self._pick_plan_rejection_reason(grasp)}'
            )
            plan = self._best_effort_pick_plan(grasp)
            best_effort = True
        if plan is None:
            return None

        obj = self._target_from_pick_plan(obj, plan, xyz_raw)
        grasp = plan['grasp']
        label = 'best-effort tracked' if best_effort else 'tracked'
        self.get_logger().info(
            f'Selected {label} {obj.get("color", "?")} {obj.get("shape", "object")} at '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return obj

    def _select_gazebo_truth_target(self) -> Optional[Dict]:
        if not bool(self.get_parameter('use_gazebo_truth_fallback').value):
            return None
        if not self.get_entity_state_client.wait_for_service(timeout_sec=0.05):
            return None

        candidates = []
        for obj, xyz_raw in self._gazebo_truth_objects():
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            plan = self._pick_plan_for_grasp(grasp)
            if plan is None:
                self.get_logger().debug(
                    f'Rejecting Gazebo-verified {obj.get("id", obj.get("color", "object"))}: '
                    f'{self._pick_plan_rejection_reason(grasp)}'
                )
                continue
            candidates.append((distance(grasp), obj, plan, xyz_raw))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, obj, plan, xyz_raw = candidates[0]
        target = self._target_from_pick_plan(obj, plan, xyz_raw)
        grasp = plan['grasp']
        self.get_logger().info(
            f'Selected Gazebo-verified {target.get("color", "?")} '
            f'{target.get("shape", "object")} at '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return target

    def _gazebo_truth_objects(self) -> List[Tuple[Dict, Tuple[float, float, float]]]:
        objects = []
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

            world_position = (
                response.state.pose.position.x,
                response.state.pose.position.y,
                response.state.pose.position.z,
            )
            arm_position = self._transform_fixed_point(
                self.get_parameter('gazebo_truth_frame').value,
                world_position,
                self.get_parameter('target_frame').value,
            )
            if arm_position is None:
                continue
            objects.append((
                {
                    'id': object_id,
                    'color': color,
                    'shape': shape,
                    'confidence': 1.0,
                    'pose_source': 'gazebo_truth',
                },
                arm_position,
            ))
        return objects

    @staticmethod
    def _known_gazebo_objects() -> Tuple[Tuple[str, str, str, str], ...]:
        return (
            ('red_cube', 'k12_red_cube', 'red', 'cube'),
            ('green_cylinder', 'k12_green_cylinder', 'green', 'cylinder'),
            ('blue_rectangular_block', 'k12_blue_block', 'blue', 'rectangular_block'),
        )

    def _select_drive_target(self, payload: Dict) -> Optional[Dict]:
        base_frame = self.get_parameter('base_frame').value
        candidates = []
        for obj, xyz in self._transform_objects(payload, base_frame):
            if not self._detection_has_confidence(obj):
                continue
            x, y, _ = xyz
            if x <= 0.05:
                continue
            candidates.append((math.hypot(x, y), obj, xyz))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, obj, xyz = candidates[0]
        return {'object': obj, 'base_position': xyz}

    def _select_drive_tracked_target(self) -> Optional[Dict]:
        tracked = self._tracked_target_in_frame(self.get_parameter('base_frame').value)
        if tracked is None:
            return None

        obj, xyz = tracked
        if not self._detection_has_confidence(obj):
            return None
        x, _, _ = xyz
        if x <= 0.05:
            return None
        return {'object': obj, 'base_position': xyz}

    def _stop_base(self) -> None:
        return

    def _place_table_param(self, name_suffix: str):
        param_name = f'place_table_{name_suffix}'
        if self.has_parameter(param_name):
            val = self.get_parameter(param_name).value
            if val is not None:
                return val
        return None

    def _current_place_pose(self) -> Tuple[float, float, float]:
        fallback = self._compute_fallback_place_pose()
        with self.hold_lock:
            entity_name = self.held_entity_name
        offset = self._place_xy_offset_for_entity(entity_name)
        entity_label = 'held object' if entity_name is None else entity_name

        for adjustment_x in self._place_slot_reach_adjustments():
            shelf_pose = self._place_table_pose_for_entity(entity_name, adjustment_x=adjustment_x)
            transformed = self._transform_fixed_point(
                self._place_table_param('frame'),
                shelf_pose,
                self.get_parameter('target_frame').value,
            )
            if transformed is None:
                continue

            candidate = (transformed[0], transformed[1], transformed[2])
            if not self._within_workspace(candidate) or self.ik.solve(candidate) is None:
                self.get_logger().info(
                    f'Placement slot candidate rejected for {entity_label}: '
                    f'table_adjustment_x={adjustment_x:.2f}, '
                    f'arm_base=({candidate[0]:.3f}, {candidate[1]:.3f}, {candidate[2]:.3f}).'
                )
                continue

            if abs(adjustment_x) > 1e-6:
                self.get_logger().warn(
                    f'Adjusted placement slot for {entity_label} by {adjustment_x:.2f} m in table x '
                    'to keep the release pose inside the arm IK envelope.'
                )
            self.get_logger().info(
                f'Placing {entity_label} at table slot offset=({offset[0]:.2f}, {offset[1]:.2f}) '
                f'adjustment_x={adjustment_x:.2f} '
                f'arm_base=({candidate[0]:.3f}, {candidate[1]:.3f}, {candidate[2]:.3f})'
            )
            self.place_table_adjustment_x = adjustment_x
            return candidate

        if self._within_workspace(fallback) and self.ik.solve(fallback) is not None:
            self.get_logger().error(
                f'No reachable table slot was found for {entity_label}; '
                f'using configured fallback place pose arm_base=({fallback[0]:.3f}, '
                f'{fallback[1]:.3f}, {fallback[2]:.3f}).'
            )
            self.place_table_adjustment_x = 0.0
            return fallback

        raise RuntimeError(
            f'No reachable table slot or fallback place pose is available for {entity_label}.'
        )

    def _compute_fallback_place_pose(self) -> Tuple[float, float, float]:
        configured_fallback = tuple(float(v) for v in self.get_parameter('place_position_arm_base').value)
        shelf = tuple(float(v) for v in self._place_table_param('position'))
        place_top_z = float(self._place_table_param('top_z'))
        shelf_xy_z = (shelf[0], shelf[1], place_top_z)
        transformed = self._transform_fixed_point(
            self._place_table_param('frame'),
            shelf_xy_z,
            self.get_parameter('target_frame').value,
        )
        if transformed is not None:
            shelf_z_arm = transformed[2]
            clearance = float(self.get_parameter('release_clearance').value)
            safe_z = shelf_z_arm + clearance
            candidate = (configured_fallback[0], configured_fallback[1], safe_z)
            self.get_logger().info(
                f'Fallback place pose: ({candidate[0]:.3f}, {candidate[1]:.3f}, {candidate[2]:.3f}) '
                f'[shelf_z_arm={shelf_z_arm:.3f}, clearance={clearance:.4f}]'
            )
            return candidate
        self.get_logger().warn('Could not transform shelf Z for fallback; using configured fallback')
        return configured_fallback

    def _grasp_pose_from_object(
        self,
        object_arm_base: Tuple[float, float, float],
        obj: Optional[Dict] = None,
    ) -> Tuple[float, float, float]:
        x, y, z = object_arm_base
        forward_offset = max(0.0, float(self.get_parameter('grasp_forward_offset').value))
        center_offset = self._center_offset_for_object(obj)
        radius = max(math.hypot(x, y), 1e-6)
        net_offset = center_offset - forward_offset
        return (
            x + net_offset * x / radius,
            y + net_offset * y / radius,
            z + float(self.get_parameter('grasp_z_bias').value),
        )

    def _center_offset_for_object(self, obj: Optional[Dict]) -> float:
        if obj is not None and obj.get('pose_source') == 'gazebo_truth':
            return 0.0

        configured = max(0.0, float(self.get_parameter('grasp_surface_to_center_offset').value))
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

    def _target_from_pick_plan(
        self,
        obj: Dict,
        plan: Dict[str, Tuple[float, float, float]],
        object_arm_base_position: Tuple[float, float, float],
    ) -> Dict:
        target = dict(obj)
        target['arm_base_position'] = plan['grasp']
        target['object_arm_base_position'] = object_arm_base_position
        target['pre_grasp_arm_base_position'] = plan['pre_grasp']
        target['lift_arm_base_position'] = plan['lift']
        return target

    def _pick_plan_for_grasp(self, grasp: Tuple[float, float, float]) -> Optional[Dict[str, Tuple[float, float, float]]]:
        if not self._within_workspace(grasp) or self.ik.solve(grasp) is None:
            return None

        approach = float(self.get_parameter('approach_distance').value)
        unit = self._radial_unit(grasp)
        pre_grasp = self._reachable_offset_pose(grasp, unit, approach)
        if self.ik.solve(pre_grasp) is None:
            return None

        for lift_height in self._lift_height_candidates():
            lift_pose = (grasp[0], grasp[1], grasp[2] + lift_height)
            if self._within_workspace(lift_pose) and self.ik.solve(lift_pose) is not None:
                return {
                    'grasp': grasp,
                    'pre_grasp': pre_grasp,
                    'lift': lift_pose,
                }
        return None

    def _pick_plan_rejection_reason(self, grasp: Tuple[float, float, float]) -> str:
        x, y, z = grasp
        radius = math.hypot(x, y)
        min_radius = float(self.get_parameter('reachable_min_radius').value)
        max_radius = float(self.get_parameter('reachable_max_radius').value)
        min_z = float(self.get_parameter('reachable_min_z').value)
        max_z = float(self.get_parameter('reachable_max_z').value)
        if radius < min_radius or radius > max_radius:
            return (
                f'grasp radius {radius:.3f} m outside workspace '
                f'[{min_radius:.3f}, {max_radius:.3f}] for pose '
                f'({x:.3f}, {y:.3f}, {z:.3f})'
            )
        if z < min_z or z > max_z:
            return (
                f'grasp z {z:.3f} m outside workspace '
                f'[{min_z:.3f}, {max_z:.3f}] for pose '
                f'({x:.3f}, {y:.3f}, {z:.3f})'
            )
        if self.ik.solve(grasp) is None:
            return f'IK failed for grasp pose ({x:.3f}, {y:.3f}, {z:.3f})'

        approach = float(self.get_parameter('approach_distance').value)
        pre_grasp = self._reachable_offset_pose(grasp, self._radial_unit(grasp), approach)
        if self.ik.solve(pre_grasp) is None:
            return (
                f'IK failed for pre-grasp pose '
                f'({pre_grasp[0]:.3f}, {pre_grasp[1]:.3f}, {pre_grasp[2]:.3f})'
            )

        lift_reasons = []
        for lift_height in self._lift_height_candidates():
            lift_pose = (x, y, z + lift_height)
            if not self._within_workspace(lift_pose):
                lift_reasons.append(f'lift {lift_height:.3f} outside workspace')
                continue
            if self.ik.solve(lift_pose) is None:
                lift_reasons.append(f'lift {lift_height:.3f} IK failed')
                continue
            return 'pick plan is valid'
        return 'no valid vertical lift pose: ' + ', '.join(lift_reasons)

    def _lift_height_candidates(self) -> Tuple[float, ...]:
        configured = max(0.02, float(self.get_parameter('lift_height').value))
        candidates = [configured, 0.12, 0.10, 0.08, 0.06]
        unique = []
        for value in candidates:
            value = round(max(0.02, min(configured, value)), 3)
            if value not in unique:
                unique.append(value)
        return tuple(unique)

    def _best_effort_pick_plan(
        self,
        grasp: Tuple[float, float, float],
    ) -> Optional[Dict[str, Tuple[float, float, float]]]:
        if not bool(self.get_parameter('allow_best_effort_grasp').value):
            return None

        x, y, z = grasp
        radius = math.hypot(x, y)
        if radius > float(self.get_parameter('best_effort_max_radius').value):
            return None

        min_radius = float(self.get_parameter('reachable_min_radius').value)
        max_radius = float(self.get_parameter('reachable_max_radius').value)
        min_z = float(self.get_parameter('reachable_min_z').value)
        max_z = float(self.get_parameter('reachable_max_z').value)
        yaw = math.atan2(y, x)

        radius_seed = max(min_radius, min(max_radius - 0.02, radius))
        radius_candidates = [
            radius_seed,
            radius_seed - 0.03,
            radius_seed - 0.06,
            radius_seed - 0.09,
            radius_seed - 0.12,
        ]
        z_seed = max(min_z, min(max_z, z))
        z_candidates = [z_seed, z_seed - 0.02, z_seed - 0.04, z_seed - 0.06, z_seed - 0.08]

        for candidate_radius in radius_candidates:
            candidate_radius = max(min_radius, min(max_radius - 0.02, candidate_radius))
            for candidate_z in z_candidates:
                candidate_z = max(min_z, min(max_z, candidate_z))
                candidate = (
                    candidate_radius * math.cos(yaw),
                    candidate_radius * math.sin(yaw),
                    candidate_z,
                )
                plan = self._pick_plan_for_grasp(candidate)
                if plan is not None:
                    return plan
        return None

    def _best_effort_grasp_pose(self, grasp: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        if not bool(self.get_parameter('allow_best_effort_grasp').value):
            return None

        x, y, z = grasp
        radius = math.hypot(x, y)
        if radius > float(self.get_parameter('best_effort_max_radius').value):
            return None

        min_radius = float(self.get_parameter('reachable_min_radius').value)
        max_radius = float(self.get_parameter('reachable_max_radius').value)
        min_z = float(self.get_parameter('reachable_min_z').value)
        max_z = float(self.get_parameter('reachable_max_z').value)

        yaw = math.atan2(y, x)
        clamped_radius = max(min_radius, min(max_radius - 0.02, radius))
        candidate = (
            clamped_radius * math.cos(yaw),
            clamped_radius * math.sin(yaw),
            max(min_z, min(max_z, z)),
        )
        if self.ik.solve(candidate) is None:
            return None
        return candidate

    def _gazebo_entity_name(self, target: Dict) -> Optional[str]:
        by_id = {
            'red_cube': 'k12_red_cube',
            'green_cylinder': 'k12_green_cylinder',
            'blue_rectangular_block': 'k12_blue_block',
        }
        object_id = target.get('id')
        if object_id in by_id:
            return by_id[object_id]

        color = target.get('color')
        if color == 'red':
            return 'k12_red_cube'
        if color == 'green':
            return 'k12_green_cylinder'
        if color == 'blue':
            return 'k12_blue_block'
        return None

    def _attach_object(self, target: Dict) -> None:
        if not bool(self.get_parameter('use_gazebo_grasp_assist').value):
            return
        backend = str(self.get_parameter('grasp_assist_backend').value)
        if backend != 'gazebo_state':
            raise RuntimeError(f'Unsupported grasp assist backend: {backend}')

        entity_name = self._gazebo_entity_name(target)
        if entity_name is None:
            message = f'No Gazebo entity mapping for detected object: {target}'
            if bool(self.get_parameter('require_verified_grasp_attachment').value):
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return
        service_ready = self.entity_state_client.wait_for_service(timeout_sec=5.0)
        held_offset = self._held_offset_from_target(target)
        self._set_held_link_gravity(entity_name, enabled=False)

        with self.hold_lock:
            self.held_entity_name = entity_name
            self.held_object_offset = held_offset
            self.hold_request_pending = None
            self.hold_request_started = 0.0
            self.hold_request_reference_frame = ''
            self.hold_reference_frame_failed = False
            self.hold_request_failure_warned = False
            self.last_hold_position = None
            self.last_hold_orientation = None
        self._hold_gripper_closed()
        for _ in range(3):
            self._update_held_object(force=True)
            self._sleep(0.05)
        if service_ready:
            self.get_logger().info(
                f'Gazebo grasp assist holding {entity_name} at gripper hold offset '
                f'({held_offset[0]:.3f}, {held_offset[1]:.3f}, {held_offset[2]:.3f}).'
            )
        else:
            service_name = self.get_parameter('gazebo_set_entity_state_service').value
            self.get_logger().warn(
                f'{service_name} is not discovered yet; grasp assist will keep trying while lifting.'
            )

        self._verify_grasp_secured(entity_name)

    @staticmethod
    def _held_link_name(entity_name: str) -> str:
        return f'{entity_name}::link'

    def _get_cached_link_properties(self, entity_name: str) -> Optional[GetLinkProperties.Response]:
        link_name = self._held_link_name(entity_name)
        if link_name in self.link_properties_cache:
            return self.link_properties_cache[link_name]
        if not self.get_link_properties_client.wait_for_service(timeout_sec=1.0):
            service_name = self.get_parameter('gazebo_get_link_properties_service').value
            self.get_logger().warn(f'{service_name} is not available; cannot lock held object gravity.')
            return None

        request = GetLinkProperties.Request()
        request.link_name = link_name
        future = self.get_link_properties_client.call_async(request)
        if not self._wait_for_future(future, 1.0):
            self.get_logger().warn(f'Timed out reading link properties for {link_name}.')
            return None
        response = future.result()
        if response is None or not response.success:
            message = response.status_message if response is not None else 'no response'
            self.get_logger().warn(f'Could not read link properties for {link_name}: {message}')
            return None
        self.link_properties_cache[link_name] = response
        return response

    def _set_held_link_gravity(self, entity_name: str, enabled: bool) -> None:
        if not bool(self.get_parameter('disable_gravity_while_holding').value):
            return

        properties = self._get_cached_link_properties(entity_name)
        if properties is None:
            return
        if not self.set_link_properties_client.wait_for_service(timeout_sec=1.0):
            service_name = self.get_parameter('gazebo_set_link_properties_service').value
            self.get_logger().warn(f'{service_name} is not available; cannot change held object gravity.')
            return

        request = SetLinkProperties.Request()
        request.link_name = self._held_link_name(entity_name)
        request.com = properties.com
        request.gravity_mode = enabled
        request.mass = properties.mass
        request.ixx = properties.ixx
        request.ixy = properties.ixy
        request.ixz = properties.ixz
        request.iyy = properties.iyy
        request.iyz = properties.iyz
        request.izz = properties.izz

        future = self.set_link_properties_client.call_async(request)
        if not self._wait_for_future(future, 1.0):
            self.get_logger().warn(f'Timed out setting gravity={enabled} for {request.link_name}.')
            return
        response = future.result()
        if response is None or not response.success:
            message = response.status_message if response is not None else 'no response'
            self.get_logger().warn(f'Could not set gravity={enabled} for {request.link_name}: {message}')
            return

        state = 'enabled' if enabled else 'disabled'
        self.get_logger().info(f'Held object gravity {state} for {request.link_name}.')

    def _held_offset_from_target(self, target: Dict) -> Tuple[float, float, float]:
        fallback = tuple(float(v) for v in self.get_parameter('held_object_offset_end_effector').value)
        if not bool(self.get_parameter('use_live_grasp_offset').value):
            return fallback

        object_position = target.get('arm_base_position', target.get('object_arm_base_position'))
        if object_position is None:
            return fallback

        position = tuple(float(v) for v in object_position)
        offset = self._transform_fixed_point(
            self.get_parameter('target_frame').value,
            position,
            'end_effector',
        )
        if offset is None:
            self.get_logger().warn(
                'Could not compute live object-to-gripper offset; using configured held object offset.'
            )
            return fallback
        offset_distance = distance(offset)
        max_offset = float(self.get_parameter('max_live_grasp_offset_m').value)
        if offset_distance > max_offset:
            self.get_logger().warn(
                f'Live object-to-gripper offset is too large ({offset_distance:.3f} m); '
                'using configured held object offset.'
            )
            return fallback
        return offset

    def _release_held_object(self, final_pose_arm_base: Optional[Tuple[float, float, float]]) -> None:
        with self.hold_lock:
            entity_name = self.held_entity_name
            pending = self.hold_request_pending

        if entity_name is None:
            return
        if final_pose_arm_base is None:
            self._keep_object_held_after_error('release requested without table pose')
            return

        if pending is not None and not pending.done():
            self._wait_for_future(pending, 0.5)

        self._update_held_object(force=True)
        with self.hold_lock:
            pending = self.hold_request_pending
        if pending is not None and not pending.done():
            self._wait_for_future(pending, 0.5)

        held_pose = self._current_held_world_pose()

        position = None
        orientation = None
        if bool(self.get_parameter('snap_release_to_table_center').value):
            position = self._place_table_release_position()
            if held_pose is not None:
                orientation = held_pose[1]
        if position is None:
            if held_pose is not None:
                position, orientation = held_pose
            else:
                with self.hold_lock:
                    position = self.last_hold_position
                    orientation = self.last_hold_orientation
        if position is None:
            z_bias = float(self.get_parameter('release_object_z_bias').value)
            final_pose = (
                final_pose_arm_base[0],
                final_pose_arm_base[1],
                final_pose_arm_base[2] + z_bias,
            )
            position = self._transform_fixed_point(
                self.get_parameter('target_frame').value,
                final_pose,
                self.get_parameter('tracking_frame').value,
            )
        if position is None:
            self._keep_object_held_after_error('release pose transform failed')
            return

        if bool(self.get_parameter('release_upright_orientation').value):
            orientation = (0.0, 0.0, 0.0, 1.0)

        for _ in range(3):
            future = self._send_entity_state(entity_name, position, orientation, reference_frame='world')
            self._wait_for_future(future, 0.5)

        self._set_held_link_gravity(entity_name, enabled=True)

        with self.hold_lock:
            if self.held_entity_name == entity_name:
                self.held_entity_name = None
                self.held_object_offset = None
                self.hold_request_pending = None
                self.hold_request_started = 0.0
                self.hold_request_reference_frame = ''
                self.hold_reference_frame_failed = False
                self.hold_request_failure_warned = False
                self.last_hold_position = None
                self.last_hold_orientation = None
                self.place_table_adjustment_x = 0.0

        self.get_logger().info(f'Released {entity_name}.')

    def _place_table_release_position(self) -> Optional[Tuple[float, float, float]]:
        with self.hold_lock:
            entity_name = self.held_entity_name
        release_height = (
            self._release_height_for_entity(entity_name)
            + float(self.get_parameter('release_object_z_bias').value)
        )
        shelf_pose = self._place_table_pose_for_entity(
            entity_name,
            z=release_height,
            adjustment_x=self.place_table_adjustment_x,
        )
        release_pose = (
            shelf_pose[0],
            shelf_pose[1],
            shelf_pose[2],
        )
        return self._transform_fixed_point(
            self._place_table_param('frame'),
            release_pose,
            self.get_parameter('tracking_frame').value,
        )

    def _place_table_pose_for_entity(
        self,
        entity_name: Optional[str],
        z: Optional[float] = None,
        adjustment_x: float = 0.0,
    ) -> Tuple[float, float, float]:
        shelf = tuple(float(v) for v in self._place_table_param('position'))
        xy_offset = self._place_xy_offset_for_entity(entity_name)
        place_z = float(self._place_table_param('top_z'))
        return (
            shelf[0] + xy_offset[0] + float(adjustment_x),
            shelf[1] + xy_offset[1],
            place_z if z is None else z,
        )

    def _place_slot_reach_adjustments(self) -> Tuple[float, ...]:
        values = list(self.get_parameter('place_slot_reach_adjustments_x').value)
        unique = []
        for value in values:
            value = round(float(value), 3)
            if value not in unique:
                unique.append(value)
        if 0.0 not in unique:
            unique.insert(0, 0.0)
        return tuple(unique)

    def _place_xy_offset_for_entity(self, entity_name: Optional[str]) -> Tuple[float, float]:
        base_offset = self._xy_parameter('place_table_release_xy_offset')
        slot_key = {
            'k12_red_cube': 'red_cube',
            'k12_green_cylinder': 'green_cylinder',
            'k12_blue_block': 'blue_block',
        }.get(entity_name or '')
        if slot_key is None:
            return base_offset

        slot_offset = self._xy_parameter(f'place_table_release_{slot_key}_xy_offset')
        return (
            base_offset[0] + slot_offset[0],
            base_offset[1] + slot_offset[1],
        )

    def _xy_parameter(self, name: str) -> Tuple[float, float]:
        if not self.has_parameter(name):
            return 0.0, 0.0
        values = list(self.get_parameter(name).value)
        if len(values) < 2:
            return 0.0, 0.0
        return float(values[0]), float(values[1])

    def _release_height_for_entity(self, entity_name: Optional[str]) -> float:
        fallback = float(self._place_table_param('release_height'))
        if entity_name is None:
            return fallback

        heights = {
            'k12_red_cube': float(self.get_parameter('release_cube_height').value),
            'k12_green_cylinder': float(self.get_parameter('release_cylinder_height').value),
            'k12_blue_block': float(self.get_parameter('release_block_height').value),
        }
        object_height = heights.get(entity_name)
        if object_height is None:
            return fallback

        shelf_top_z = float(self._place_table_param('top_z'))
        clearance = max(0.0, float(self.get_parameter('release_clearance').value))
        return shelf_top_z + 0.5 * object_height + clearance

    def _holding_object(self) -> bool:
        with self.hold_lock:
            return self.held_entity_name is not None

    def _verify_grasp_secured(self, expected_entity_name: Optional[str] = None) -> None:
        with self.hold_lock:
            held_entity = self.held_entity_name
        if held_entity is None:
            raise RuntimeError('Grasp verification failed: no object is attached to the gripper.')
        if expected_entity_name is not None and held_entity != expected_entity_name:
            raise RuntimeError(
                f'Grasp verification failed: expected {expected_entity_name}, holding {held_entity}.'
            )
        self._hold_gripper_closed()
        self.get_logger().info(f'Grasp verified: {held_entity} is attached and the gripper is commanded closed.')

    def _verify_release_complete(self) -> None:
        if self._holding_object():
            raise RuntimeError('Release verification failed: object is still attached to the gripper.')
        self.get_logger().info('Release verified: gripper is open and no object is attached.')

    def _hold_gripper_closed(self) -> None:
        if self.hold_gripper_republish_suspended:
            return
        if not self._holding_object():
            return
        value = float(self.get_parameter('closed_gripper_width').value)
        self._publish_gripper(value, value, 'hold gripper closed', log=False)

    def _keep_object_held_after_error(self, reason: str) -> None:
        if self._holding_object():
            self._hold_gripper_closed()
            self.get_logger().warn(f'{reason}; keeping object attached until a valid place release.')

    def _current_held_world_pose(
        self,
        offset: Optional[Tuple[float, float, float]] = None,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        if offset is None:
            with self.hold_lock:
                offset = self.held_object_offset
        if offset is None:
            offset = tuple(float(v) for v in self.get_parameter('held_object_offset_end_effector').value)
        return self._transform_fixed_pose(
            'end_effector',
            offset,
            self.get_parameter('tracking_frame').value,
        )

    def _record_hold_request_result(self, future, reference_frame: str) -> None:
        if future is None or not future.done():
            return

        try:
            response = future.result()
        except Exception as exc:
            response = None
            message = str(exc)
        else:
            message = getattr(response, 'status_message', 'success=false') if response is not None else 'no response'

        if response is not None and response.success:
            return

        with self.hold_lock:
            if reference_frame:
                self.hold_reference_frame_failed = True
            already_warned = self.hold_request_failure_warned
            self.hold_request_failure_warned = True

        if not already_warned:
            if reference_frame:
                self.get_logger().warn(
                    f'Gazebo rejected held-object reference frame "{reference_frame}" ({message}); '
                    'falling back to world-frame grasp assist.'
                )
            else:
                self.get_logger().warn(f'Gazebo rejected held-object update: {message}')

    def _update_held_object(self, force: bool = False) -> None:
        with self.hold_lock:
            entity_name = self.held_entity_name
            held_offset = self.held_object_offset
            pending = self.hold_request_pending
            pending_reference_frame = self.hold_request_reference_frame
            reference_frame_failed = self.hold_reference_frame_failed
            pending_age = time.monotonic() - self.hold_request_started if self.hold_request_started else 0.0

        if entity_name is None:
            return
        self._record_hold_request_result(pending, pending_reference_frame)
        with self.hold_lock:
            reference_frame_failed = self.hold_reference_frame_failed

        request_timeout = float(self.get_parameter('held_object_request_timeout_sec').value)
        if not force and pending is not None and not pending.done() and pending_age < request_timeout:
            return
        if not self.entity_state_client.service_is_ready():
            return

        offset = held_offset
        if offset is None:
            offset = tuple(float(v) for v in self.get_parameter('held_object_offset_end_effector').value)
        held_pose = self._current_held_world_pose(offset)
        position = held_pose[0] if held_pose is not None else None
        orientation = None
        if bool(self.get_parameter('held_object_orientation_follows_gripper').value):
            if held_pose is not None:
                orientation = held_pose[1]
        if orientation is None:
            with self.hold_lock:
                orientation = self.last_hold_orientation
        if position is None:
            with self.hold_lock:
                position = self.last_hold_position
            if position is None:
                return

        reference_frame = str(self.get_parameter('held_object_reference_frame').value).strip()
        if reference_frame in {
            'world',
            'map',
            'odom',
            'base_link',
            'arm_base_link',
            'end_effector',
        }:
            reference_frame = ''
        if reference_frame_failed:
            reference_frame = ''
        if reference_frame:
            hold_position = offset
            hold_orientation = (0.0, 0.0, 0.0, 1.0)
            future = self._send_entity_state(
                entity_name,
                hold_position,
                hold_orientation,
                reference_frame=reference_frame,
            )
        else:
            future = self._send_entity_state(entity_name, position, orientation, reference_frame='world')
        wait_sec = max(0.0, float(self.get_parameter('held_object_update_wait_sec').value))
        if wait_sec > 0.0:
            self._wait_for_future(future, wait_sec)
            self._record_hold_request_result(future, reference_frame)
        with self.hold_lock:
            if self.held_entity_name == entity_name:
                self.hold_request_pending = future
                self.hold_request_started = time.monotonic()
                self.hold_request_reference_frame = reference_frame
                self.last_hold_position = position
                self.last_hold_orientation = orientation

    def _send_entity_state(
        self,
        entity_name: str,
        position: Tuple[float, float, float],
        orientation: Optional[Tuple[float, float, float, float]] = None,
        reference_frame: str = 'world',
    ):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = entity_name
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.pose.position.z = position[2]
        if orientation is None:
            state.pose.orientation.w = 1.0
        else:
            state.pose.orientation.x = orientation[0]
            state.pose.orientation.y = orientation[1]
            state.pose.orientation.z = orientation[2]
            state.pose.orientation.w = orientation[3]
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = reference_frame
        req.state = state
        return self.entity_state_client.call_async(req)

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        end_time = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < end_time:
            time.sleep(0.02)
        return future.done()

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
            self.get_logger().debug(f'TF transform failed for {frame_id} fixed target: {exc}')
            return None

        return transformed.point.x, transformed.point.y, transformed.point.z

    def _transform_fixed_pose(
        self,
        frame_id: str,
        position: Tuple[float, float, float],
        target_frame: str,
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.w = 1.0

        try:
            transformed = self.tf_buffer.transform(pose, target_frame, timeout=Duration(seconds=0.25))
        except Exception as exc:
            self.get_logger().debug(f'TF pose transform failed for {frame_id} fixed target: {exc}')
            return None

        return (
            (
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z,
            ),
            (
                transformed.pose.orientation.x,
                transformed.pose.orientation.y,
                transformed.pose.orientation.z,
                transformed.pose.orientation.w,
            ),
        )

    def _within_workspace(self, point: Iterable[float]) -> bool:
        x, y, z = point
        radius = math.hypot(x, y)
        return (
            float(self.get_parameter('reachable_min_radius').value)
            <= radius
            <= float(self.get_parameter('reachable_max_radius').value)
            and float(self.get_parameter('reachable_min_z').value)
            <= z
            <= float(self.get_parameter('reachable_max_z').value)
        )

    def _execute_sequence(self, target: Dict) -> None:
        try:
            self.get_logger().info('Pick-and-place sequence started.')
            self._execute_pick(target)
            self._execute_place()
            self.get_logger().info('Pick-and-place sequence complete.')
            self.completed = True
        except Exception as exc:
            self._keep_object_held_after_error('pick-and-place sequence failed')
            self.get_logger().error(f'Pick-and-place sequence failed: {exc}')
        finally:
            self.executing = False

    def _execute_pick(self, target: Dict) -> None:
        self._stop_base()
        p = tuple(float(v) for v in target['arm_base_position'])
        approach = float(self.get_parameter('approach_distance').value)
        lift = float(self.get_parameter('lift_height').value)
        unit = self._radial_unit(p)
        pre_grasp = tuple(float(v) for v in target.get(
            'pre_grasp_arm_base_position',
            self._reachable_offset_pose(p, unit, approach),
        ))
        grasp = p
        lift_pose = tuple(float(v) for v in target.get(
            'lift_arm_base_position',
            (p[0], p[1], p[2] + lift),
        ))

        self.get_logger().info(
            f'Pick sequence started for {target.get("id", target.get("color", "object"))}: '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f}), '
            f'pre_grasp=({pre_grasp[0]:.3f}, {pre_grasp[1]:.3f}, {pre_grasp[2]:.3f}), '
            f'lift=({lift_pose[0]:.3f}, {lift_pose[1]:.3f}, {lift_pose[2]:.3f}).'
        )
        self._open_gripper()
        self._move_joints(self.get_parameter('home_joints').value, 3.0, 'home')
        self._move_pose(pre_grasp, 3.0, 'pre-grasp')
        self._move_pose(grasp, 2.5, 'grasp')
        self._close_gripper(settle_sec=float(self.get_parameter('grasp_close_settle_sec').value))
        self._attach_object(target)
        self._sleep(0.5)
        self._move_pose(lift_pose, 3.0, 'lift')
        self._move_joints(self.get_parameter('carry_joints').value, 3.0, 'carry')
        self.get_logger().info('Pick sequence complete; object secured.')

    def _execute_place(self) -> None:
        self._stop_base()
        if (
            bool(self.get_parameter('use_gazebo_grasp_assist').value)
            and bool(self.get_parameter('require_verified_grasp_attachment').value)
            and not self._holding_object()
        ):
            raise RuntimeError('Refusing to place: no verified object is attached to the gripper.')
        approach = float(self.get_parameter('approach_distance').value)
        place = self._current_place_pose()
        pre_place_unit = self._radial_unit(place)
        pre_place = self._reachable_offset_pose(place, pre_place_unit, approach)
        place_lift = self._select_place_lift_pose(place, 'pre-place high')

        self.get_logger().info('Place sequence started.')
        self._move_pose(place_lift, 3.0, 'pre-place high')
        self._move_pose(pre_place, 3.0, 'pre-place')
        self._move_pose(place, 2.5, 'place')
        self._sleep(float(self.get_parameter('place_contact_settle_sec').value))
        if bool(self.get_parameter('hold_object_during_place_open').value):
            self.hold_gripper_republish_suspended = True
            try:
                self._open_gripper(allow_while_holding=True)
                self._sleep(float(self.get_parameter('release_settle_before_open_sec').value))
                self._release_held_object(place)
            finally:
                self.hold_gripper_republish_suspended = False
        else:
            self._release_held_object(place)
            self._sleep(float(self.get_parameter('release_settle_before_open_sec').value))
            self._open_gripper()
        self._verify_release_complete()
        self._clear_completed_target()
        self._sleep(0.5)
        self._retreat_straight_up_from_place(place)
        self._move_joints(self.get_parameter('home_joints').value, 3.0, 'return home')
        self.get_logger().info('Place sequence complete.')

    def _clear_completed_target(self) -> None:
        self.latest_payload = None
        self.latest_time = 0.0
        self.tracked_target = None
        self.tracked_time = 0.0
        self._reset_stable_pick_target()

    def _retreat_straight_up_from_place(self, place: Tuple[float, float, float]) -> None:
        target = self._select_place_lift_pose(place, 'place vertical retreat target')

        steps = max(1, int(self.get_parameter('place_retreat_steps').value))
        duration = max(0.2, float(self.get_parameter('place_retreat_step_duration_sec').value))
        start_z = place[2]
        target_z = target[2]
        for step in range(1, steps + 1):
            ratio = step / steps
            z = start_z + (target_z - start_z) * ratio
            self._move_pose((place[0], place[1], z), duration, f'place vertical retreat {step}/{steps}')

    def _select_place_lift_pose(self, place: Tuple[float, float, float], label: str) -> Tuple[float, float, float]:
        heights = (
            float(self.get_parameter('place_lift_height').value),
            float(self.get_parameter('place_retreat_lift_height').value),
            float(self.get_parameter('lift_height').value),
            0.14,
            0.12,
            0.10,
            0.08,
            0.06,
        )
        unique_heights = []
        for height in heights:
            height = round(max(0.02, height), 3)
            if height not in unique_heights:
                unique_heights.append(height)

        max_z = float(self.get_parameter('reachable_max_z').value)
        rejection_reasons = []
        for height in unique_heights:
            pose = (place[0], place[1], min(max_z, place[2] + height))
            if not self._within_workspace(pose):
                rejection_reasons.append(f'{height:.3f}m outside workspace')
                continue
            if self.ik.solve(pose) is None:
                rejection_reasons.append(f'{height:.3f}m IK failed')
                continue
            self.get_logger().info(
                f'Selected {label}: arm_base_pose=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}), '
                f'height_above_place={pose[2] - place[2]:.3f} m.'
            )
            return pose

        raise RuntimeError(
            f'No IK solution for {label} above place pose {place}: '
            + ', '.join(rejection_reasons)
        )

    def _move_pose(self, point_arm_base: Iterable[float], duration_sec: float, label: str) -> None:
        point = tuple(float(v) for v in point_arm_base)
        joints = self.ik.solve(point)
        if joints is None:
            self.get_logger().error(f'IK failed for {label}: arm_base_pose={point}.')
            raise RuntimeError(f'No IK solution for {label}: {point}')
        self.get_logger().info(
            f'IK solved for {label}: arm_base_pose=({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}), '
            f'joints={[round(float(value), 3) for value in joints]}.'
        )
        self._move_joints(joints, duration_sec, label)

    def _recover_after_failed_pick(self) -> None:
        self.get_logger().warn('Recovering from failed pick: opening gripper and returning arm to a safe pose.')
        try:
            self._open_gripper()
        except Exception as exc:
            self.get_logger().warn(f'Could not open gripper during pick recovery: {exc}')
        try:
            self._move_joints(self.get_parameter('home_joints').value, 2.5, 'pick failure recovery home')
        except Exception as exc:
            self.get_logger().warn(f'Could not return arm home during pick recovery: {exc}')

    def _move_joints(self, positions: Iterable[float], duration_sec: float, label: str) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        trajectory.points.append(point)
        self.get_logger().info(f'Moving to {label}: {[round(v, 3) for v in point.positions]}')
        if bool(self.get_parameter('use_follow_joint_trajectory_action').value):
            self._send_joint_trajectory_goal(trajectory, duration_sec, label)
            return

        self._publish_trajectory_fallback(trajectory, duration_sec, label)

    def _publish_trajectory_fallback(
        self,
        trajectory: JointTrajectory,
        duration_sec: float,
        label: str,
    ) -> None:
        if not bool(self.get_parameter('allow_trajectory_topic_fallback').value):
            raise RuntimeError('Trajectory topic fallback is disabled.')
        self.get_logger().warn(f'Using trajectory topic fallback for {label}.')
        self.arm_pub.publish(trajectory)
        self._sleep(duration_sec + 0.3)

    def _send_joint_trajectory_goal(
        self,
        trajectory: JointTrajectory,
        duration_sec: float,
        label: str,
    ) -> None:
        server_timeout = float(self.get_parameter('trajectory_action_server_timeout_sec').value)
        send_timeout = float(self.get_parameter('trajectory_goal_send_timeout_sec').value)
        result_margin = float(self.get_parameter('trajectory_result_margin_sec').value)
        if not self.arm_action_client.wait_for_server(timeout_sec=server_timeout):
            if bool(self.get_parameter('allow_trajectory_topic_fallback').value):
                self.get_logger().warn(
                    'FollowJointTrajectory action unavailable; using explicit topic fallback.'
                )
                self._publish_trajectory_fallback(trajectory, duration_sec, label)
                return
            raise RuntimeError('FollowJointTrajectory action server is not available.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.arm_action_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, send_timeout):
            if bool(self.get_parameter('allow_trajectory_topic_fallback').value):
                self.get_logger().warn(f'Action goal send timed out for {label}; falling back to topic command.')
                self._publish_trajectory_fallback(trajectory, duration_sec, label)
                return
            raise RuntimeError(f'Timed out sending trajectory goal for {label}.')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Trajectory goal for {label} was rejected.')

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, duration_sec + result_margin):
            goal_handle.cancel_goal_async()
            if bool(self.get_parameter('allow_trajectory_topic_fallback').value):
                self.get_logger().warn(f'Action result timed out for {label}; falling back to topic command.')
                self._publish_trajectory_fallback(trajectory, duration_sec, label)
                return
            raise RuntimeError(f'Trajectory goal for {label} timed out.')

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = 'no result' if result is None else str(result.status)
            if bool(self.get_parameter('allow_trajectory_topic_fallback').value):
                self.get_logger().warn(
                    f'Trajectory action failed for {label} with status {status}; falling back to topic command.'
                )
                self._publish_trajectory_fallback(trajectory, duration_sec, label)
                return
            raise RuntimeError(f'Trajectory goal for {label} failed with status {status}.')

    def _on_joint_states(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = position

    def _gripper_reached(self, target: float, tolerance: float = 0.003) -> Tuple[bool, bool]:
        """Returns (left_reached, right_reached). Unknown state (no feedback
        yet) is treated as NOT reached, so we never falsely claim success."""
        left = self.latest_joint_positions.get('left_gripper_joint')
        right = self.latest_joint_positions.get('right_gripper_joint')
        left_ok = left is not None and abs(left - target) <= tolerance
        right_ok = right is not None and abs(right - target) <= tolerance
        return left_ok, right_ok

    def _open_gripper(self, allow_while_holding: bool = False) -> None:
        if self._holding_object() and not allow_while_holding:
            self.get_logger().warn('Ignoring open gripper command while object is held.')
            return
        value = float(self.get_parameter('open_gripper_width').value)
        left_pos = self.latest_joint_positions.get('left_gripper_joint')
        right_pos = self.latest_joint_positions.get('right_gripper_joint')
        self.get_logger().info(
            f'open gripper: target={value:.4f}, '
            f'current left={left_pos}, right={right_pos}'
        )
        self._command_gripper_until_reached(value, value, 'open gripper', max_wait_sec=4.0)
        left_pos = self.latest_joint_positions.get('left_gripper_joint')
        right_pos = self.latest_joint_positions.get('right_gripper_joint')
        self.get_logger().info(
            f'open gripper result: left={left_pos}, right={right_pos}'
        )
        left_ok = left_pos is not None and abs(left_pos - value) <= 0.005
        right_ok = right_pos is not None and abs(right_pos - value) <= 0.005
        if not left_ok or not right_ok:
            self.get_logger().warn(
                'open gripper: retry with repeated commands to unstick finger(s)'
            )
            for _ in range(20):
                self._publish_gripper(value, value, 'open gripper force', log=False)
                self._sleep(0.10)
            left_pos = self.latest_joint_positions.get('left_gripper_joint')
            right_pos = self.latest_joint_positions.get('right_gripper_joint')
            self.get_logger().info(
                f'open gripper final: left={left_pos}, right={right_pos}'
            )

    def _close_gripper(self, settle_sec: float = 0.8) -> None:
        value = float(self.get_parameter('closed_gripper_width').value)
        # Closing against an object is expected to stop early on contact --
        # the object itself is the limit. Use a velocity-based check:
        # both fingers are "closed" when they stop moving (velocity near
        # zero) AND are at or below the target position.
        self._publish_gripper(value, value, 'close gripper')
        end_time = time.monotonic() + max(0.0, settle_sec) + 1.0  # extra buffer
        left_stopped = right_stopped = False
        while rclpy.ok() and time.monotonic() < end_time:
            left_pos = self.latest_joint_positions.get('left_gripper_joint')
            right_pos = self.latest_joint_positions.get('right_gripper_joint')
            # Consider a finger "stopped" if it's at or below target
            # (closed against object or fully closed)
            if left_pos is not None and left_pos <= value + 0.003:
                left_stopped = True
            if right_pos is not None and right_pos <= value + 0.003:
                right_stopped = True
            if left_stopped and right_stopped:
                break
            self._publish_gripper(value, value, 'hold gripper closing', log=False)
            self._sleep(0.10)
        if not (left_stopped and right_stopped):
            stuck = []
            if not left_stopped:
                stuck.append('left')
            if not right_stopped:
                stuck.append('right')
            self.get_logger().warn(
                f'close gripper: {"/".join(stuck)} finger did not reach '
                f'target {value:.4f} within timeout '
                f'(left={self.latest_joint_positions.get("left_gripper_joint")}, '
                f'right={self.latest_joint_positions.get("right_gripper_joint")}).'
            )

    def _command_gripper_until_reached(
        self,
        left_target: float,
        right_target: float,
        label: str,
        max_wait_sec: float = 2.5,
    ) -> None:
        # CHANGED: was a blind fixed-0.8s wait with no feedback at all.
        # Now actively re-publishes at 10 Hz and checks actual joint_states
        # feedback, so a lagging/stuck finger keeps getting nudged (rather
        # than silently assumed successful) for up to max_wait_sec. Logs a
        # clear warning identifying exactly which finger didn't make it,
        # instead of failing silently.
        self._publish_gripper(left_target, right_target, label)
        end_time = time.monotonic() + max_wait_sec
        left_ok = right_ok = False
        while rclpy.ok() and time.monotonic() < end_time:
            left_ok, right_ok = self._gripper_reached(left_target)
            if left_ok and right_ok:
                break
            self._publish_gripper(left_target, right_target, f'{label} (retry)', log=False)
            self._sleep(0.10)
        if not (left_ok and right_ok):
            stuck = []
            if not left_ok:
                stuck.append('left')
            if not right_ok:
                stuck.append('right')
            self.get_logger().warn(
                f'{label}: {"/".join(stuck)} finger did not reach target '
                f'{left_target:.4f} within {max_wait_sec:.1f}s '
                f'(left={self.latest_joint_positions.get("left_gripper_joint")}, '
                f'right={self.latest_joint_positions.get("right_gripper_joint")}).'
            )

    def _publish_gripper(self, left: float, right: float, label: str, log: bool = True) -> None:
        msg = Float64MultiArray()
        msg.data = [left, right]
        if log:
            self.get_logger().info(f'{label}: {msg.data}')
        self.gripper_pub.publish(msg)

    @staticmethod
    def _radial_unit(point: Iterable[float]) -> Tuple[float, float]:
        x, y, _ = point
        radius = max(math.hypot(x, y), 1e-6)
        return x / radius, y / radius

    def _reachable_offset_pose(
        self,
        point: Iterable[float],
        unit: Tuple[float, float],
        approach_distance: float,
    ) -> Tuple[float, float, float]:
        x, y, z = point
        for scale in (1.0, 0.75, 0.5, 0.25, 0.0):
            candidate = (
                x - approach_distance * scale * unit[0],
                y - approach_distance * scale * unit[1],
                z,
            )
            if self.ik.solve(candidate) is not None:
                return candidate
        return (x, y, z)

    def _sleep(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time and rclpy.ok():
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceCommander()
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
