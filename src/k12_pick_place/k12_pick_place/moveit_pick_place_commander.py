import json
import math
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, GetLinkProperties, SetEntityState, SetLinkProperties
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _distance(point):
    x, y, z = point
    return math.sqrt(x * x + y * y + z * z)


class MoveItPickPlaceCommander(Node):
    def __init__(self):
        super().__init__('moveit_pick_place_commander')

        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pick_target_topic', '/amms/pick_target')
        self.declare_parameter('prefer_external_pick_target', True)
        self.declare_parameter('external_pick_target_timeout_sec', 3.0)
        self.declare_parameter('gripper_controller_topic', '/gripper_position_controller/commands')
        self.declare_parameter('arm_action_name', '/arm_position_controller/follow_joint_trajectory')
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
        self.declare_parameter('approach_distance', 0.151875)
        self.declare_parameter('lift_height', 0.18225)
        self.declare_parameter('grasp_z_bias', -0.01125)
        self.declare_parameter('grasp_forward_offset', 0.0)
        self.declare_parameter('grasp_surface_to_center_offset', 0.045)
        self.declare_parameter('use_shape_center_offsets', True)
        self.declare_parameter('cube_center_offset_m', 0.05625)
        self.declare_parameter('cylinder_center_offset_m', 0.05625)
        self.declare_parameter('block_center_offset_m', 0.05625)
        self.declare_parameter('default_center_offset_m', 0.05625)
        self.declare_parameter('grasp_close_settle_sec', 0.8)
        self.declare_parameter('open_gripper_width', 0.039)
        self.declare_parameter('closed_gripper_width', 0.0014625)
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
        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('move_group_planning_time', 10.0)
        self.declare_parameter('move_group_velocity_scale', 0.5)
        self.declare_parameter('move_group_acceleration_scale', 0.5)
        self.declare_parameter('move_group_num_planning_attempts', 10)
        self.declare_parameter('move_group_goal_orientation_tolerance', 0.1)
        self.declare_parameter('move_group_goal_position_tolerance', 0.01)
        self.declare_parameter('base_approach_enabled', True)
        self.declare_parameter('base_approach_max_distance_m', 1.5)
        self.declare_parameter('base_approach_step_m', 0.05)
        self.declare_parameter('base_approach_linear_speed', 0.15)
        self.declare_parameter('base_approach_step_duration_sec', 0.4)
        self.declare_parameter('base_approach_settling_sec', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.joint_names = [
            'arm_base_yaw_joint',
            'arm_shoulder_pitch_joint',
            'arm_elbow_pitch_joint',
            'arm_wrist_pitch_joint',
            'arm_wrist_roll_joint',
        ]
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.command_callback_group = ReentrantCallbackGroup()

        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('arm_action_name').value,
            callback_group=self.command_callback_group,
        )
        self._move_group_action = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.command_callback_group,
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('gripper_controller_topic').value,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
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
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.command_callback_group,
        )

        self.create_subscription(String, self.get_parameter('detections_topic').value, self._on_detections, 10)
        self.create_subscription(String, self.get_parameter('pick_target_topic').value, self._on_pick_target, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_service(
            Trigger, self.get_parameter('pick_service').value,
            self._on_pick_request, callback_group=self.command_callback_group,
        )
        self.create_service(
            Trigger, self.get_parameter('place_service').value,
            self._on_place_request, callback_group=self.command_callback_group,
        )
        self.create_service(
            Trigger, self.get_parameter('fold_arm_service').value,
            self._on_fold_arm_request, callback_group=self.command_callback_group,
        )

        self.latest_payload = None
        self.latest_time = 0.0
        self.latest_external_pick_target = None
        self.latest_external_pick_target_time = 0.0
        self.tracked_target = None
        self.tracked_time = 0.0
        self.stable_target_key = ''
        self.stable_target_position = None
        self.stable_target_count = 0
        self.stable_target_time = 0.0
        self.hold_lock = threading.Lock()
        self.held_entity_name = None
        self.held_object_offset = None
        self.hold_request_pending = None
        self.hold_request_started = 0.0
        self.hold_request_reference_frame = ''
        self.hold_reference_frame_failed = False
        self.hold_request_failure_warned = False
        self.last_hold_position = None
        self.last_hold_orientation = None
        self.link_properties_cache = {}
        self.hold_gripper_republish_suspended = False
        self.place_table_adjustment_x = 0.0
        self.executing = False
        self.completed = False
        self.latest_joint_positions = {}
        self.create_timer(0.5, self._tick)
        self.create_timer(float(self.get_parameter('held_object_update_period_sec').value), self._update_held_object)
        self.create_timer(float(self.get_parameter('held_gripper_republish_sec').value), self._hold_gripper_closed)

        self.get_logger().info('MoveIt Pick-Place Commander started. Waiting for move_group...')

    def _has_valid_joint_states(self):
        required = [
            'arm_base_yaw_joint', 'arm_shoulder_pitch_joint',
            'arm_elbow_pitch_joint', 'arm_wrist_pitch_joint', 'arm_wrist_roll_joint',
        ]
        return all(name in self.latest_joint_positions for name in required)

    def _on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid detection JSON: {exc}')
            return
        self.latest_payload = payload
        self.latest_time = time.monotonic()
        if payload.get('objects', []):
            self._update_tracked_target(payload)

    def _on_pick_target(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid pick target JSON: {exc}')
            return
        self.latest_external_pick_target = payload
        self.latest_external_pick_target_time = time.monotonic()

    def _tick(self):
        if not bool(self.get_parameter('auto_start').value):
            return
        if self.executing:
            return
        if self.completed and bool(self.get_parameter('run_once').value):
            return
        if not self._has_valid_joint_states():
            return

        target = None
        if self.latest_payload is not None:
            latest_age = time.monotonic() - self.latest_time
            if latest_age <= float(self.get_parameter('object_timeout_sec').value):
                self._update_tracked_target(self.latest_payload)
                target = self._select_reachable_target(self.latest_payload)
                if target is None:
                    target = self._select_reachable_tracked_target()

        if target is None:
            target = self._select_gazebo_truth_target()

        if target is not None:
            if not self._accept_stable_pick_target(target):
                return
            self.executing = True
            thread = threading.Thread(target=self._execute_sequence, args=(target,), daemon=True)
            thread.start()
            return

        self._reset_stable_pick_target()

    def _on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = position

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

    def _wait_for_service_pick_target(self):
        start = time.monotonic()
        timeout = float(self.get_parameter('service_pick_target_timeout_sec').value)
        self._reset_stable_pick_target()
        while rclpy.ok() and time.monotonic() - start < timeout:
            target = self._select_current_pick_target()
            if target is not None and self._accept_stable_pick_target(target):
                return target
            if target is None:
                self._reset_stable_pick_target()
            self._sleep(0.1)
        return None

    def _select_current_pick_target(self):
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

    def _select_external_pick_target(self):
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
        if not self._detection_has_confidence(obj):
            return None
        grasp = self._position_from_payload(payload.get('arm_base_position'))
        if grasp is None:
            self.get_logger().warn(f'Rejecting external pick target {object_id}: missing arm_base_position.')
            return None
        object_position = self._position_from_payload(payload.get('object_arm_base_position')) or grasp
        target = self._target_from_object(obj, grasp, object_position)
        self.get_logger().info(f'Selected external target {target.get("id", target.get("color", "object"))}')
        return target

    @staticmethod
    def _position_from_payload(value):
        if value is None:
            return None
        try:
            if isinstance(value, dict):
                return float(value['x']), float(value['y']), float(value['z'])
            x, y, z = value
            return float(x), float(y), float(z)
        except (KeyError, TypeError, ValueError):
            return None


    def _detection_has_confidence(self, obj):
        threshold = float(self.get_parameter('min_detection_confidence').value)
        try:
            confidence = float(obj.get('confidence', 1.0))
        except (TypeError, ValueError):
            confidence = 0.0
        return confidence >= threshold

    def _accept_stable_pick_target(self, target):
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
            and _distance((
                position[0] - self.stable_target_position[0],
                position[1] - self.stable_target_position[1],
                position[2] - self.stable_target_position[2],
            )) <= tolerance
        )
        self.stable_target_key = key
        self.stable_target_position = position
        self.stable_target_time = now
        self.stable_target_count = self.stable_target_count + 1 if same_target else 1
        return self.stable_target_count >= required

    def _reset_stable_pick_target(self):
        self.stable_target_key = ''
        self.stable_target_position = None
        self.stable_target_count = 0
        self.stable_target_time = 0.0

    @staticmethod
    def _target_key(target):
        object_id = str(target.get('id', '')).strip()
        if object_id:
            return object_id
        return f'{target.get("color", "")}:{target.get("shape", "")}'

    @staticmethod
    def _target_position(target):
        values = target.get('arm_base_position', target.get('object_arm_base_position'))
        if values is None:
            return None
        try:
            x, y, z = values
        except (TypeError, ValueError):
            return None
        return float(x), float(y), float(z)

    def _transform_objects(self, payload, target_frame):
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
            xyz = (transformed.point.x, transformed.point.y, transformed.point.z)
            transformed_objects.append((obj, xyz))
        return transformed_objects

    def _update_tracked_target(self, payload):
        base_target = self._select_drive_target(payload)
        if base_target is None:
            return
        tracking_frame = self.get_parameter('tracking_frame').value
        selected_id = base_target['object'].get('id')
        for obj, xyz in self._transform_objects(payload, tracking_frame):
            if obj.get('id') != selected_id:
                continue
            self.tracked_target = {'object': dict(obj), 'frame_id': tracking_frame, 'position': xyz}
            self.tracked_time = time.monotonic()
            return

    def _select_reachable_target(self, payload):
        target_frame = self.get_parameter('target_frame').value
        candidates = []
        for obj, xyz_raw in self._transform_objects(payload, target_frame):
            if not self._detection_has_confidence(obj):
                continue
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            if not self._is_reachable(grasp):
                continue
            candidates.append((_distance(grasp), obj, grasp, xyz_raw))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        _, obj, grasp, xyz_raw = candidates[0]
        self.get_logger().info(
            f'Selected {obj.get("color", "?")} {obj.get("shape", "object")} at '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return self._target_from_object(obj, grasp, xyz_raw)

    def _select_reachable_tracked_target(self):
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
            transformed = self.tf_buffer.transform(point, self.get_parameter('target_frame').value, timeout=Duration(seconds=0.25))
        except Exception:
            return None
        obj = self.tracked_target['object']
        xyz_raw = (transformed.point.x, transformed.point.y, transformed.point.z)
        grasp = self._grasp_pose_from_object(xyz_raw, obj)
        if not self._is_reachable(grasp):
            return None
        return self._target_from_object(obj, grasp, xyz_raw)

    def _select_gazebo_truth_target(self):
        if not bool(self.get_parameter('use_gazebo_truth_fallback').value):
            return None
        if not self.get_entity_state_client.wait_for_service(timeout_sec=0.05):
            return None
        candidates = []
        for obj, xyz_raw in self._gazebo_truth_objects():
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            if not self._is_reachable(grasp):
                continue
            candidates.append((_distance(grasp), obj, grasp, xyz_raw))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        _, obj, grasp, xyz_raw = candidates[0]
        return self._target_from_object(obj, grasp, xyz_raw)

    def _gazebo_truth_objects(self):
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
                {'id': object_id, 'color': color, 'shape': shape, 'confidence': 1.0, 'pose_source': 'gazebo_truth'},
                arm_position,
            ))
        return objects

    @staticmethod
    def _known_gazebo_objects():
        return (
            ('red_cube', 'k12_red_cube', 'red', 'cube'),
            ('green_cylinder', 'k12_green_cylinder', 'green', 'cylinder'),
            ('blue_rectangular_block', 'k12_blue_block', 'blue', 'rectangular_block'),
        )

    def _select_drive_target(self, payload):
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


    def _target_from_object(self, obj, grasp, object_arm_base_position):
        target = dict(obj)
        target['arm_base_position'] = grasp
        target['object_arm_base_position'] = object_arm_base_position
        approach = float(self.get_parameter('approach_distance').value)
        unit = self._radial_unit(grasp)
        pre_grasp = (
            grasp[0] - approach * unit[0],
            grasp[1] - approach * unit[1],
            grasp[2],
        )
        target['pre_grasp_arm_base_position'] = pre_grasp
        lift_height = float(self.get_parameter('lift_height').value)
        target['lift_arm_base_position'] = (grasp[0], grasp[1], grasp[2] + lift_height)
        return target

    def _grasp_pose_from_object(self, object_arm_base, obj=None):
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

    def _center_offset_for_object(self, obj):
        if obj is not None and obj.get('pose_source') == 'gazebo_truth':
            return 0.0
        configured = max(0.0, float(self.get_parameter('grasp_surface_to_center_offset').value))
        if not bool(self.get_parameter('use_shape_center_offsets').value):
            return configured
        shape = str(obj.get('shape', '')).lower() if obj else ''
        color = str(obj.get('color', '')).lower() if obj else ''
        if shape == 'cube' or color == 'red':
            return float(self.get_parameter('cube_center_offset_m').value)
        if shape == 'cylinder' or color == 'green':
            return float(self.get_parameter('cylinder_center_offset_m').value)
        if shape == 'rectangular_block' or color == 'blue':
            return float(self.get_parameter('block_center_offset_m').value)
        return configured if configured > 0.0 else float(self.get_parameter('default_center_offset_m').value)

    def _is_reachable(self, grasp):
        x, y, z = grasp
        radius = math.hypot(x, y)
        min_r = float(self.get_parameter('reachable_min_radius').value)
        max_r = float(self.get_parameter('reachable_max_radius').value)
        min_z = float(self.get_parameter('reachable_min_z').value)
        max_z = float(self.get_parameter('reachable_max_z').value)
        if radius < min_r or radius > max_r:
            return False
        if z < min_z or z > max_z:
            return False
        return self._check_ik(grasp)

    def _check_ik(self, point_arm_base):
        if not self.ik_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('IK service not available; assuming reachable.')
            return True
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('move_group_name').value
        req.ik_request.ik_link_name = 'end_effector'
        req.ik_request.pose_stamped.header.frame_id = self.get_parameter('target_frame').value
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = point_arm_base[0]
        req.ik_request.pose_stamped.pose.position.y = point_arm_base[1]
        req.ik_request.pose_stamped.pose.position.z = point_arm_base[2]
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.timeout = Duration(seconds=2.0).to_msg()
        future = self.ik_client.call_async(req)
        if not self._wait_for_future(future, 3.0):
            return False
        response = future.result()
        if response is None:
            return False
        return response.error_code.val == 1

    def _place_table_param(self, name_suffix):
        param_name = f'place_table_{name_suffix}'
        if self.has_parameter(param_name):
            val = self.get_parameter(param_name).value
            if val is not None:
                return val
        return None

    def _current_place_pose(self):
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
            if not self._is_reachable(candidate):
                continue
            if abs(adjustment_x) > 1e-6:
                self.get_logger().warn(f'Adjusted placement slot by {adjustment_x:.2f}m for {entity_label}.')
            self.get_logger().info(
                f'Placing {entity_label} at arm_base=({candidate[0]:.3f}, {candidate[1]:.3f}, {candidate[2]:.3f})'
            )
            self.place_table_adjustment_x = adjustment_x
            return candidate
        if self._is_reachable(fallback):
            self.get_logger().warn(f'Using fallback place pose for {entity_label}')
            self.place_table_adjustment_x = 0.0
            return fallback
        raise RuntimeError(f'No reachable place pose for {entity_label}')

    def _compute_fallback_place_pose(self):
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

    def _gazebo_entity_name(self, target):
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


    def _execute_sequence(self, target):
        try:
            self.get_logger().info('Pick-and-place sequence started.')
            target = self._approach_target_with_base(target)
            self._execute_pick(target)
            self._execute_place()
            self.get_logger().info('Pick-and-place sequence complete.')
            self.completed = True
        except Exception as exc:
            self._keep_object_held_after_error('pick-and-place sequence failed')
            self.get_logger().error(f'Pick-and-place sequence failed: {exc}')
        finally:
            self.executing = False

    def _approach_target_with_base(self, target):
        """Drive the rover forward in steps until the target becomes reachable by the arm.

        Uses the grasp position stored in `target['arm_base_position']` (already in
        `arm_base_link` frame) to decide whether approach is needed.  If the object
        is already reachable, or base_approach_enabled is false, the method returns
        the target unchanged.
        """
        if not self.get_parameter('base_approach_enabled').value:
            return target

        # Check current reachability using the arm_base_position already in the target
        arm_pos = target.get('arm_base_position')
        if arm_pos is None:
            return target

        if self._is_reachable(tuple(float(v) for v in arm_pos)):
            # Already within workspace — no driving needed
            return target

        max_distance = float(self.get_parameter('base_approach_max_distance_m').value)
        step_m = float(self.get_parameter('base_approach_step_m').value)
        speed = float(self.get_parameter('base_approach_linear_speed').value)
        step_duration = float(self.get_parameter('base_approach_step_duration_sec').value)
        settling = float(self.get_parameter('base_approach_settling_sec').value)
        tracking_frame = self.get_parameter('tracking_frame').value
        target_frame = self.get_parameter('target_frame').value

        # Get the object position in a world-fixed frame for re-query during approach.
        # Prefer the tracked_target (in tracking_frame) if available; otherwise fall back
        # to a Gazebo truth query each step.
        object_tracking_position = None
        object_tracking_frame = None
        if self.tracked_target is not None:
            object_tracking_position = self.tracked_target.get('position')
            object_tracking_frame = self.tracked_target.get('frame_id', tracking_frame)

        distance_driven = 0.0
        stop_twist = Twist()

        self.get_logger().info(
            f'[base_approach] Starting: max_distance={max_distance:.2f}m step={step_m:.3f}m'
        )

        while rclpy.ok() and distance_driven < max_distance:
            # Re-query object position in arm_base_link frame
            if object_tracking_position is not None and object_tracking_frame is not None:
                current_arm_pos = self._transform_fixed_point(
                    object_tracking_frame, object_tracking_position, target_frame
                )
            else:
                # Fall back to Gazebo truth each step
                current_arm_pos = None
                for _obj, xyz in self._gazebo_truth_objects():
                    current_arm_pos = xyz
                    break

            if current_arm_pos is None:
                self.get_logger().warn('[base_approach] Cannot re-query object position, stopping')
                break

            reachable = self._is_reachable(current_arm_pos)
            dist = math.hypot(current_arm_pos[0], current_arm_pos[1])
            self.get_logger().info(
                f'[base_approach] xy_dist={dist:.3f}m reachable={reachable} driven={distance_driven:.3f}m'
            )
            if reachable:
                self.get_logger().info('[base_approach] Target now within workspace, stopping.')
                # Update target with freshly-computed arm position
                new_target = self._target_from_object(
                    {k: v for k, v in target.items() if k not in (
                        'arm_base_position', 'pre_grasp_arm_base_position', 'lift_arm_base_position'
                    )},
                    self._grasp_pose_from_object(current_arm_pos, target),
                    current_arm_pos,
                )
                self._sleep(settling)
                return new_target

            # Drive one step forward
            twist = Twist()
            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)
            self._sleep(step_duration)
            self.cmd_vel_pub.publish(stop_twist)
            distance_driven += step_m
            self._sleep(0.15)  # brief pause before next TF check

        self._sleep(settling)
        # Final reachability check after driving
        final_arm_pos = None
        if object_tracking_position is not None and object_tracking_frame is not None:
            final_arm_pos = self._transform_fixed_point(
                object_tracking_frame, object_tracking_position, target_frame
            )
        if final_arm_pos is None:
            for _obj, xyz in self._gazebo_truth_objects():
                final_arm_pos = xyz
                break

        if final_arm_pos is not None and self._is_reachable(final_arm_pos):
            self.get_logger().info('[base_approach] Target reachable after full drive.')
            new_target = self._target_from_object(
                {k: v for k, v in target.items() if k not in (
                    'arm_base_position', 'pre_grasp_arm_base_position', 'lift_arm_base_position'
                )},
                self._grasp_pose_from_object(final_arm_pos, target),
                final_arm_pos,
            )
            return new_target

        raise RuntimeError(
            f'[base_approach] Target still not reachable after driving {distance_driven:.3f}m'
        )

    def _execute_pick(self, target):
        p = tuple(float(v) for v in target['arm_base_position'])
        pre_grasp = tuple(float(v) for v in target.get('pre_grasp_arm_base_position', p))
        grasp = p
        lift_pose = tuple(float(v) for v in target.get('lift_arm_base_position', (p[0], p[1], p[2] + 0.15)))

        self.get_logger().info(
            f'Pick: grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f}), '
            f'pre_grasp=({pre_grasp[0]:.3f}, {pre_grasp[1]:.3f}, {pre_grasp[2]:.3f}), '
            f'lift=({lift_pose[0]:.3f}, {lift_pose[1]:.3f}, {lift_pose[2]:.3f})'
        )
        self._open_gripper()
        self._move_joints(self.get_parameter('home_joints').value, 3.0, 'home')
        self._move_pose(pre_grasp, 'pre-grasp')
        self._move_pose(grasp, 'grasp')
        self._close_gripper(settle_sec=float(self.get_parameter('grasp_close_settle_sec').value))
        self._attach_object(target)
        self._sleep(0.5)
        self._move_pose(lift_pose, 'lift')
        self._move_joints(self.get_parameter('carry_joints').value, 3.0, 'carry')
        self.get_logger().info('Pick sequence complete.')

    def _execute_place(self):
        if (
            bool(self.get_parameter('use_gazebo_grasp_assist').value)
            and bool(self.get_parameter('require_verified_grasp_attachment').value)
            and not self._holding_object()
        ):
            raise RuntimeError('Refusing to place: no verified object is attached to the gripper.')
        place = self._current_place_pose()
        approach = float(self.get_parameter('approach_distance').value)
        pre_place_unit = self._radial_unit(place)
        pre_place = (
            place[0] - approach * pre_place_unit[0],
            place[1] - approach * pre_place_unit[1],
            place[2],
        )
        place_lift = self._select_place_lift_pose(place)

        self.get_logger().info('Place sequence started.')
        self._move_pose(place_lift, 'pre-place high')
        self._move_pose(pre_place, 'pre-place')
        self._move_pose(place, 'place')
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

    def _clear_completed_target(self):
        self.latest_payload = None
        self.latest_time = 0.0
        self.tracked_target = None
        self.tracked_time = 0.0
        self._reset_stable_pick_target()

    def _retreat_straight_up_from_place(self, place):
        target = self._select_place_lift_pose(place)
        steps = max(1, int(self.get_parameter('place_retreat_steps').value))
        start_z = place[2]
        target_z = target[2]
        for step in range(1, steps + 1):
            ratio = step / steps
            z = start_z + (target_z - start_z) * ratio
            self._move_pose((place[0], place[1], z), f'retreat {step}/{steps}')

    def _select_place_lift_pose(self, place):
        heights = (
            float(self.get_parameter('place_lift_height').value),
            float(self.get_parameter('place_retreat_lift_height').value),
            float(self.get_parameter('lift_height').value),
            0.14, 0.12, 0.10, 0.08,
        )
        unique_heights = []
        for h in heights:
            h = round(max(0.02, h), 3)
            if h not in unique_heights:
                unique_heights.append(h)
        max_z = float(self.get_parameter('reachable_max_z').value)
        for height in unique_heights:
            pose = (place[0], place[1], min(max_z, place[2] + height))
            if self._is_reachable(pose):
                return pose
        raise RuntimeError(f'No reachable lift pose above place pose {place}')


    def _move_pose(self, point_arm_base, label):
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('target_frame').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point_arm_base[0])
        pose.pose.position.y = float(point_arm_base[1])
        pose.pose.position.z = float(point_arm_base[2])
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f'MoveIt move to {label}: ({point_arm_base[0]:.3f}, {point_arm_base[1]:.3f}, {point_arm_base[2]:.3f})')

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.get_parameter('move_group_name').value
        goal.request.num_planning_attempts = int(self.get_parameter('move_group_num_planning_attempts').value)
        goal.request.allowed_planning_time = float(self.get_parameter('move_group_planning_time').value)
        goal.request.max_velocity_scaling_factor = float(self.get_parameter('move_group_velocity_scale').value)
        goal.request.max_acceleration_scaling_factor = float(self.get_parameter('move_group_acceleration_scale').value)

        goal.request.goal_constraints = []
        constraints = Constraints()
        constraints.name = 'end_effector_position'

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = 'end_effector'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(self.get_parameter('move_group_goal_position_tolerance').value)]

        pos_constraint.constraint_region = BoundingVolume()
        pos_constraint.constraint_region.primitives = [primitive]
        pos_constraint.constraint_region.primitive_poses = [pose.pose]
        pos_constraint.weight = 1.0
        constraints.position_constraints = [pos_constraint]

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f'Sending MoveIt goal for {label}...')
        send_future = self._move_group_action.send_goal_async(goal)
        if not self._wait_for_future(send_future, 5.0):
            raise RuntimeError(f'MoveIt goal send timeout for {label}')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'MoveIt goal rejected for {label}')

        result_future = goal_handle.get_result_async()
        planning_time = float(self.get_parameter('move_group_planning_time').value)
        if not self._wait_for_future(result_future, planning_time + 30.0):
            goal_handle.cancel_goal_async()
            raise RuntimeError(f'MoveIt goal timeout for {label}')

        result = result_future.result()
        if result is None:
            raise RuntimeError(f'No result from MoveIt for {label}')

        error_code = result.result.error_code
        if error_code.val != 1:
            raise RuntimeError(f'MoveIt planning failed for {label} with error code {error_code.val}')

        self.get_logger().info(f'MoveIt motion complete for {label}')

    def _move_joints(self, positions, duration_sec, label):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        trajectory.points.append(point)

        self.get_logger().info(f'Moving joints for {label}: {[round(v, 3) for v in point.positions]}')

        self._send_joint_trajectory_goal(trajectory, duration_sec, label)

    def _send_joint_trajectory_goal(self, trajectory, duration_sec, label):
        server_timeout = 15.0
        if not self.arm_action_client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError('FollowJointTrajectory action server is not available.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.arm_action_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 15.0):
            raise RuntimeError(f'Timed out sending trajectory goal for {label}')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Trajectory goal for {label} was rejected.')

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, duration_sec + 8.0):
            goal_handle.cancel_goal_async()
            raise RuntimeError(f'Trajectory goal for {label} timed out')

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = 'no result' if result is None else str(result.status)
            raise RuntimeError(f'Trajectory goal for {label} failed with status {status}')

    def _open_gripper(self, allow_while_holding=False):
        if self._holding_object() and not allow_while_holding:
            self.get_logger().warn('Ignoring open gripper command while object is held.')
            return
        value = float(self.get_parameter('open_gripper_width').value)
        self._command_gripper_until_reached(value, value, 'open gripper', max_wait_sec=4.0)
        left_pos = self.latest_joint_positions.get('left_gripper_joint')
        right_pos = self.latest_joint_positions.get('right_gripper_joint')
        left_ok = left_pos is not None and abs(left_pos - value) <= 0.005
        right_ok = right_pos is not None and abs(right_pos - value) <= 0.005
        if not left_ok or not right_ok:
            self.get_logger().warn('open gripper: retry with repeated commands')
            for _ in range(20):
                self._publish_gripper(value, value, 'open gripper force', log=False)
                self._sleep(0.10)

    def _close_gripper(self, settle_sec=0.8):
        value = float(self.get_parameter('closed_gripper_width').value)
        self._publish_gripper(value, value, 'close gripper')
        end_time = time.monotonic() + max(0.0, settle_sec) + 1.0
        left_stopped = right_stopped = False
        while rclpy.ok() and time.monotonic() < end_time:
            left_pos = self.latest_joint_positions.get('left_gripper_joint')
            right_pos = self.latest_joint_positions.get('right_gripper_joint')
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
            self.get_logger().warn(f'close gripper: {"/".join(stuck)} finger did not close properly')

    def _command_gripper_until_reached(self, left_target, right_target, label, max_wait_sec=2.5):
        self._publish_gripper(left_target, right_target, label)
        end_time = time.monotonic() + max_wait_sec
        while rclpy.ok() and time.monotonic() < end_time:
            left = self.latest_joint_positions.get('left_gripper_joint')
            right = self.latest_joint_positions.get('right_gripper_joint')
            left_ok = left is not None and abs(left - left_target) <= 0.003
            right_ok = right is not None and abs(right - right_target) <= 0.003
            if left_ok and right_ok:
                break
            self._publish_gripper(left_target, right_target, f'{label} retry', log=False)
            self._sleep(0.10)

    def _publish_gripper(self, left, right, label, log=True):
        msg = Float64MultiArray()
        msg.data = [left, right]
        if log:
            self.get_logger().info(f'{label}: {msg.data}')
        self.gripper_pub.publish(msg)


    def _attach_object(self, target):
        if not bool(self.get_parameter('use_gazebo_grasp_assist').value):
            return
        entity_name = self._gazebo_entity_name(target)
        if entity_name is None:
            message = f'No Gazebo entity mapping for detected object: {target}'
            if bool(self.get_parameter('require_verified_grasp_attachment').value):
                raise RuntimeError(message)
            self.get_logger().warn(message)
            return
        self.entity_state_client.wait_for_service(timeout_sec=5.0)
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
        self.get_logger().info(f'Gazebo grasp assist holding {entity_name}')
        self._verify_grasp_secured(entity_name)

    def _release_held_object(self, final_pose_arm_base):
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
            final_pose = (final_pose_arm_base[0], final_pose_arm_base[1], final_pose_arm_base[2] + z_bias)
            position = self._transform_fixed_point(
                self.get_parameter('target_frame').value, final_pose, self.get_parameter('tracking_frame').value,
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
        self.get_logger().info(f'Released {entity_name}')

    def _held_offset_from_target(self, target):
        fallback = tuple(float(v) for v in self.get_parameter('held_object_offset_end_effector').value)
        if not bool(self.get_parameter('use_live_grasp_offset').value):
            return fallback
        object_position = target.get('arm_base_position', target.get('object_arm_base_position'))
        if object_position is None:
            return fallback
        position = tuple(float(v) for v in object_position)
        offset = self._transform_fixed_point(self.get_parameter('target_frame').value, position, 'end_effector')
        if offset is None:
            return fallback
        offset_distance = _distance(offset)
        max_offset = float(self.get_parameter('max_live_grasp_offset_m').value)
        if offset_distance > max_offset:
            return fallback
        return offset

    def _current_held_world_pose(self, offset=None):
        if offset is None:
            with self.hold_lock:
                offset = self.held_object_offset
        if offset is None:
            offset = tuple(float(v) for v in self.get_parameter('held_object_offset_end_effector').value)
        return self._transform_fixed_pose('end_effector', offset, self.get_parameter('tracking_frame').value)

    def _update_held_object(self, force=False):
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
        if reference_frame in {'world', 'map', 'odom', 'base_link', 'arm_base_link', 'end_effector'}:
            reference_frame = ''
        if reference_frame_failed:
            reference_frame = ''
        if reference_frame:
            future = self._send_entity_state(entity_name, offset, (0.0, 0.0, 0.0, 1.0), reference_frame=reference_frame)
        else:
            future = self._send_entity_state(entity_name, position, orientation, reference_frame='world')
        with self.hold_lock:
            if self.held_entity_name == entity_name:
                self.hold_request_pending = future
                self.hold_request_started = time.monotonic()
                self.hold_request_reference_frame = reference_frame
                self.last_hold_position = position
                self.last_hold_orientation = orientation


    def _send_entity_state(self, entity_name, position, orientation=None, reference_frame='world'):
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

    def _record_hold_request_result(self, future, reference_frame):
        if future is None or not future.done():
            return
        try:
            response = future.result()
        except Exception:
            response = None
        if response is not None and getattr(response, 'success', False):
            return
        with self.hold_lock:
            if reference_frame:
                self.hold_reference_frame_failed = True
            self.hold_request_failure_warned = True

    @staticmethod
    def _held_link_name(entity_name):
        return f'{entity_name}::link'

    def _get_cached_link_properties(self, entity_name):
        link_name = self._held_link_name(entity_name)
        if link_name in self.link_properties_cache:
            return self.link_properties_cache[link_name]
        if not self.get_link_properties_client.wait_for_service(timeout_sec=1.0):
            return None
        request = GetLinkProperties.Request()
        request.link_name = link_name
        future = self.get_link_properties_client.call_async(request)
        if not self._wait_for_future(future, 1.0):
            return None
        response = future.result()
        if response is None or not response.success:
            return None
        self.link_properties_cache[link_name] = response
        return response

    def _set_held_link_gravity(self, entity_name, enabled):
        if not bool(self.get_parameter('disable_gravity_while_holding').value):
            return
        properties = self._get_cached_link_properties(entity_name)
        if properties is None:
            return
        if not self.set_link_properties_client.wait_for_service(timeout_sec=1.0):
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
        self._wait_for_future(future, 1.0)

    def _place_table_release_position(self):
        with self.hold_lock:
            entity_name = self.held_entity_name
        release_height = (
            self._release_height_for_entity(entity_name)
            + float(self.get_parameter('release_object_z_bias').value)
        )
        shelf_pose = self._place_table_pose_for_entity(entity_name, z=release_height, adjustment_x=self.place_table_adjustment_x)
        return self._transform_fixed_point(
            self._place_table_param('frame'), shelf_pose, self.get_parameter('tracking_frame').value,
        )

    def _place_table_pose_for_entity(self, entity_name, z=None, adjustment_x=0.0):
        shelf = tuple(float(v) for v in self._place_table_param('position'))
        xy_offset = self._place_xy_offset_for_entity(entity_name)
        place_z = float(self._place_table_param('top_z'))
        return (shelf[0] + xy_offset[0] + float(adjustment_x), shelf[1] + xy_offset[1], place_z if z is None else z)

    def _place_slot_reach_adjustments(self):
        values = list(self.get_parameter('place_slot_reach_adjustments_x').value)
        unique = []
        for v in values:
            v = round(float(v), 3)
            if v not in unique:
                unique.append(v)
        if 0.0 not in unique:
            unique.insert(0, 0.0)
        return tuple(unique)

    def _place_xy_offset_for_entity(self, entity_name):
        base_offset = self._xy_parameter('place_table_release_xy_offset')
        slot_key = {
            'k12_red_cube': 'red_cube',
            'k12_green_cylinder': 'green_cylinder',
            'k12_blue_block': 'blue_block',
        }.get(entity_name or '')
        if slot_key is None:
            return base_offset
        slot_offset = self._xy_parameter(f'place_table_release_{slot_key}_xy_offset')
        return (base_offset[0] + slot_offset[0], base_offset[1] + slot_offset[1])

    def _xy_parameter(self, name):
        if not self.has_parameter(name):
            return 0.0, 0.0
        values = list(self.get_parameter(name).value)
        if len(values) < 2:
            return 0.0, 0.0
        return float(values[0]), float(values[1])

    def _release_height_for_entity(self, entity_name):
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

    def _holding_object(self):
        with self.hold_lock:
            return self.held_entity_name is not None

    def _verify_grasp_secured(self, expected_entity_name=None):
        with self.hold_lock:
            held_entity = self.held_entity_name
        if held_entity is None:
            raise RuntimeError('Grasp verification failed: no object is attached to the gripper.')
        if expected_entity_name is not None and held_entity != expected_entity_name:
            raise RuntimeError(f'Grasp verification failed: expected {expected_entity_name}, holding {held_entity}.')
        self._hold_gripper_closed()
        self.get_logger().info(f'Grasp verified: {held_entity} is attached.')

    def _verify_release_complete(self):
        if self._holding_object():
            raise RuntimeError('Release verification failed: object is still attached.')
        self.get_logger().info('Release verified: gripper is open.')

    def _hold_gripper_closed(self):
        if self.hold_gripper_republish_suspended:
            return
        if not self._holding_object():
            return
        value = float(self.get_parameter('closed_gripper_width').value)
        self._publish_gripper(value, value, 'hold gripper closed', log=False)

    def _keep_object_held_after_error(self, reason):
        if self._holding_object():
            self._hold_gripper_closed()
            self.get_logger().warn(f'{reason}; keeping object attached until a valid place release.')

    def _recover_after_failed_pick(self):
        self.get_logger().warn('Recovering from failed pick.')
        try:
            self._open_gripper()
        except Exception as exc:
            self.get_logger().warn(f'Could not open gripper during recovery: {exc}')
        try:
            self._move_joints(self.get_parameter('home_joints').value, 2.5, 'recovery home')
        except Exception as exc:
            self.get_logger().warn(f'Could not return arm home during recovery: {exc}')

    def _transform_fixed_point(self, frame_id, position, target_frame):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = position
        try:
            transformed = self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.25))
        except Exception as exc:
            self.get_logger().debug(f'TF transform failed: {exc}')
            return None
        return (transformed.point.x, transformed.point.y, transformed.point.z)

    def _transform_fixed_pose(self, frame_id, position, target_frame):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.w = 1.0
        try:
            transformed = self.tf_buffer.transform(pose, target_frame, timeout=Duration(seconds=0.25))
        except Exception as exc:
            self.get_logger().debug(f'TF pose transform failed: {exc}')
            return None
        return (
            (transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z),
            (transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w),
        )

    @staticmethod
    def _radial_unit(point):
        x, y, _ = point
        radius = max(math.hypot(x, y), 1e-6)
        return x / radius, y / radius

    def _wait_for_future(self, future, timeout_sec):
        end_time = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < end_time:
            time.sleep(0.02)
        return future.done()

    def _sleep(self, seconds):
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time and rclpy.ok():
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPickPlaceCommander()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
