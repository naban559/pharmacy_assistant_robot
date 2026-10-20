import json
import math
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, GetLinkProperties, SetEntityState, SetLinkProperties
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    JointConstraint,
    RobotState,
    CollisionObject,
    AttachedCollisionObject,
    OrientationConstraint,
    PositionConstraint,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity, ApplyPlanningScene, GetPlanningScene
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

from k12_pick_place.ik import PlanarArmIK, distance
from k12_pick_place.gazebo_observer import GazeboObserver, resolve_entities, ENTITY_MAP, pose_matrix, matrix_quaternion, transform_matrix


def _distance(point):
    x, y, z = point
    return math.sqrt(x * x + y * y + z * z)


class MoveItPickPlaceCommander(Node):
    def __init__(self):
        super().__init__('moveit_pick_place_commander')

        self.declare_parameter('detections_topic', '/amms/detections')
        self.declare_parameter('pick_target_topic', '/amms/pick_target')
        self.declare_parameter('place_station_live_pose_topic', '/amms/place_station_pose')
        self.declare_parameter('prefer_external_pick_target', True)
        self.declare_parameter('external_pick_target_timeout_sec', 3.0)
        self.declare_parameter('external_target_match_tolerance_m', 0.05)
        self.declare_parameter('gripper_controller_topic', '/gripper_position_controller/commands')
        self.declare_parameter('arm_action_name', '/arm_position_controller/follow_joint_trajectory')
        self.declare_parameter('target_frame', 'arm_base_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('place_table_frame', 'map')
        self.declare_parameter('place_table_position', [0.0, -2.75, 0.0])
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
        self.declare_parameter('approach_distance', 0.05)
        self.declare_parameter('lift_height', 0.12)
        self.declare_parameter('grasp_z_bias', 0.010)
        self.declare_parameter('grasp_forward_offset', 0.0)
        self.declare_parameter('grasp_surface_to_center_offset', 0.0125)
        self.declare_parameter('use_shape_center_offsets', True)
        self.declare_parameter('cube_center_offset_m', 0.0125)
        self.declare_parameter('cylinder_center_offset_m', 0.0125)
        self.declare_parameter('block_center_offset_m', 0.0125)
        self.declare_parameter('default_center_offset_m', 0.0125)
        self.declare_parameter('grasp_close_settle_sec', 0.8)
        self.declare_parameter('open_gripper_width', 0.0115)
        self.declare_parameter('closed_gripper_width', 0.0)
        self.declare_parameter('use_gazebo_grasp_assist', False)
        self.declare_parameter('require_verified_grasp_attachment', True)
        self.declare_parameter('grasp_assist_backend', 'gazebo_state')
        self.declare_parameter('gazebo_set_entity_state_service', '/set_entity_state')
        self.declare_parameter('gazebo_get_entity_state_service', '/get_entity_state')
        self.declare_parameter('gazebo_get_link_properties_service', '/get_link_properties')
        self.declare_parameter('gazebo_set_link_properties_service', '/set_link_properties')
        self.declare_parameter('use_gazebo_truth_fallback', False)
        self.declare_parameter('gazebo_truth_frame', 'map')
        self.declare_parameter('disable_gravity_while_holding', True)
        self.declare_parameter('use_live_grasp_offset', True)
        self.declare_parameter('held_object_offset_end_effector', [0.055, 0.0, 0.0])
        self.declare_parameter('held_object_reference_frame', 'k12_amms::end_effector')
        self.declare_parameter('held_object_orientation_follows_gripper', True)
        self.declare_parameter('max_live_grasp_offset_m', 0.950625)
        self.declare_parameter('release_object_z_bias', 0.0)
        self.declare_parameter('release_upright_orientation', True)
        self.declare_parameter('snap_release_to_table_center', True)
        self.declare_parameter('place_table_release_height', 0.1315)
        self.declare_parameter('place_table_top_z', 0.10)
        self.declare_parameter('place_platform_thickness', 0.015)
        self.declare_parameter('release_clearance', 0.004)
        self.declare_parameter('release_cube_height', 0.025)
        self.declare_parameter('release_cylinder_height', 0.025)
        self.declare_parameter('release_block_height', 0.025)
        self.declare_parameter('place_table_release_xy_offset', [0.0, 0.11])
        self.declare_parameter('place_table_release_red_cube_xy_offset', [-0.08, 0.0])
        self.declare_parameter('place_table_release_green_cylinder_xy_offset', [0.0, 0.0])
        self.declare_parameter('place_table_release_blue_block_xy_offset', [0.08, 0.0])
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
        self.declare_parameter('home_joints', [0.0, -0.45, 0.90, -0.45, 0.0])
        self.declare_parameter('carry_joints', [0.0, -0.45, 0.90, -0.45, 0.0])
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('place_position_arm_base', [0.32, 0.0, 0.12])
        self.declare_parameter('reachable_min_radius', 0.1447875)
        self.declare_parameter('reachable_max_radius', 0.658125)
        self.declare_parameter('reachable_min_z', -0.307125)
        self.declare_parameter('reachable_max_z', 0.23034375)
        self.declare_parameter('allow_best_effort_grasp', False)
        self.declare_parameter('best_effort_max_radius', 0.78975)
        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('move_group_planning_time', 10.0)
        self.declare_parameter('move_group_velocity_scale', 0.5)
        self.declare_parameter('move_group_acceleration_scale', 0.5)
        self.declare_parameter('move_group_num_planning_attempts', 10)
        self.declare_parameter('move_group_goal_orientation_tolerance', 0.1)
        self.declare_parameter('move_group_goal_position_tolerance', 0.01)
        self.declare_parameter('object_alignment_yaw', 0.0)
        self.declare_parameter('planning_scene_frame', 'map')
        self.declare_parameter('planning_scene_boxes', [
            -3.0, 2.75, 0.05, 0.30, 0.30, 0.10,
             0.0, 2.75, 0.05, 0.30, 0.30, 0.10,
             3.0, 2.75, 0.05, 0.30, 0.30, 0.10,
             0.0, -2.75, 0.05, 0.30, 0.30, 0.10,
            -0.08, -2.64, 0.1075, 0.06, 0.06, 0.015,
             0.00, -2.64, 0.1075, 0.06, 0.06, 0.015,
             0.08, -2.64, 0.1075, 0.06, 0.06, 0.015,
             0.0, 0.0, -0.025, 9.0, 6.0, 0.05,
            -4.45, 0.0, 0.20, 0.10, 6.0, 0.40,
             4.45, 0.0, 0.20, 0.10, 6.0, 0.40,
             0.0, 2.95, 0.20, 9.0, 0.10, 0.40,
             0.0, -2.95, 0.20, 9.0, 0.10, 0.40])
        self.declare_parameter('joint_feedback_timeout_sec', 1.0)
        self.declare_parameter('cartesian_position_tolerance', 0.003)
        self.declare_parameter('cartesian_orientation_tolerance', 0.03)
        self.declare_parameter('base_approach_enabled', False)
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
        self.ik = PlanarArmIK()
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

        self.state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity', callback_group=self.command_callback_group)
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self.command_callback_group)
        self.joint_feedback_times = {}
        self.motion_fault = False

        self.create_subscription(String, self.get_parameter('detections_topic').value, self._on_detections, 10)
        self.create_subscription(String, self.get_parameter('pick_target_topic').value, self._on_pick_target, 10)
        self.inspect_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.create_subscription(PointStamped, self.get_parameter('place_station_live_pose_topic').value, self._on_live_place_pose, 10)
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
        self._live_place_pose: Optional[Tuple[float, float, float]] = None
        self._live_place_pose_time: float = 0.0
        self.executing = False
        self.completed = False
        self.latest_joint_positions = {}
        self._startup_home_done = not bool(self.get_parameter('home_on_start').value)
        self.create_timer(1.0, self._go_home_once)
        self.create_timer(0.5, self._tick)
        self.create_timer(float(self.get_parameter('held_object_update_period_sec').value), self._update_held_object)
        self.create_timer(float(self.get_parameter('held_gripper_republish_sec').value), self._hold_gripper_closed)
        self.observer = GazeboObserver(node=self)

        self.get_logger().info('MoveIt Pick-Place Commander started. Waiting for move_group...')

    def _go_home_once(self):
        """Send the arm to home_joints (the folded reference posture) once, after joint states are available."""
        if (self._startup_home_done or self.executing or not self._has_valid_joint_states()
                or not self._move_group_action.server_is_ready()):
            return
        self._startup_home_done = True
        self.executing = True
        threading.Thread(target=self._startup_home_worker, daemon=True).start()

    def _startup_home_worker(self):
        try:
            self._move_joints(self.get_parameter('home_joints').value, 4.0, 'startup home')
        except Exception as exc:
            self.get_logger().warn(f'Startup home move failed: {exc}')
        finally:
            self.executing = False

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

    def _on_live_place_pose(self, msg: PointStamped):
        self._live_place_pose = (msg.point.x, msg.point.y, msg.point.z)
        self._live_place_pose_time = time.monotonic()
        self.get_logger().info(
            f'Received live place station pose: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})')

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
            self.joint_feedback_times[name] = time.monotonic()

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
        # The task manager publishes the selected target once. Confirm that
        # selection with new camera frames rather than treating repeated reads
        # of the same message as independent stable observations.
        if (self.latest_payload is None or time.monotonic() - self.latest_time
                > float(self.get_parameter('object_timeout_sec').value)):
            return None
        target = None
        tolerance = float(self.get_parameter('external_target_match_tolerance_m').value)
        for observed, position in self._transform_objects(
                self.latest_payload, self.get_parameter('target_frame').value):
            if observed.get('id') != object_id or not self._detection_has_confidence(observed):
                continue
            current_grasp = self._grasp_pose_from_object(position, observed)
            if math.dist(current_grasp, grasp) > tolerance:
                continue
            target = self._target_from_object(observed, current_grasp, position)
            if target is not None:
                target['_observation_stamp'] = self._observation_stamp(self.latest_payload)
                break
        if target is None:
            return None
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
        stamp = target.get('_observation_stamp')
        if stamp is not None and getattr(self, 'stable_target_stamp', None) == stamp:
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
        self.stable_target_stamp = stamp
        self.stable_target_count = self.stable_target_count + 1 if same_target else 1
        return self.stable_target_count >= required

    def _reset_stable_pick_target(self):
        self.stable_target_key = ''
        self.stable_target_position = None
        self.stable_target_stamp = None
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
            stamp = payload.get('header', {}).get('stamp', {})
            point.header.stamp.sec = int(stamp.get('sec', 0))
            point.header.stamp.nanosec = int(stamp.get('nanosec', 0))
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

    def _observation_stamp(self, payload):
        stamp = payload.get('header', {}).get('stamp')
        if stamp is not None:
            return int(stamp.get('sec', 0)), int(stamp.get('nanosec', 0))
        return self.latest_time

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

    @staticmethod
    def _radial_unit(point: Iterable[float]) -> Tuple[float, float, float]:
        x, y, _ = point
        radius = math.hypot(x, y)
        if radius < 1e-6:
            return 1.0, 0.0, 0.0
        return x / radius, y / radius, 0.0

    def _reachable_offset_pose(
        self,
        pose: Tuple[float, float, float],
        direction_unit: Tuple[float, float, float],
        offset: float,
    ) -> Tuple[float, float, float]:
        dx, dy, _ = direction_unit
        candidate = (pose[0] - offset * dx, pose[1] - offset * dy, pose[2])
        if self._within_workspace(candidate) and self.ik.solve(candidate) is not None:
            return candidate
        return pose

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

    def _target_from_object(
        self,
        obj: Dict,
        grasp: Tuple[float, float, float],
        object_arm_base_position: Tuple[float, float, float],
    ) -> Optional[Dict]:
        if obj.get('center_estimate_valid') is False:
            return None
        plan = self._pick_plan_for_grasp(grasp)
        if plan is None:
            return None
        return self._target_from_pick_plan(obj, plan, object_arm_base_position)

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
        target['pre_grasp_joints'] = plan['pre_grasp_joints']
        return target

    def _pick_plan_for_grasp(self, grasp):
        if not self._within_workspace(grasp):
            return None
        return self.ik.pick_plan(
            grasp, float(self.get_parameter('approach_distance').value),
            float(self.get_parameter('lift_height').value))

    def _lift_height_candidates(self) -> Tuple[float, ...]:
        configured = max(0.02, float(self.get_parameter('lift_height').value))
        candidates = [configured, 0.12, 0.10, 0.08, 0.06]
        unique = []
        for value in candidates:
            value = round(max(0.02, min(configured, value)), 3)
            if value not in unique:
                unique.append(value)
        return tuple(unique)

    def _best_effort_pick_plan(self, grasp):
        # Moving a grasp coordinate does not move the cube. Reposition through Nav2.
        return None

    def _select_reachable_target(self, payload):
        target_frame = self.get_parameter('target_frame').value
        candidates = []
        fallback_candidates = []
        for obj, xyz_raw in self._transform_objects(payload, target_frame):
            if (not self._detection_has_confidence(obj)
                    or obj.get('center_estimate_valid') is False):
                continue
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            plan = self._pick_plan_for_grasp(grasp)
            if plan is not None:
                candidates.append((_distance(grasp), obj, plan, xyz_raw, False))
                continue
            fallback_plan = self._best_effort_pick_plan(grasp)
            if fallback_plan is not None:
                fallback_candidates.append((_distance(grasp), obj, fallback_plan, xyz_raw, True))

        if not candidates:
            candidates = fallback_candidates
        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, obj, plan, xyz_raw, best_effort = candidates[0]
        target = self._target_from_pick_plan(obj, plan, xyz_raw)
        target['_observation_stamp'] = self._observation_stamp(payload)
        grasp = plan['grasp']
        label = 'best-effort ' if best_effort else ''
        self.get_logger().info(
            f'Selected {label}{target.get("color", "?")} {target.get("shape", "object")} at '
            f'grasp=({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})'
        )
        return target

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
        plan = self._pick_plan_for_grasp(grasp)
        if plan is None:
            plan = self._best_effort_pick_plan(grasp)
        if plan is None:
            return None

        return self._target_from_pick_plan(obj, plan, xyz_raw)

    def _select_gazebo_truth_target(self):
        if not bool(self.get_parameter('use_gazebo_truth_fallback').value):
            return None
        if not self.get_entity_state_client.wait_for_service(timeout_sec=0.05):
            return None
        candidates = []
        for obj, xyz_raw in self._gazebo_truth_objects():
            grasp = self._grasp_pose_from_object(xyz_raw, obj)
            plan = self._pick_plan_for_grasp(grasp)
            if plan is None:
                continue
            candidates.append((_distance(grasp), obj, plan, xyz_raw))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        _, obj, plan, xyz_raw = candidates[0]
        return self._target_from_pick_plan(obj, plan, xyz_raw)

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
        if obj is not None and (obj.get('pose_source') == 'gazebo_truth' or obj.get('position_is_center', False)):
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
        if self.ik.solve(grasp) is None:
            return False
        return self._check_ik(grasp)

    def _check_ik(self, point_arm_base):
        if self.ik.solve(point_arm_base) is not None:
            return True
        if not self.ik_client.wait_for_service(timeout_sec=0.1):
            return False
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter('move_group_name').value
        req.ik_request.ik_link_name = 'end_effector'
        req.ik_request.pose_stamped.header.frame_id = self.get_parameter('target_frame').value
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose.position.x = point_arm_base[0]
        req.ik_request.pose_stamped.pose.position.y = point_arm_base[1]
        req.ik_request.pose_stamped.pose.position.z = point_arm_base[2]
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.timeout = Duration(seconds=0.5).to_msg()
        future = self.ik_client.call_async(req)
        if not self._wait_for_future(future, 0.5):
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

    def _current_place_pose(self, return_plan=False):
        """Plan the exact measured platform centre in the physical robot frame."""
        with self.hold_lock:
            entity = self.held_entity_name
        _, platform, _ = resolve_entities(entity)
        measured = self.observer.get_pose(platform, max_age_sec=.5)
        if measured is None:
            raise RuntimeError(f'Fresh platform observation unavailable: {platform}')
        # Initial estimate only; measure the actual held offset at pre-place below.
        centre = (pose_matrix(measured['position'], measured['orientation']) @
                  np.array([0., 0., .0075+.0125+.004, 1.]))[:3]
        tcp_world = centre + np.array([0., 0., float(self.get_parameter('grasp_z_bias').value)])
        point = self.observer.world_point_in_frame(self, tcp_world, self.get_parameter('target_frame').value)
        plan = self.ik.pick_plan(point, approach=.025, lift=.025, pitches=(math.pi/2,))
        if plan is None:
            raise RuntimeError(f'Exact platform centre unreachable: world={centre}, arm_tcp={point}')
        self.get_logger().info(f'Physical placement target: platform={platform} world_cube={centre.tolist()} arm_tcp={point}')
        return (point, plan) if return_plan else point

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
        grasp = tuple(float(v) for v in target['arm_base_position'])
        plan = self._pick_plan_for_grasp(grasp)
        if plan is None:
            raise RuntimeError(f'No complete TCP descent/lift for {grasp}; reposition the base')
        self.cmd_vel_pub.publish(Twist())
        self._sync_support_surfaces()
        self._set_detected_object(grasp)
        self.get_logger().info(f"Pick TCP: grasp={grasp}, pre_grasp={plan['pre_grasp']}, "
                               f"pitch={plan['pitch']:.3f}; preserve orientation during descent")
        self._open_gripper()
        self._move_joints(self.get_parameter('home_joints').value, 3.0, 'home')
        self._move_joints(self._aligned_grasp_joints(plan['pre_grasp_joints']), 3.0, 'pre-grasp')
        self._verify_tcp_position(plan['pre_grasp'])
        self._vertical_move(grasp, 'grasp')
        self._close_gripper(settle_sec=float(self.get_parameter('grasp_close_settle_sec').value))
        self._attach_object(target)
        self._vertical_move(plan['lift'], 'lift')
        self._verify_grasp_secured()
        self._move_joints(self.get_parameter('carry_joints').value, 3.0, 'carry')
        self._verify_grasp_secured()
        with self.hold_lock:
            entity = self.held_entity_name
        model_name, _, _ = resolve_entities(entity)
        p_pick = self.observer.get_pose(model_name, max_age_sec=5.0)
        self.get_logger().info(f'[PHYSICAL_TRACK] Pick complete: held={model_name} pose={p_pick}')
        self.get_logger().info('Pick sequence complete.')

    def _execute_place(self):
        self.cmd_vel_pub.publish(Twist())
        self._wait_physical_base_stopped()
        self._sync_support_surfaces(physical=True)
        if not self._holding_object():
            raise RuntimeError('Refusing to place: no verified grasp')
        self._verify_grasp_secured()
        with self.hold_lock:
            entity = self.held_entity_name
        model, platform, _ = resolve_entities(entity)
        place, plan = self._current_place_pose(return_plan=True)
        self._move_joints(self._aligned_grasp_joints(plan['pre_grasp_joints'], physical=True), 3., 'pre-place')
        self._verify_tcp_position(plan['pre_grasp'])
        # Use measured cube/TCP geometry at the final orientation. Corrections
        # happen above the surface, through the same collision-checked planner
        # used for picking. Never descend on an unreachable correction.
        for attempt in range(4):
            self._sleep(.3)
            self._wait_physical_base_stopped()
            cube = self.observer.get_pose(model, max_age_sec=.5)
            support = self.observer.get_pose(platform, max_age_sec=.5)
            if cube is None or support is None:
                raise RuntimeError('Fresh cube/platform observations required before descent')
            desired = (pose_matrix(support['position'], support['orientation']) @
                       np.array([0., 0., .0075+.0125+.004+.025, 1.]))[:3]
            error = desired - np.array(cube['position'])
            tcp_transform = transform_matrix(self.tf_buffer.lookup_transform(
                self.get_parameter('target_frame').value, 'gripper_tcp', rclpy.time.Time(),
                timeout=Duration(seconds=.5)))
            world_to_arm = self.observer.world_to_frame(self, self.get_parameter('target_frame').value)
            cube_arm = world_to_arm @ pose_matrix(cube['position'], cube['orientation'])
            held_transform = np.linalg.inv(tcp_transform) @ cube_arm
            self.get_logger().info(
                f'Pre-place measurement {attempt}: world_error_mm={(error*1000).tolist()} '
                f'held_cube_in_tcp={held_transform[:3,3].tolist()} cube={cube["position"]}')
            self._set_payload_collision(True, measured_transform=held_transform)
            if np.linalg.norm(error[:2]) <= .0015 and abs(error[2]) <= .001:
                # Descent from actual feedback preserves X/Y and orientation.
                actual = self.ik.forward(self._arm_feedback())
                place = (actual[0], actual[1], actual[2]-.025)
                retreat = tuple(actual)
                break
            if attempt == 3:
                raise RuntimeError('Measured pre-place alignment did not converge; keeping gripper closed')
            corrected = tcp_transform[:3,3] + world_to_arm[:3,:3] @ error
            joints = self.ik.solve(tuple(corrected), pitch=math.pi/2, seed=self._arm_feedback())
            if joints is None:
                raise RuntimeError(f'Measured placement correction is unreachable: {corrected}')
            joints = tuple(joints[:4]) + (self._arm_feedback()[4],)
            self._move_joints(joints, 2., 'pre-place physical correction')
            self._verify_tcp_position(tuple(corrected))
        self._vertical_move(place, 'place')
        self._sleep(.3)
        ok, reason, metrics = self.observer.verify_cube_above_platform(
            model, platform, max_horizontal_error=.003, min_gap=.001, max_gap=.006, max_age_sec=.5)
        self.get_logger().info(f'Pre-release physical metrics: {metrics}')
        if not ok:
            raise RuntimeError(f'Pre-release verification rejected: {reason}')
        self._wait_physical_base_stopped()
        self._sleep(float(self.get_parameter('release_settle_before_open_sec').value))
        self.hold_gripper_republish_suspended = True
        try:
            self._open_gripper(allow_while_holding=True)
            self._release_held_object(place)
        finally:
            self.hold_gripper_republish_suspended = False
        self._verify_release_complete()
        self._wait_supported_cube(model, platform, 'post-release')
        self._sync_released_cube(model)
        self._vertical_move(retreat, 'place retreat', released_support_verified=True)
        self._move_joints(self.get_parameter('home_joints').value, 3., 'return home')
        self._wait_supported_cube(model, platform, 'post-retreat')
        self._clear_completed_target()
        self.get_logger().info(f'VERIFIED SUCCESS: {model} physically rests on {platform}!')
        self.get_logger().info('Place sequence complete.')

    def _sync_released_cube(self, model):
        """Replace the detached object's release pose with its settled pose.

        MoveIt detachment retains the release transform. Gravity and finger
        opening then move the real cube; retreat must check the real geometry.
        """
        if self.inspect_scene_client.wait_for_service(timeout_sec=2.):
            request = GetPlanningScene.Request()
            request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                             PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            future = self.inspect_scene_client.call_async(request)
            if self._wait_for_future(future, 2.) and future.result() is not None:
                scene = future.result().scene
                for obj in scene.world.collision_objects:
                    if obj.id == 'amms_held_object':
                        self.get_logger().info(f'Detached collision object before physical sync: {obj}')
                if any(obj.object.id == 'amms_held_object' for obj in scene.robot_state.attached_collision_objects):
                    raise RuntimeError('Payload remains attached in MoveIt after release')
        measured = self.observer.get_pose(model, max_age_sec=.5)
        if measured is None:
            raise RuntimeError('Fresh released-cube pose required to update collision scene')
        matrix = self.observer.world_to_frame(self, self.get_parameter('target_frame').value) @ pose_matrix(
            measured['position'], measured['orientation'])
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        obj = CollisionObject()
        obj.id = 'amms_held_object'
        obj.header.frame_id = self.get_parameter('target_frame').value
        obj.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type, shape.dimensions = SolidPrimitive.BOX, [.025]*3
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3,3])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = matrix_quaternion(matrix)
        obj.primitives, obj.primitive_poses = [shape], [pose]
        request.scene.world.collision_objects = [obj]
        self._apply_scene(request)
        self.get_logger().info(
            f'Released collision object synchronized: world_cube={measured["position"]} '
            f'arm_cube={matrix[:3,3].tolist()} fingers={self._finger_feedback()}')

    def _wait_physical_base_stopped(self):
        deadline = time.monotonic()+5.
        since = None
        while time.monotonic() < deadline:
            pose = self.observer.get_pose('four_wheel', max_age_sec=.5)
            if pose and pose['speed'] < .002 and pose['angular_speed'] < .01:
                since = pose['stamp'] if since is None else since
                if pose['stamp']-since >= .3:
                    return
            else:
                since = None
            self._sleep(.05)
        raise RuntimeError('Physical base motion has not settled; refusing placement motion')

    def _wait_supported_cube(self, model, platform, stage):
        deadline = time.monotonic()+10.
        reason, metrics = 'No observations', {}
        while time.monotonic() < deadline:
            ok, reason, metrics = self.observer.verify_cube_resting_on_platform(
                model, platform, max_horizontal_error=.005, max_height_error=.002,
                max_speed=.005, max_age_sec=.5, stability_duration=1.)
            if ok:
                self.get_logger().info(f'{stage} physical verification: {metrics}')
                return
            self._sleep(.1)
        raise RuntimeError(f'{stage} physical verification failed: {reason}; metrics={metrics}')

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


    def _apply_scene(self, request):
        if not self.scene_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt planning-scene service unavailable')
        future = self.scene_client.call_async(request)
        if not self._wait_for_future(future, 5.0) or not future.result().success:
            raise RuntimeError('MoveIt planning-scene update failed')

    def _sync_support_surfaces(self, physical=False):
        values = list(self.get_parameter('planning_scene_boxes').value)
        if len(values) % 6:
            raise RuntimeError('planning_scene_boxes must contain x,y,z,size_x,size_y,size_z rows')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        world_to_arm = self.observer.world_to_frame(self, self.get_parameter('target_frame').value) if physical else None
        for index in range(0, len(values), 6):
            x,y,z,sx,sy,sz = values[index:index+6]
            pose = PoseStamped()
            pose.header.frame_id = self.get_parameter('planning_scene_frame').value
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x,y,z
            pose.pose.orientation.w = 1.0
            if physical:
                transformed = world_to_arm @ pose_matrix((x,y,z), (0.,0.,0.,1.))
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, transformed[:3,3])
                q = matrix_quaternion(transformed)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            else:
                pose = self.tf_buffer.transform(pose, self.get_parameter('target_frame').value,
                                                timeout=Duration(seconds=1.0))
            obj = CollisionObject()
            obj.id = f'amms_support_{index//6}'
            obj.header.frame_id = self.get_parameter('target_frame').value
            obj.operation = CollisionObject.ADD
            shape = SolidPrimitive()
            shape.type, shape.dimensions = SolidPrimitive.BOX, [sx,sy,sz]
            obj.primitives, obj.primitive_poses = [shape], [pose.pose]
            request.scene.world.collision_objects.append(obj)
        self._apply_scene(request)

    def _set_detected_object(self, grasp):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        obj = CollisionObject()
        obj.id = 'amms_held_object'
        obj.header.frame_id = self.get_parameter('target_frame').value
        obj.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type, shape.dimensions = SolidPrimitive.BOX, [0.025]*3
        pose = Pose()
        pose.position.x, pose.position.y = grasp[:2]
        pose.position.z = grasp[2] - float(self.get_parameter('grasp_z_bias').value)
        transform = self.tf_buffer.lookup_transform(
            self.get_parameter('target_frame').value,
            self.get_parameter('planning_scene_frame').value, rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        q = transform.transform.rotation
        half_yaw = float(self.get_parameter('object_alignment_yaw').value) / 2
        c, t = math.cos(half_yaw), math.sin(half_yaw)
        pose.orientation.x, pose.orientation.y = c*q.x+t*q.y, c*q.y-t*q.x
        pose.orientation.z, pose.orientation.w = c*q.z+t*q.w, c*q.w-t*q.z
        obj.primitives, obj.primitive_poses = [shape], [pose]
        request.scene.world.collision_objects = [obj]
        self._apply_scene(request)

    def _set_payload_collision(self, attached, measured_transform=None):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        payload = AttachedCollisionObject()
        payload.link_name = 'gripper_tcp'
        payload.touch_links = ['left_gripper', 'right_gripper', 'end_effector']
        payload.object.header.frame_id = 'gripper_tcp'
        payload.object.id = 'amms_held_object'
        payload.object.operation = CollisionObject.ADD if attached else CollisionObject.REMOVE
        if attached:
            shape = SolidPrimitive()
            shape.type, shape.dimensions = SolidPrimitive.BOX, [0.025]*3
            pose = Pose()
            # Local +X points down during grasp: preserve the grasp offset.
            pose.position.x = float(self.get_parameter('grasp_z_bias').value)
            pose.orientation.w = 1.0
            if measured_transform is not None:
                pose.position.x, pose.position.y, pose.position.z = map(float, measured_transform[:3,3])
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = matrix_quaternion(measured_transform)
            payload.object.primitives, payload.object.primitive_poses = [shape], [pose]
        request.scene.robot_state.attached_collision_objects = [payload]
        self._apply_scene(request)

    def _aligned_grasp_joints(self, joints, physical=False):
        # The supplied world has axis-aligned cubes. Their yaw can be configured
        # without changing dimensions. Alignment happens BEFORE descent.
        transform = self.tf_buffer.lookup_transform(
            self.get_parameter('planning_scene_frame').value,
            self.get_parameter('target_frame').value, rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        q = transform.transform.rotation
        base_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if physical:
            arm_to_world = np.linalg.inv(self.observer.world_to_frame(self, self.get_parameter('target_frame').value))
            base_yaw = math.atan2(arm_to_world[1,0], arm_to_world[0,0])
        roll = math.remainder(base_yaw + joints[0]
                              - float(self.get_parameter('object_alignment_yaw').value), math.pi/2)
        aligned = tuple(joints[:4]) + (roll,)
        if not self.ik.valid_joints(aligned):
            raise RuntimeError('Cube alignment violates wrist limits')
        return aligned

    def _arm_feedback(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('joint_feedback_timeout_sec').value)
        if any(now-self.joint_feedback_times.get(name, -float('inf')) > timeout
               for name in self.joint_names):
            raise RuntimeError('Missing or stale arm joint feedback')
        q = tuple(self.latest_joint_positions[name] for name in self.joint_names)
        if not self.ik.valid_joints(q):
            raise RuntimeError(f'Arm feedback violates URDF limits: {q}')
        return q

    def _verify_tcp_position(self, expected):
        actual = self.ik.forward(self._arm_feedback())
        tolerance = float(self.get_parameter('cartesian_position_tolerance').value)
        if math.dist(actual, expected) > tolerance:
            raise RuntimeError(f'TCP did not reach target: actual={actual}, expected={expected}')
        # Detect a stale/different loaded robot model before descending.
        transform = self.tf_buffer.lookup_transform(
            self.get_parameter('target_frame').value, 'gripper_tcp', rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        t = transform.transform.translation
        if math.dist(actual, (t.x, t.y, t.z)) > tolerance:
            raise RuntimeError('Analytical FK disagrees with loaded gripper_tcp TF; rebuild robot description')

    def _validate_joint_path(self, path, departing_support=False, label='', released_support_verified=False):
        if not self.state_validity_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt state-validity service unavailable; refusing unchecked motion')
        samples = [path[0]]
        for start, end in zip(path, path[1:]):
            count = max(2, math.ceil(max(abs(a-b) for a, b in zip(start, end))/0.01))
            samples.extend(tuple(a+(b-a)*i/count for a, b in zip(start, end))
                           for i in range(1, count+1))
        # A physically supported cube can rest against an open finger with
        # sub-micrometre simulator contact. Permit only withdrawal from that
        # existing contact, never a new collision or a penetrating motion.
        released_retreat = label == 'place retreat' and released_support_verified
        initial_contacts = {}
        contact_cleared = False
        previous_z = self.ik.forward(path[0])[2]
        if released_retreat:
            if self._holding_object() or min(self._finger_feedback()) < .0114:
                raise RuntimeError('Released-cube retreat requires fully open, empty gripper')
            start_xyz = self.ik.forward(path[0])
            for q in samples:
                xyz = self.ik.forward(q)
                if (math.hypot(xyz[0]-start_xyz[0], xyz[1]-start_xyz[1]) > 1e-5
                        or xyz[2] < previous_z-1e-9
                        or max(abs(q[0]-path[0][0]), abs(q[4]-path[0][4]),
                               abs(sum(q[1:4])-sum(path[0][1:4]))) > 1e-6):
                    raise RuntimeError('Released-cube retreat must ascend vertically at fixed orientation')
                previous_z = xyz[2]
            if previous_z <= start_xyz[2]+.002:
                raise RuntimeError('Released-cube retreat must provide positive separation')
        allowed_touch_samples = 0
        for index, q in enumerate(samples):
            req = GetStateValidity.Request()
            req.group_name = self.get_parameter('move_group_name').value
            req.robot_state.is_diff = True
            positions = dict(self.latest_joint_positions)
            positions.update(zip(self.joint_names, q))
            req.robot_state.joint_state.name = list(positions)
            req.robot_state.joint_state.position = list(positions.values())
            future = self.state_validity_client.call_async(req)
            if not self._wait_for_future(future, 2.0):
                raise RuntimeError('MoveIt collision check timed out')
            result = future.result()
            if released_retreat and result is not None:
                if result.valid:
                    contact_cleared = True
                elif not contact_cleared and result.contacts and index < len(samples)-1:
                    depths = {}
                    for c in result.contacts:
                        pair = frozenset((c.contact_body_1, c.contact_body_2))
                        depths[pair] = max(depths.get(pair, 0.), c.depth)
                    finger_pairs = {frozenset(('amms_held_object', finger))
                                    for finger in ('left_gripper', 'right_gripper')}
                    if index == 0:
                        initial_contacts = depths.copy()
                    if (depths.keys() <= finger_pairs and depths.keys() <= initial_contacts.keys()
                            and all(0. <= d <= 1e-6 and d <= initial_contacts[p]+1e-8
                                    for p, d in depths.items())):
                        initial_contacts = depths
                        allowed_touch_samples += 1
                        continue
            if result is not None and not result.valid:
                # A supported payload starts in contact or reaches shallow contact at table/platform.
                # Allow only <=1 mm of payload/support contact during lift or place descent.
                dz = self.ik.forward(q)[2] - self.ik.forward(path[0])[2]
                is_support_contact = bool(result.contacts) and all(
                    c.depth <= 0.001 and 'amms_held_object' in (c.contact_body_1, c.contact_body_2)
                    and any(
                        (body in ('amms_support_0', 'amms_support_1', 'amms_support_2') if departing_support else body.startswith('amms_support_'))
                        for body in (c.contact_body_1, c.contact_body_2)
                    )
                    for c in result.contacts)
                if is_support_contact and ((departing_support and 0.0 <= dz <= 0.002) or (label == 'place')):
                    continue
            if result is None or not result.valid:
                contacts = [] if result is None else [
                    f'{c.contact_body_1}/{c.contact_body_2} depth={c.depth:.9f}m' for c in result.contacts]
                raise RuntimeError(f'Cartesian trajectory is in collision: {contacts}')
        if allowed_touch_samples:
            self.get_logger().info(
                f'Released-cube retreat: {allowed_touch_samples} initial finger-touch samples '
                'within 1 micrometre; penetration did not increase and endpoint is collision-free')

    def _vertical_move(self, target, label, released_support_verified=False):
        if self.motion_fault:
            raise RuntimeError('Previous motion timed out; refusing another trajectory')
        start = self._arm_feedback()
        position = self.ik.forward(start)
        tolerance = float(self.get_parameter('cartesian_position_tolerance').value)
        if math.hypot(position[0]-target[0], position[1]-target[1]) > tolerance:
            raise RuntimeError(f'{label}: TCP is not aligned above target; refusing lateral sweep')
        if label == 'grasp' and target[2] >= position[2]:
            raise RuntimeError('Grasp must descend from above the target')
        path = self.ik.vertical_path(start, target[2])
        self._validate_joint_path(path, departing_support=(label == 'lift' and target[2] > position[2]),
                                  label=label, released_support_verified=released_support_verified)
        # Validation may take time; don't execute from an obsolete start state.
        if max(abs(a-b) for a, b in zip(start, self._arm_feedback())) > 0.01:
            raise RuntimeError('Arm moved during Cartesian path validation')
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        elapsed = 0.0
        for index, q in enumerate(path):
            if index:
                delta = max(abs(a-b) for a, b in zip(path[index-1], q))
                # Zero velocity/acceleration endpoints give monotone quintic
                # joint interpolation. Every joint uses the same time scaling.
                elapsed += max(0.08, 1.875*delta/0.4, math.sqrt(5.8*delta/0.8))
            point = JointTrajectoryPoint()
            point.positions = list(q)
            point.velocities = [0.0]*5
            point.accelerations = [0.0]*5
            point.time_from_start = Duration(seconds=elapsed).to_msg()
            trajectory.points.append(point)
        self.get_logger().info(f'{label}: {len(path)} Cartesian samples, '
                               f'roll={start[4]:.4f}, pitch={sum(start[1:4]):.4f}, '
                               f'X/Y fixed at {position[:2]}')
        self._send_joint_trajectory_goal(trajectory, elapsed, label, cartesian_start=start)
        self._verify_tcp_position((position[0], position[1], target[2]))
        actual = self._arm_feedback()
        if max(abs(actual[0]-start[0]), abs(actual[4]-start[4]),
               abs(sum(actual[1:4])-sum(start[1:4]))) > float(
                   self.get_parameter('cartesian_orientation_tolerance').value):
            raise RuntimeError(f'{label}: final gripper orientation differs from pre-grasp')

    def _move_pose(self, point_arm_base, label, duration_sec=3.0):
        joints = self.ik.solve(point_arm_base, seed=self._arm_feedback())
        if joints is None:
            raise RuntimeError(f'No URDF-valid TCP solution for {label}: {point_arm_base}')
        self._move_joints(joints, duration_sec, label)
        self._verify_tcp_position(point_arm_base)

    def _move_joints(self, positions, duration_sec, label):
        if self.motion_fault:
            raise RuntimeError('Previous motion timed out; restart commander after verifying arm is stopped')
        positions = tuple(map(float, positions))
        if not self.ik.valid_joints(positions):
            raise RuntimeError(f'Invalid {label} joint target: {positions}')
        self._arm_feedback()
        if not self._move_group_action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveIt unavailable; no direct joint-motion fallback')
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter('move_group_name').value
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = float(self.get_parameter('move_group_planning_time').value)
        goal.request.num_planning_attempts = int(self.get_parameter('move_group_num_planning_attempts').value)
        goal.request.max_velocity_scaling_factor = float(self.get_parameter('move_group_velocity_scale').value)
        goal.request.max_acceleration_scaling_factor = float(self.get_parameter('move_group_acceleration_scale').value)
        constraints = Constraints()
        for name, position in zip(self.joint_names, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = constraint.tolerance_below = 0.002
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        future = self._move_group_action.send_goal_async(goal)
        if not self._wait_for_future(future, 10.0):
            self.motion_fault = True
            # A late accepted goal must not race a recovery command.
            future.add_done_callback(lambda done: done.result().cancel_goal_async()
                                     if done.result() and done.result().accepted else None)
            raise RuntimeError(f'MoveIt goal submission timed out: {label}')
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'MoveIt rejected {label}')
        future = handle.get_result_async()
        if not self._wait_for_future(future, goal.request.allowed_planning_time+45.0):
            self.motion_fault = True
            self._wait_for_future(handle.cancel_goal_async(), 5.0)
            raise RuntimeError(f'MoveIt execution timed out: {label}')
        result = future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != 1:
            err_code = -999 if result is None or not hasattr(result, 'result') else result.result.error_code.val
            raise RuntimeError(f'MoveIt failed {label}: status={getattr(result, "status", None)}, error_code={err_code}')
        # Action completion and /joint_states arrive on separate subscriptions.
        # Also allow the position servo to settle inside the stricter TCP check.
        settle_deadline = time.monotonic() + 3.0
        while max(abs(a-b) for a, b in zip(positions, self._arm_feedback())) > 0.005:
            if time.monotonic() >= settle_deadline:
                raise RuntimeError(f'Joint feedback did not reach {label}')
            self._sleep(0.02)

    def _send_joint_trajectory_goal(self, trajectory, duration_sec, label, cartesian_start=None):
        server_timeout = 15.0
        if not self.arm_action_client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError('FollowJointTrajectory action server is not available.')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        # Controller defaults have no per-joint position tolerance. Without
        # these, a stopped arm can report success short of the grasp pose.
        # Keep the last waypoint commanded while the measured joints settle.
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.002, velocity=0.01)
            for name in trajectory.joint_names
        ]
        goal.goal_time_tolerance = Duration(seconds=8.0).to_msg()
        send_future = self.arm_action_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, 15.0):
            self.motion_fault = True
            send_future.add_done_callback(lambda done: done.result().cancel_goal_async()
                                          if done.result() and done.result().accepted else None)
            raise RuntimeError(f'Timed out sending trajectory goal for {label}')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'Trajectory goal for {label} was rejected.')

        result_future = goal_handle.get_result_async()
        # Controllers use ROS time; slow simulation must not consume its motion
        # allowance in wall time. Still abort if the ROS clock stops or resets.
        last_ros = self.get_clock().now().nanoseconds * 1e-9
        # Allow delivery of the controller's result after its 8 s goal window.
        deadline = last_ros + duration_sec + 9.0
        last_progress_wall = time.monotonic()
        try:
            while not result_future.done():
                now_ros = self.get_clock().now().nanoseconds * 1e-9
                if now_ros < last_ros:
                    raise RuntimeError(f'{label}: ROS clock moved backwards')
                if now_ros > last_ros:
                    last_progress_wall = time.monotonic()
                elif time.monotonic() - last_progress_wall > 5.0:
                    raise RuntimeError(f'{label}: ROS clock stalled')
                last_ros = now_ros
                if now_ros >= deadline:
                    break
                q = self._arm_feedback()
                if cartesian_start is not None:
                    actual, start_point = self.ik.forward(q), self.ik.forward(cartesian_start)
                    if math.hypot(actual[0]-start_point[0], actual[1]-start_point[1]) > 0.005:
                        raise RuntimeError(f'{label}: measured lateral drift exceeds 5 mm')
                    angular_error = max(abs(q[0]-cartesian_start[0]), abs(q[4]-cartesian_start[4]),
                                        abs(sum(q[1:4])-sum(cartesian_start[1:4])))
                    if angular_error > float(self.get_parameter('cartesian_orientation_tolerance').value):
                        raise RuntimeError(f'{label}: measured orientation drift exceeds tolerance')
                self._sleep(0.02)
            if not result_future.done():
                raise RuntimeError(f'Trajectory goal for {label} timed out')
        except Exception:
            self.motion_fault = True
            self._wait_for_future(goal_handle.cancel_goal_async(), 5.0)
            raise

        result = result_future.result()
        if (result is None or result.status != GoalStatus.STATUS_SUCCEEDED
                or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            status = 'no result' if result is None else str(result.status)
            detail = '' if result is None else (
                f': code={result.result.error_code}, {result.result.error_string}')
            raise RuntimeError(f'Trajectory goal for {label} failed with status {status}{detail}')

    def _finger_feedback(self):
        names = ('left_gripper_joint', 'right_gripper_joint')
        now = time.monotonic()
        if any(now-self.joint_feedback_times.get(name, -float('inf')) > 1.0 for name in names):
            raise RuntimeError('Missing/stale gripper feedback')
        return tuple(self.latest_joint_positions[name] for name in names)

    def _open_gripper(self, allow_while_holding=False):
        if self._holding_object() and not allow_while_holding:
            raise RuntimeError('Cannot open while transporting an object')
        value = float(self.get_parameter('open_gripper_width').value)
        if not 0.0 <= value <= 0.0115:
            raise RuntimeError('Gripper displacement must be within URDF range 0..0.0115 m')
        self._command_gripper_until_reached(value, value, 'open gripper', max_wait_sec=10.0)
        feedback = self._finger_feedback()
        if any(abs(q-value) > 0.001 for q in feedback):
            raise RuntimeError(f'Gripper did not open fully: feedback={feedback}, target={value}')

    def _close_gripper(self, settle_sec=0.8):
        value = float(self.get_parameter('closed_gripper_width').value)
        open_value = float(self.get_parameter('open_gripper_width').value)
        if not 0.0 <= value <= 0.0115:
            raise RuntimeError('Invalid closed gripper displacement')
        self._publish_gripper(value, value, 'close gripper')
        self._sleep(max(0.5, settle_sec))

        # ── Phase 1: confirm the controller acted on the close command ──────────
        # ForwardCommandController + GazeboSimSystem may ignore a single publish
        # if the sim is busy post-trajectory. The old stability loop exits after
        # 0.4 s of "no movement", but fingers already stable at 0.0115 (open)
        # satisfy that condition immediately — producing a silent false-close.
        # We must keep re-commanding until at least one finger has moved ≥ 1 mm
        # inward from the open position before declaring the close phase complete.
        _move_threshold = 0.001  # 1 mm is unambiguous inward movement
        _move_deadline = time.monotonic() + 3.0
        while time.monotonic() < _move_deadline:
            self._publish_gripper(value, value, 'close command', log=False)
            self._sleep(0.05)
            left, right = self._finger_feedback()
            if (open_value - left) > _move_threshold or (open_value - right) > _move_threshold:
                break
        else:
            left, right = self._finger_feedback()
            raise RuntimeError(
                f'Gripper controller not responding: fingers did not move toward closed position '
                f'(left={left:.4f}, right={right:.4f})'
            )

        # ── Phase 2: wait for contact to stabilise ──────────────────────────────
        previous = self._finger_feedback()
        stable_since = time.monotonic()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            # Re-publish every cycle so ForwardCommandController never misses the command.
            self._publish_gripper(value, value, 'close hold', log=False)
            self._sleep(0.10)
            current = self._finger_feedback()
            if max(abs(a-b) for a,b in zip(current, previous)) > 0.0002:
                stable_since = time.monotonic()
            elif time.monotonic()-stable_since >= 0.4:
                break
            previous = current
        else:
            raise RuntimeError('Gripper contact did not settle')
        self._verify_finger_contact()

    def _verify_finger_contact(self):
        left, right = self._finger_feedback()
        if left is None or right is None or math.isnan(left) or math.isnan(right):
            raise RuntimeError('No credible two-finger contact: nan or None')
        try:
            object_width = float(self.get_parameter('grasp_object_width_m').value)
            width_tol = float(self.get_parameter('grasp_width_tolerance_m').value)
            center_tol = float(self.get_parameter('grasp_centering_tolerance_m').value)
        except Exception:
            object_width = 0.025
            width_tol = 0.008
            center_tol = 0.006

        # Fully-closed empty gap = 7 mm (0.007 m). Total gap = 0.007 + left + right.
        gap = 0.007 + left + right
        if not (min(left, right) > 0.0005 and max(left, right) <= 0.012 and min(left, right) < 0.0114):
            raise RuntimeError(f'No credible two-finger contact: fingers={left:.4f}, {right:.4f}')
        if abs(gap - object_width) > width_tol:
            raise RuntimeError(f'No credible two-finger contact: gap={gap:.4f} outside width tolerance {width_tol}')
        if abs(left - right) / 2.0 > center_tol:
            raise RuntimeError(f'No credible two-finger contact: centering offset {abs(left-right)/2.0:.4f} exceeds {center_tol}')

    def _command_gripper_until_reached(self, left_target, right_target, label, max_wait_sec=2.5):
        self._publish_gripper(left_target, right_target, label)
        end_time = time.monotonic() + max_wait_sec
        while rclpy.ok() and time.monotonic() < end_time:
            left = self.latest_joint_positions.get('left_gripper_joint')
            right = self.latest_joint_positions.get('right_gripper_joint')
            left_ok = left is not None and abs(left - left_target) <= 0.0005
            right_ok = right is not None and abs(right - right_target) <= 0.0005
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
            self._verify_finger_contact()
            with self.hold_lock:
                self.held_entity_name = str(target.get('id', target.get('color', 'object')))
            self._set_payload_collision(True)
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
        if not bool(self.get_parameter('use_gazebo_grasp_assist').value):
            value = float(self.get_parameter('open_gripper_width').value)
            if any(abs(q-value) > 0.001 for q in self._finger_feedback()):
                raise RuntimeError('Release failed: fingers are not open')
            self._set_payload_collision(False)
            with self.hold_lock:
                self.held_entity_name = None
                self.held_object_offset = None
            return
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
        if not bool(self.get_parameter('use_gazebo_grasp_assist').value):
            return
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
        configured_h = float(self.get_parameter('place_table_release_height').value)
        place_z = configured_h if z is None else z
        return (shelf[0] + xy_offset[0] + float(adjustment_x), shelf[1] + xy_offset[1], place_z)

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
            'red_cube': 'red_cube',
            'green_cube': 'green_cylinder',
            'blue_cube': 'blue_block',
            'green_cylinder': 'green_cylinder',
            'blue_rectangular_block': 'blue_block',
            'k12_red_cube': 'red_cube',
            'k12_green_cylinder': 'green_cylinder',
            'k12_blue_block': 'blue_block',
        }.get(entity_name or '')
        if slot_key is None:
            name_str = str(entity_name or '').lower()
            if 'red' in name_str:
                slot_key = 'red_cube'
            elif 'green' in name_str:
                slot_key = 'green_cylinder'
            elif 'blue' in name_str:
                slot_key = 'blue_block'
            else:
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
        cube_h = float(self.get_parameter('release_cube_height').value)
        heights = {
            'k12_red_cube': cube_h,
            'red_cube': cube_h,
            'cube_red': cube_h,
            'k12_green_cylinder': float(self.get_parameter('release_cylinder_height').value),
            'green_cylinder': float(self.get_parameter('release_cylinder_height').value),
            'green_cube': cube_h,
            'cube_green': cube_h,
            'k12_blue_block': float(self.get_parameter('release_block_height').value),
            'blue_rectangular_block': float(self.get_parameter('release_block_height').value),
            'blue_cube': cube_h,
            'cube_blue': cube_h,
        }
        object_height = heights.get(entity_name)
        if object_height is None:
            return fallback
        shelf_top_z = float(self._place_table_param('top_z'))
        platform_h = float(self.get_parameter('place_platform_thickness').value) if self.has_parameter('place_platform_thickness') else 0.015
        clearance = max(0.0, float(self.get_parameter('release_clearance').value))
        return shelf_top_z + platform_h + 0.5 * object_height + clearance

    def _holding_object(self):
        with self.hold_lock:
            return self.held_entity_name is not None

    def _verify_grasp_secured(self, expected_entity_name=None):
        if not bool(self.get_parameter('use_gazebo_grasp_assist').value):
            if not self._holding_object():
                raise RuntimeError('No recorded grasp')
            self._verify_finger_contact()
            self.get_logger().info('Finger contact retained after lift')
            return
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
        if frame_id == target_frame:
            return tuple(float(v) for v in position)
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = float(position[0]), float(position[1]), float(position[2])
        try:
            transformed = self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=1.0))
            return (transformed.point.x, transformed.point.y, transformed.point.z)
        except Exception:
            # Fallback to direct lookup_transform with manual rigid-body transform
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame, frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0))
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
                px, py, pz = float(position[0]), float(position[1]), float(position[2])
                tx = r00 * px + r01 * py + r02 * pz + tr.x
                ty = r10 * px + r11 * py + r12 * pz + tr.y
                tz = r20 * px + r21 * py + r22 * pz + tr.z
                return (tx, ty, tz)
            except Exception as exc2:
                self.get_logger().warn(f'TF transform {frame_id}->{target_frame} failed: {exc2}')
                return None

    def _transform_vector_from_map(self, vector, target_frame):
        frame_id = self.get_parameter('place_table_frame').value
        p0 = self._transform_fixed_point(frame_id, (0.0, 0.0, 0.0), target_frame)
        p1 = self._transform_fixed_point(frame_id, vector, target_frame)
        if p0 is None or p1 is None:
            return None
        return (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])

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

    def _wait_for_future(self, future, timeout_sec):
        end_time = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < end_time:
            time.sleep(0.02)
        return future.done()

    def _sleep(self, seconds):
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time and rclpy.ok():
            time.sleep(0.05)

    def _current_base_pose(self):
        frame_id = self.get_parameter('place_table_frame').value
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
