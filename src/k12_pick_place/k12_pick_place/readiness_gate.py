import sys
import time
from typing import Dict, List, Tuple

import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import DriveOnHeading, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class ReadinessGate(Node):
    def __init__(self):
        super().__init__('amms_readiness_gate')
        self.declare_parameter('required_topics', [''])
        self.declare_parameter('required_services', [''])
        self.declare_parameter('required_lifecycle_nodes', [''])
        self.declare_parameter('required_actions', [''])
        self.declare_parameter('required_trajectory_actions', [''])
        self.declare_parameter('required_transforms', [''])
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('poll_period_sec', 0.25)
        self.declare_parameter('lifecycle_recovery_enabled', True)
        self.declare_parameter('lifecycle_recovery_initial_delay_sec', 20.0)
        self.declare_parameter('lifecycle_recovery_period_sec', 5.0)
        self.declare_parameter('lifecycle_transition_timeout_sec', 8.0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lifecycle_clients: Dict[str, object] = {}
        self.lifecycle_change_clients: Dict[str, object] = {}
        self.action_clients: Dict[str, ActionClient] = {}
        self.trajectory_action_clients: Dict[str, ActionClient] = {}

    def wait_until_ready(self) -> bool:
        deadline = time.monotonic() + float(self.get_parameter('timeout_sec').value)
        started_at = time.monotonic()
        last_recovery_at = 0.0
        poll_period = float(self.get_parameter('poll_period_sec').value)
        last_report = ''

        while rclpy.ok() and time.monotonic() < deadline:
            missing = self._missing_requirements()
            if not missing:
                self.get_logger().info('AMMS readiness gate satisfied.')
                return True
            now = time.monotonic()
            if (
                bool(self.get_parameter('lifecycle_recovery_enabled').value)
                and now - started_at >= float(self.get_parameter('lifecycle_recovery_initial_delay_sec').value)
                and now - last_recovery_at >= float(self.get_parameter('lifecycle_recovery_period_sec').value)
            ):
                last_recovery_at = now
                self._recover_lifecycle_nodes()
            report = ', '.join(missing[:6])
            if report != last_report:
                self.get_logger().info(f'Waiting for readiness: {report}')
                last_report = report
            time.sleep(poll_period)

        self.get_logger().error('Timed out waiting for AMMS readiness.')
        return False

    def _missing_requirements(self) -> List[str]:
        missing = []
        missing.extend(self._missing_topics())
        missing.extend(self._missing_services())
        missing.extend(self._missing_lifecycle_nodes())
        missing.extend(self._missing_actions())
        missing.extend(self._missing_trajectory_actions())
        missing.extend(self._missing_transforms())
        return missing

    def _missing_topics(self) -> List[str]:
        missing = []
        for topic in self._string_list('required_topics'):
            if self.count_publishers(topic) == 0:
                missing.append(f'topic {topic}')
        return missing

    def _missing_services(self) -> List[str]:
        available = {name for name, _types in self.get_service_names_and_types()}
        return [
            f'service {service}'
            for service in self._string_list('required_services')
            if service not in available
        ]

    def _missing_lifecycle_nodes(self) -> List[str]:
        missing = []
        for node_name in self._string_list('required_lifecycle_nodes'):
            client = self.lifecycle_clients.get(node_name)
            if client is None:
                client = self.create_client(GetState, f'{node_name}/get_state')
                self.lifecycle_clients[node_name] = client
            if not client.wait_for_service(timeout_sec=0.05):
                missing.append(f'lifecycle {node_name}')
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
            if not future.done():
                missing.append(f'lifecycle {node_name}')
                continue
            response = future.result()
            if response is None or response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                missing.append(f'lifecycle {node_name}')
        return missing

    def _missing_actions(self) -> List[str]:
        missing = []
        for action_name in self._string_list('required_actions'):
            client = self.action_clients.get(action_name)
            if client is None:
                action_type = DriveOnHeading if action_name.endswith('drive_on_heading') else NavigateToPose
                client = ActionClient(self, action_type, action_name)
                self.action_clients[action_name] = client
            if not client.wait_for_server(timeout_sec=0.05):
                missing.append(f'action {action_name}')
        return missing

    def _missing_trajectory_actions(self) -> List[str]:
        missing = []
        for action_name in self._string_list('required_trajectory_actions'):
            client = self.trajectory_action_clients.get(action_name)
            if client is None:
                client = ActionClient(self, FollowJointTrajectory, action_name)
                self.trajectory_action_clients[action_name] = client
            if not client.wait_for_server(timeout_sec=0.05):
                missing.append(f'action {action_name}')
        return missing

    def _missing_transforms(self) -> List[str]:
        missing = []
        for item in self._string_list('required_transforms'):
            target, source = self._parse_transform_requirement(item)
            if not self.tf_buffer.can_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ):
                missing.append(f'tf {target}<-{source}')
        return missing

    def _recover_lifecycle_nodes(self) -> None:
        self.get_logger().warn('Readiness gate attempting Nav2 lifecycle recovery.')
        for node_name in self._string_list('required_lifecycle_nodes'):
            state = self._get_lifecycle_state(node_name)
            if state is None:
                self.get_logger().warn(f'Readiness lifecycle recovery skipped {node_name}: get_state unavailable.')
                continue
            if state.id == State.PRIMARY_STATE_ACTIVE:
                continue
            self.get_logger().warn(
                f'Readiness lifecycle recovery inspecting {node_name}: state={state.label or state.id}.'
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
                    f'Readiness lifecycle recovery cannot transition {node_name} '
                    f'from state={state.label or state.id}.'
                )

    def _get_lifecycle_state(self, node_name: str):
        client = self.lifecycle_clients.get(node_name)
        if client is None:
            client = self.create_client(GetState, f'{node_name}/get_state')
            self.lifecycle_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=0.25):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
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
            self.get_logger().warn(f'Readiness lifecycle recovery cannot reach {node_name}/change_state.')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        label = 'configure' if transition_id == Transition.TRANSITION_CONFIGURE else 'activate'
        self.get_logger().warn(f'Readiness lifecycle recovery requesting {label} for {node_name}.')
        future = client.call_async(request)
        timeout = float(self.get_parameter('lifecycle_transition_timeout_sec').value)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().warn(f'Readiness lifecycle recovery timed out requesting {label} for {node_name}.')
            return False
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warn(f'Readiness lifecycle recovery {label} failed for {node_name}.')
            return False
        self.get_logger().info(f'Readiness lifecycle recovery {label} accepted for {node_name}.')
        return True

    @staticmethod
    def _parse_transform_requirement(item: str) -> Tuple[str, str]:
        if '<-' in item:
            target, source = item.split('<-', 1)
            return target.strip(), source.strip()
        if ':' in item:
            target, source = item.split(':', 1)
            return target.strip(), source.strip()
        raise ValueError(f'Invalid transform requirement: {item}')

    def _string_list(self, parameter_name: str) -> List[str]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        return [str(item) for item in value if str(item)]


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGate()
    try:
        ok = node.wait_until_ready()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
