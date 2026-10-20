"""AMMS Operator CLI — standalone command interface for A/B/C/D pick-and-place.

Run in a SEPARATE terminal after the launch stack is up:

    ros2 run k12_pick_place amms_command

This script:
  - Queries TF for the robot's current pose
  - Subscribes to /amms/task_state for live state display
  - Calls /amms/goto_{a,b,c,d} Trigger services to send commands
  - Prints the operator banner and prompt

It is a rclpy.node.Node because it needs TF lookups and service clients.
It spins a minimal executor only long enough to get responses, then returns
control to the interactive input loop.
"""

import math
import sys
import threading
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401 — registers geometry transforms
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class AmmsCommandCli(Node):

    VALID_COMMANDS = ('A', 'B', 'C', 'D')
    LABELS = {
        'A': 'Pick Point 1 - RED',
        'B': 'Pick Point 2 - GREEN',
        'C': 'Pick Point 3 - BLUE',
        'D': 'Place Station',
    }

    def __init__(self):
        super().__init__('amms_command_cli')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('task_state_topic', '/amms/task_state')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._current_state = 'UNKNOWN'
        self.create_subscription(
            String,
            self.get_parameter('task_state_topic').value,
            self._on_state, 10)

        self._goto_clients = {}
        for letter in ('a', 'b', 'c', 'd'):
            client = self.create_client(Trigger, f'/amms/goto_{letter}')
            self._goto_clients[letter] = client

    def _on_state(self, msg: String) -> None:
        self._current_state = msg.data

    def get_current_pose(self):
        frame_id = self.get_parameter('frame_id').value
        base_frame = self.get_parameter('base_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                frame_id, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception:
            return None
        rot = t.transform.rotation
        yaw = math.atan2(
            2.0 * (rot.w * rot.z + rot.x * rot.y),
            1.0 - 2.0 * (rot.y ** 2 + rot.z ** 2))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def send_command(self, letter: str) -> bool:
        letter_lower = letter.lower()
        client = self._goto_clients.get(letter_lower)
        if client is None:
            return False
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service /amms/goto_{letter_lower} not available.')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f'Service call /amms/goto_{letter_lower} timed out.')
            return False
        result = future.result()
        if result is None:
            self.get_logger().error(f'Service call /amms/goto_{letter_lower} returned None.')
            return False
        if not result.success:
            self.get_logger().warn(f'Command {letter} rejected: {result.message}')
            print(f'  Rejected: {result.message}', flush=True)
            return False
        self.get_logger().info(f'Command {letter} accepted: {result.message}')
        return True


def _print_banner():
    print('', flush=True)
    print('==================================================', flush=True)
    print('          ARTIGOBOT PICK & PLACE', flush=True)
    print('==================================================', flush=True)
    print('', flush=True)


def _print_prompt(cli: AmmsCommandCli):
    print('', flush=True)
    print(f'  State: {cli._current_state}', flush=True)
    print('', flush=True)

    pose = cli.get_current_pose()
    if pose is not None:
        print('  Where am I?', flush=True)
        print(f'    x:   {pose[0]:.2f}', flush=True)
        print(f'    y:   {pose[1]:.2f}', flush=True)
        print(f'    yaw: {pose[2]:.2f} rad', flush=True)
    else:
        print('  Where am I? (localization unavailable)', flush=True)

    print('', flush=True)
    print('  Available destinations:', flush=True)
    print('', flush=True)
    print('    A  Pick Point 1 - RED', flush=True)
    print('    B  Pick Point 2 - GREEN', flush=True)
    print('    C  Pick Point 3 - BLUE', flush=True)
    print('    D  Place Station', flush=True)
    print('', flush=True)
    print('  Enter A, B, C, or D: ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    cli = AmmsCommandCli()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(cli)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _print_banner()
        while rclpy.ok():
            _print_prompt(cli)
            try:
                raw = input().strip().upper()
            except (EOFError, KeyboardInterrupt):
                print('\n  Exiting.', flush=True)
                break

            if raw not in cli.VALID_COMMANDS:
                if raw:
                    print(f'\n  Invalid command "{raw}".', flush=True)
                    print('  Please enter A, B, C, or D.', flush=True)
                continue

            print(f'\n  Sending command {raw} ({cli.LABELS[raw]})...', flush=True)
            ok = cli.send_command(raw)
            if ok:
                print(f'  Command {raw} accepted. Monitoring state...', flush=True)
                deadline = time.monotonic() + 5.0
                initial_state = cli._current_state
                while rclpy.ok() and time.monotonic() < deadline:
                    if cli._current_state != initial_state and cli._current_state != 'WAIT_FOR_COMMAND':
                        print(f'  State: {cli._current_state}', flush=True)
                        break
                    time.sleep(0.1)
            else:
                print(f'  Command {raw} failed.', flush=True)
    finally:
        executor.shutdown()
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
