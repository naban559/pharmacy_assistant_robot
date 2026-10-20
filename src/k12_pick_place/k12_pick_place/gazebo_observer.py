"""Gazebo simulation physical observer and truth-verification adapter.

Provides centralized entity mapping, observation caching from Gazebo Sim's
bridged pose stream (/gazebo/pose_info and /gazebo/dynamic_pose_info), velocity
estimation, and strict containment and physical support verification.
"""

import math
import threading
import time
from collections import deque
import numpy as np
from typing import Any, Dict, Optional, Tuple


ENTITY_MAP = {
    'red': {
        'cube': 'cube_red',
        'platform': 'placing_platform_red',
        'platform_center': (-0.080, -2.640, 0.115),
        'platform_size': (0.06, 0.06, 0.015),
        'cube_size': (0.025, 0.025, 0.025),
    },
    'green': {
        'cube': 'cube_green',
        'platform': 'placing_platform_green',
        'platform_center': (0.000, -2.640, 0.115),
        'platform_size': (0.06, 0.06, 0.015),
        'cube_size': (0.025, 0.025, 0.025),
    },
    'blue': {
        'cube': 'cube_blue',
        'platform': 'placing_platform_blue',
        'platform_center': (0.080, -2.640, 0.115),
        'platform_size': (0.06, 0.06, 0.015),
        'cube_size': (0.025, 0.025, 0.025),
    },
}


def normalize_entity_name(raw_name: str) -> str:
    """Extract canonical entity name from hierarchical or scoped Gazebo names."""
    if not raw_name:
        return ''
    cleaned = raw_name.strip()
    # Strip partition or world scope
    if '::' in cleaned:
        cleaned = cleaned.split('::')[-1]
    if '/' in cleaned:
        parts = cleaned.split('/')
        # Filter out common link names
        if parts[-1] in ('link', 'chassis', 'base_link', 'box'):
            cleaned = parts[-2]
        else:
            cleaned = parts[-1]
    return cleaned


def resolve_entities(identifier: str) -> Tuple[str, str, Dict[str, Any]]:
    """Resolve a color, target id, or mission letter to (cube_name, platform_name, metadata)."""
    norm = str(identifier or '').lower().strip()
    letter_map = {'a': 'red', 'b': 'green', 'c': 'blue'}
    if norm in letter_map:
        norm = letter_map[norm]

    for color, data in ENTITY_MAP.items():
        aliases = {color, f'cube_{color}', f'placing_platform_{color}',
                   f'{color}_cube', f'{color}_cylinder', f'{color}_block'}
        if norm in aliases:
            return data['cube'], data['platform'], data
    raise ValueError(f'Unknown placement identity: {identifier!r}')


def pose_matrix(position, orientation):
    """Rigid transform, with explicit finite/unit-quaternion validation."""
    p = np.asarray(position, dtype=float)
    q = np.asarray(orientation, dtype=float)
    if not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid physical pose')
    x, y, z, w = q / np.linalg.norm(q)
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = p
    return m


def transform_matrix(t):
    p, q = t.transform.translation, t.transform.rotation
    return pose_matrix((p.x,p.y,p.z), (q.x,q.y,q.z,q.w))


def matrix_quaternion(matrix):
    """Quaternion (x,y,z,w) for a proper rotation matrix."""
    m = np.asarray(matrix)[:3, :3]
    # Symmetric eigenproblem remains well conditioned at rotations near pi.
    k = np.array([
        [m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]],
        [m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]],
        [m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
        [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3.
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return tuple(q if q[3] >= 0 else -q)


class GazeboObserver:
    """Observes ground truth entity states published from Gazebo Sim."""

    def __init__(self, node: Optional[Any] = None, max_history_sec: float = 5.0):
        self._lock = threading.Lock()
        self.poses: Dict[str, Dict[str, Any]] = {}
        self.history = {}
        self._last_track_log: Dict[str, float] = {}
        self.max_history_sec = max_history_sec
        self.node = node

        if node is not None:
            # Import ROS message types when running within a ROS node
            try:
                from tf2_msgs.msg import TFMessage
                cb_group = getattr(node, 'command_callback_group', None)
                kwargs = {'callback_group': cb_group} if cb_group is not None else {}
                node.create_subscription(
                    TFMessage, '/gazebo/pose_info', self._on_pose_message, 10, **kwargs
                )
                # One stream avoids duplicate samples resetting estimated velocity.
                if hasattr(node, 'get_logger'):
                    node.get_logger().info('GazeboObserver initialized with /gazebo/pose_info subscriptions.')
            except ImportError:
                pass

    def _on_pose_message(self, msg: Any) -> None:
        """Callback for bridged Gazebo tf2_msgs/TFMessage."""
        transforms = getattr(msg, 'transforms', [])
        now_wall = time.monotonic()
        for t in transforms:
            child = getattr(t, 'child_frame_id', '')
            # SceneBroadcaster also emits model-relative link/visual poses with
            # empty parents. Only these exact top-level models are world poses.
            name = child
            allowed = {'four_wheel', 'placing_station'} | {
                data[key] for data in ENTITY_MAP.values() for key in ('cube','platform')}
            if name not in allowed or getattr(t.header, 'frame_id', '') not in ('', 'world', 'robotics_arena'):
                continue

            trans = t.transform.translation
            rot = t.transform.rotation
            pos = (float(trans.x), float(trans.y), float(trans.z))
            quat = (float(rot.x), float(rot.y), float(rot.z), float(rot.w))
            stamp_sec = float(t.header.stamp.sec) + float(t.header.stamp.nanosec) * 1e-9
            if stamp_sec <= 0:
                # Fortress Pose_V->TFMessage drops the envelope timestamp. Use
                # the receiving simulation clock, never declare zero-stamp motion stationary.
                if self.node is None:
                    continue
                stamp_sec = self.node.get_clock().now().nanoseconds * 1e-9
            try:
                pose_matrix(pos, quat)
            except ValueError:
                continue

            with self._lock:
                prev = self.poses.get(name)
                vel = (float('inf'),)*3
                speed = float('inf')
                angular_speed = float('inf')
                if prev is not None:
                    dt = stamp_sec - prev['stamp']
                    if dt <= 1e-4:
                        continue
                    if dt > 1e-4:
                        vel = (
                            (pos[0] - prev['position'][0]) / dt,
                            (pos[1] - prev['position'][1]) / dt,
                            (pos[2] - prev['position'][2]) / dt,
                        )
                        speed = math.hypot(vel[0], vel[1], vel[2])
                        dot = abs(float(np.dot(quat, prev['orientation'])))
                        angular_speed = 2*math.acos(min(1.0, dot))/dt

                self.poses[name] = {
                    'position': pos,
                    'orientation': quat,
                    'stamp': stamp_sec,
                    'wall_time': now_wall,
                    'velocity': vel,
                    'speed': speed,
                    'angular_speed': angular_speed,
                    'frame_id': 'world',
                }
                history = self.history.setdefault(name, deque(maxlen=1000))
                history.append(dict(self.poses[name]))
                while history and stamp_sec-history[0]['stamp'] > self.max_history_sec:
                    history.popleft()
                if name.startswith('cube_') or name.startswith('placing_platform_'):
                    last_log = self._last_track_log.get(name, 0.0)
                    if (now_wall - last_log) >= 5.0 and self.node is not None and hasattr(self.node, 'get_logger'):
                        self._last_track_log[name] = now_wall
                        self.node.get_logger().info(
                            f'[PHYSICAL_TRACK] {name}: world=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) speed={speed:.3f}m/s'
                        )

    def update_pose_manual(
        self,
        name: str,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        stamp_sec: Optional[float] = None,
        velocity: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """Manually update or mock an entity pose (used for testing or direct ingestion)."""
        canon_name = normalize_entity_name(name)
        now_wall = time.monotonic()
        t_stamp = stamp_sec if stamp_sec is not None else now_wall

        with self._lock:
            prev = self.poses.get(canon_name)
            vel = velocity or (0.0, 0.0, 0.0)
            speed = math.hypot(vel[0], vel[1], vel[2]) if velocity else 0.0
            if velocity is None and prev is not None:
                dt = t_stamp - prev['stamp']
                if dt > 1e-4:
                    vel = (
                        (position[0] - prev['position'][0]) / dt,
                        (position[1] - prev['position'][1]) / dt,
                        (position[2] - prev['position'][2]) / dt,
                    )
                    speed = math.hypot(vel[0], vel[1], vel[2])

            self.poses[canon_name] = {
                'position': (float(position[0]), float(position[1]), float(position[2])),
                'orientation': (float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])),
                'stamp': t_stamp,
                'wall_time': now_wall,
                'velocity': vel,
                'speed': speed,
                'angular_speed': 0.0,
                'frame_id': 'world',
            }
            self.history.setdefault(canon_name, deque(maxlen=1000)).append(dict(self.poses[canon_name]))

    def world_to_frame(self, node, target_frame):
        """Actual world -> ROS robot frame, independent of AMCL translation/yaw.

        four_wheel's SDF model root is base_footprint (verified from the live
        SceneBroadcaster model/link poses). TF supplies only the robot-relative
        transform; simulator world is never silently equated with map or odom.
        """
        import rclpy.time
        from rclpy.duration import Duration
        robot = self.get_pose('four_wheel', max_age_sec=.5)
        if robot is None:
            raise RuntimeError('Fresh Gazebo robot world pose unavailable')
        root_to_target = transform_matrix(node.tf_buffer.lookup_transform(
            target_frame, 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=.5)))
        return root_to_target @ np.linalg.inv(pose_matrix(robot['position'], robot['orientation']))

    def world_point_in_frame(self, node, point, target_frame):
        return tuple((self.world_to_frame(node, target_frame) @ np.r_[point, 1.0])[:3])

    def _support_metrics(self, cube_pose, plat_pose):
        relative = np.linalg.inv(pose_matrix(plat_pose['position'], plat_pose['orientation'])) @ pose_matrix(
            cube_pose['position'], cube_pose['orientation'])
        corners = np.array([[x,y,z,1.] for x in (-.0125,.0125)
                            for y in (-.0125,.0125) for z in (-.0125,.0125)])
        projected = (relative @ corners.T).T
        return {'footprint_margin': float(.03-np.max(np.abs(projected[:, :2]))),
                'bottom_gap': float(np.min(projected[:,2])-.0075)}

    def get_pose(self, name: str, max_age_sec: float = 2.0) -> Optional[Dict[str, Any]]:
        """Retrieve the latest pose of an entity if fresh within max_age_sec."""
        canon = normalize_entity_name(name)
        now_wall = time.monotonic()
        with self._lock:
            entry = self.poses.get(canon)
            if entry is None:
                return None
            if max_age_sec is not None and (now_wall - entry['wall_time'] > max_age_sec):
                return None
            return dict(entry)

    def verify_cube_above_platform(
        self,
        cube_name: str,
        platform_name: str,
        max_horizontal_error: float = 0.0145,  # 3mm margin for 60mm platform & 25mm cube
        min_gap: float = 0.001,
        max_gap: float = 0.020,
        max_age_sec: float = 2.0,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        """Verify the held cube is centered and hovering at release clearance above the platform.

        Rejects missing observations, ground-level cube, hovering too high, or lateral overhang.
        """
        cube_pose = self.get_pose(cube_name, max_age_sec=max_age_sec)
        plat_pose = self.get_pose(platform_name, max_age_sec=max_age_sec)

        if cube_pose is None:
            return False, f'Pre-release verification failed: {cube_name} measurement unavailable or stale.', {}
        if plat_pose is None:
            return False, f'Pre-release verification failed: {platform_name} measurement unavailable or stale.', {}

        c_pos = cube_pose['position']
        p_pos = plat_pose['position']

        dx = c_pos[0] - p_pos[0]
        dy = c_pos[1] - p_pos[1]
        horiz_err = math.hypot(dx, dy)

        # Platform is 15mm thick, top is z + 0.0075. Cube is 25mm tall, bottom is z - 0.0125.
        plat_top_z = p_pos[2] + 0.0075
        cube_bottom_z = c_pos[2] - 0.0125
        gap = cube_bottom_z - plat_top_z

        metrics = {
            'dx': dx,
            'dy': dy,
            'horiz_err': horiz_err,
            'gap': gap,
            'cube_pos': c_pos,
            'plat_pos': p_pos,
            'plat_top_z': plat_top_z,
            'cube_bottom_z': cube_bottom_z,
        }
        metrics.update(self._support_metrics(cube_pose, plat_pose))
        gap = metrics['bottom_gap']
        metrics['gap'] = gap
        if metrics['footprint_margin'] < .003-1e-9:
            return False, 'Pre-release lateral containment check failed: oriented footprint margin below 3mm.', metrics

        # Check horizontal containment with 3mm margin
        if horiz_err > max_horizontal_error:
            return False, (
                f'Pre-release lateral containment check failed: offset dx={dx*1000:+.1f}mm, dy={dy*1000:+.1f}mm '
                f'exceeds safe full-support threshold ({max_horizontal_error*1000:.1f}mm) with 3mm margin.'
            ), metrics

        # Check vertical height: cube bottom must be hovering slightly above platform top
        if gap < min_gap:
            return False, (
                f'Pre-release vertical check failed: cube bottom Z={cube_bottom_z:.4f}m is below platform top '
                f'or resting/colliding (gap={gap*1000:+.1f}mm < {min_gap*1000:.1f}mm).'
            ), metrics

        if gap > max_gap:
            return False, (
                f'Pre-release vertical check failed: cube bottom Z={cube_bottom_z:.4f}m is hovering too high '
                f'(gap={gap*1000:+.1f}mm > {max_gap*1000:.1f}mm); risk of dropping or unguided release.'
            ), metrics

        return True, (
            f'Pre-release verified: {cube_name} centered above {platform_name} '
            f'(dx={dx*1000:+.1f}mm, dy={dy*1000:+.1f}mm, gap={gap*1000:+.1f}mm).'
        ), metrics

    def verify_cube_resting_on_platform(
        self,
        cube_name: str,
        platform_name: str,
        max_horizontal_error: float = 0.0145,  # 3mm margin
        max_height_error: float = 0.003,       # within 3mm of resting on platform top
        max_speed: float = 0.010,              # < 10 mm/s linear velocity
        max_age_sec: float = 2.0,
        stability_duration: float = 0.0,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        """Verify the cube has settled and is physically supported on top of the platform."""
        cube_pose = self.get_pose(cube_name, max_age_sec=max_age_sec)
        plat_pose = self.get_pose(platform_name, max_age_sec=max_age_sec)

        if cube_pose is None:
            return False, f'Post-release verification failed: {cube_name} measurement unavailable or stale.', {}
        if plat_pose is None:
            return False, f'Post-release verification failed: {platform_name} measurement unavailable or stale.', {}

        c_pos = cube_pose['position']
        p_pos = plat_pose['position']

        dx = c_pos[0] - p_pos[0]
        dy = c_pos[1] - p_pos[1]
        horiz_err = math.hypot(dx, dy)

        plat_top_z = p_pos[2] + 0.0075
        cube_bottom_z = c_pos[2] - 0.0125
        height_diff = cube_bottom_z - plat_top_z
        speed = cube_pose.get('speed', 0.0)

        metrics = {
            'dx': dx,
            'dy': dy,
            'horiz_err': horiz_err,
            'height_diff': height_diff,
            'speed': speed,
            'cube_pos': c_pos,
            'plat_pos': p_pos,
            'plat_top_z': plat_top_z,
            'cube_bottom_z': cube_bottom_z,
        }
        metrics.update(self._support_metrics(cube_pose, plat_pose))
        if metrics['footprint_margin'] < .003-1e-9:
            return False, 'Post-release oriented footprint not fully supported.', metrics
        if horiz_err > max_horizontal_error:
            return False, f'Post-release centre error {horiz_err*1000:.2f}mm exceeds {max_horizontal_error*1000:.1f}mm.', metrics

        # Check horizontal containment
        if horiz_err > max_horizontal_error:
            return False, (
                f'Post-release lateral check failed: offset dx={dx*1000:+.1f}mm, dy={dy*1000:+.1f}mm '
                f'outside supported surface ({max_horizontal_error*1000:.1f}mm).'
            ), metrics

        # Check support height (cube bottom must rest on platform top)
        if abs(height_diff) > max_height_error or abs(metrics['bottom_gap']) > max_height_error:
            return False, (
                f'Post-release support height check failed: cube bottom Z={cube_bottom_z:.4f}m deviates from '
                f'platform top Z={plat_top_z:.4f}m by {height_diff*1000:+.1f}mm (tol={max_height_error*1000:.1f}mm).'
            ), metrics

        # Check physical stability (motion settled)
        if speed > max_speed:
            return False, (
                f'Post-release stability check failed: cube speed={speed*1000:.1f}mm/s exceeds limit {max_speed*1000:.1f}mm/s.'
            ), metrics

        if stability_duration > 0:
            with self._lock:
                samples = list(self.history.get(normalize_entity_name(cube_name), ()))
            cutoff = cube_pose['stamp']-stability_duration
            recent = [s for s in samples if s['stamp'] >= cutoff-.1]
            if not recent or recent[0]['stamp'] > cutoff or any(
                s['speed'] > max_speed or s.get('angular_speed', float('inf')) > .05
                or math.dist(s['position'], cube_pose['position']) > .002 for s in recent):
                return False, 'Post-release stability dwell not yet satisfied.', metrics

        return True, (
            f'Post-release verified: {cube_name} physically resting on {platform_name} '
            f'(dx={dx*1000:+.1f}mm, dy={dy*1000:+.1f}mm, height_err={height_diff*1000:+.1f}mm, speed={speed*1000:.1f}mm/s).'
        ), metrics
