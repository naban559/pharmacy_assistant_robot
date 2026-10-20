"""URDF-matched kinematics for the K12 arm, targeting gripper_tcp.

All points are in arm_base_link. At zero angles the upper arm points +Z,
while the forearm and tool point +X. Positive pitch rotates +X toward -Z.
"""
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class ArmGeometry:
    shoulder_offset: float = 0.0098
    shoulder_height: float = 0.1100
    upper_length: float = 0.2000
    forearm_length: float = 0.1400
    wrist_length: float = 0.0300
    roll_length: float = 0.0500
    palm_length: float = 0.0350
    finger_length: float = 0.0400
    tool_center_offset: float = 0.1350  # wrist + roll + palm + half finger


@dataclass(frozen=True)
class JointLimits:
    yaw: Tuple[float, float] = (-1.5708, 1.5708)
    shoulder: Tuple[float, float] = (-1.3614, 1.8500)
    elbow: Tuple[float, float] = (-2.40855, 1.3439)
    wrist: Tuple[float, float] = (-1.7802, 1.7802)
    roll: Tuple[float, float] = (-3.14159, 3.14159)

    def ordered(self):
        return self.yaw, self.shoulder, self.elbow, self.wrist, self.roll


def within(value, limits):
    return limits[0] <= value <= limits[1]


def clamp(value, limits):
    return max(limits[0], min(limits[1], value))


def distance(point: Iterable[float]) -> float:
    return math.sqrt(sum(float(v) ** 2 for v in point))


class PlanarArmIK:
    def __init__(self, geometry=ArmGeometry(), limits=JointLimits()):
        self.geometry = geometry
        self.limits = limits

    def valid_joints(self, joints, margin=0.0):
        return len(joints) == 5 and all(
            math.isfinite(q) and lo + margin <= q <= hi - margin
            for q, (lo, hi) in zip(joints, self.limits.ordered()))

    def forward(self, joints):
        yaw, shoulder, elbow, wrist, roll = joints
        g = self.geometry
        pitch = shoulder + elbow + wrist
        radial = (g.shoulder_offset + g.upper_length * math.sin(shoulder)
                  + g.forearm_length * math.cos(shoulder + elbow)
                  + g.tool_center_offset * math.cos(pitch))
        z = (g.shoulder_height + g.upper_length * math.cos(shoulder)
             - g.forearm_length * math.sin(shoulder + elbow)
             - g.tool_center_offset * math.sin(pitch))
        return radial * math.cos(yaw), radial * math.sin(yaw), z

    def solve(self, point_arm_base, pitch=0.0, roll=0.0, seed=None, margin=0.0):
        x, y, z = map(float, point_arm_base)
        if not all(math.isfinite(v) for v in (x, y, z, pitch, roll)):
            return None
        yaw = math.atan2(y, x)
        g = self.geometry
        u = math.hypot(x, y) - g.shoulder_offset - g.tool_center_offset * math.cos(pitch)
        v = g.shoulder_height - z - g.tool_center_offset * math.sin(pitch)
        l1, l2 = g.upper_length, g.forearm_length
        cosine = (u*u + v*v - l1*l1 - l2*l2) / (2*l1*l2)
        if abs(cosine) > 1.0 + 1e-12:
            return None
        candidates = []
        for beta in (math.acos(clamp(cosine, (-1, 1))), -math.acos(clamp(cosine, (-1, 1)))):
            alpha = math.atan2(v, u) - math.atan2(l2 * math.sin(beta), l1 + l2 * math.cos(beta))
            # The two links have different zero-angle axes in the URDF.
            for turns in (-1, 0, 1):
                shoulder = alpha + math.pi / 2 + turns * 2 * math.pi
                elbow = beta - math.pi / 2
                wrist = pitch - shoulder - elbow
                q = (yaw, shoulder, elbow, wrist, roll)
                if self.valid_joints(q, margin) and math.dist(self.forward(q), (x, y, z)) < 1e-8:
                    candidates.append(q)
        if not candidates:
            return None
        reference = seed if seed is not None else (yaw, 0.0, 0.0, 0.0, roll)
        # Bounded revolute joints: compare actual angles, never modulo 2*pi.
        return min(candidates, key=lambda q: sum((a-b)**2 for a, b in zip(q, reference)))

    def is_reachable(self, point_arm_base):
        return self.solve(point_arm_base) is not None

    def solve_clamped(self, point_arm_base):
        solution = self.solve(point_arm_base)
        if solution is None:
            raise ValueError(f'Unreachable TCP target: {tuple(point_arm_base)}')
        return solution

    def vertical_path(self, start_joints, target_z, step=0.001, margin=0.01):
        """Return a complete path or raise; never return a partial grasp path."""
        start = tuple(start_joints)
        if not self.valid_joints(start) or not math.isfinite(target_z) or step <= 0:
            raise ValueError('Invalid vertical path start/height/step')
        x, y, z = self.forward(start)
        pitch, roll = sum(start[1:4]), start[4]
        count = max(1, math.ceil(abs(target_z-z) / step))
        path = [start]
        for i in range(1, count+1):
            height = z + (target_z-z) * i/count
            q = self.solve((x, y, height), pitch, roll, path[-1], margin)
            if q is None or max(abs(a-b) for a, b in zip(q, path[-1])) > 0.10:
                raise ValueError(f'Fixed-orientation path ends at Z={self.forward(path[-1])[2]:.4f}; '
                                 f'requested Z={target_z:.4f} is unreachable or discontinuous')
            # Bound errors of the actual joint-linear interpolation, not just IK samples.
            for fraction in (0.25, 0.5, 0.75):
                mid = tuple(a+(b-a)*fraction for a, b in zip(path[-1], q))
                actual = self.forward(mid)
                if math.hypot(actual[0]-x, actual[1]-y) > 0.0002:
                    raise ValueError('Vertical path exceeds 0.2 mm lateral interpolation tolerance')
            path.append(q)
        return path

    def pick_plan(self, grasp, approach=0.05, lift=0.12, pitches=(math.pi/2,)):
        """Common geometric admission check used by navigation and commanders.

        Use a downward grasp by default; callers may explicitly supply other pitches.
        Never move the grasp
        point to make an unreachable object appear reachable.
        """
        grasp = tuple(map(float, grasp))
        pre = (grasp[0], grasp[1], grasp[2] + approach)
        for pitch in pitches:
            q = self.solve(pre, pitch=pitch, margin=0.03)
            if q is None:
                continue
            try:
                descent = self.vertical_path(q, grasp[2], margin=0.03)
                for height in sorted({lift, min(lift, 0.10), min(lift, 0.08), min(lift, 0.06)}, reverse=True):
                    try:
                        self.vertical_path(descent[-1], grasp[2]+height, margin=0.03)
                    except ValueError:
                        continue
                    return {'grasp': grasp, 'pre_grasp': pre,
                            'lift': (grasp[0], grasp[1], grasp[2]+height),
                            'pre_grasp_joints': q, 'pitch': pitch}
            except ValueError:
                continue
        return None
