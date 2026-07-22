import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class ArmGeometry:
    # CHANGED: rescaled to match the arm as rebuilt in articubot_one's
    # description/arm.xacro (fitted to the actual rover chassis, ~0.30x the
    # original K12 dimensions, now scaled by 1.5x). These MUST stay in sync with arm.xacro's
    # k12_arm_* xacro properties -- if you resize the URDF arm again,
    # update these to match or the IK solver will target the wrong points.
    shoulder_offset: float = 0.04168125
    shoulder_height: float = 0.1053
    upper_length: float = 0.23911875
    forearm_length: float = 0.19524375
    wrist_length: float = 0.08994375
    roll_length: float = 0.04168125
    palm_length: float = 0.05265
    finger_length: float = 0.05923125
    tool_center_offset: float = 0.20401875


@dataclass(frozen=True)
class JointLimits:
    yaw: Tuple[float, float] = (-2.70, 2.70)
    shoulder: Tuple[float, float] = (-1.75, 1.45)
    elbow: Tuple[float, float] = (-2.45, 2.45)
    wrist: Tuple[float, float] = (-1.90, 1.90)
    roll: Tuple[float, float] = (-3.14159, 3.14159)


def within(value: float, limits: Tuple[float, float]) -> bool:
    return limits[0] <= value <= limits[1]


def clamp(value: float, limits: Tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def distance(point: Iterable[float]) -> float:
    x, y, z = point
    return math.sqrt(x * x + y * y + z * z)


class PlanarArmIK:
    """Simple radial IK for the K12 5-DOF arm.

    The model keeps the gripper approximately horizontal and points the
    wrist along the radial yaw direction from arm_base_link to the target.
    It is intentionally lightweight for the Gazebo demo; MoveIt 2 can replace
    this module later without changing the perception stack.
    """

    def __init__(self, geometry: ArmGeometry = ArmGeometry(), limits: JointLimits = JointLimits()):
        self.geometry = geometry
        self.limits = limits

    def is_reachable(self, point_arm_base: Iterable[float]) -> bool:
        return self.solve(point_arm_base) is not None

    def solve(self, point_arm_base: Iterable[float]) -> Optional[Tuple[float, float, float, float, float]]:
        x, y, z = [float(v) for v in point_arm_base]
        yaw = math.atan2(y, x)
        if not within(yaw, self.limits.yaw):
            return None

        radial = math.hypot(x, y)
        wrist_r = radial - self.geometry.shoulder_offset - self.geometry.tool_center_offset
        wrist_z_down = self.geometry.shoulder_height - z

        l1 = self.geometry.upper_length
        l2 = self.geometry.forearm_length
        reach = math.hypot(wrist_r, wrist_z_down)
        if reach > l1 + l2 or reach < abs(l1 - l2):
            return None

        d = (wrist_r * wrist_r + wrist_z_down * wrist_z_down - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        d = max(-1.0, min(1.0, d))

        candidates = []
        for elbow in (math.acos(d), -math.acos(d)):
            shoulder = math.atan2(wrist_z_down, wrist_r) - math.atan2(
                l2 * math.sin(elbow),
                l1 + l2 * math.cos(elbow),
            )
            wrist = -(shoulder + elbow)
            roll = 0.0

            if (
                within(shoulder, self.limits.shoulder)
                and within(elbow, self.limits.elbow)
                and within(wrist, self.limits.wrist)
            ):
                score = abs(shoulder) + 0.7 * abs(elbow) + 0.5 * abs(wrist)
                candidates.append((score, (yaw, shoulder, elbow, wrist, roll)))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def solve_clamped(self, point_arm_base: Iterable[float]) -> Tuple[float, float, float, float, float]:
        solution = self.solve(point_arm_base)
        if solution is not None:
            return solution

        x, y, z = [float(v) for v in point_arm_base]
        yaw = clamp(math.atan2(y, x), self.limits.yaw)
        radial = max(0.195, min(0.78, math.hypot(x, y)))
        z = max(-0.04875, min(0.34125, z))
        fallback = self.solve((radial * math.cos(yaw), radial * math.sin(yaw), z))
        if fallback is not None:
            return fallback
        return yaw, 0.0, 0.0, 0.0, 0.0
