#!/usr/bin/env python3
"""Offline station/URDF admission checks, not a Gazebo physics simulation.

Run from any directory: python3 tools/verify_station_workspace.py --trials 200
Reads the source SDF, robot dimensions, and live YAML parameters. Tests exact
IK, complete approach/lift paths, independent URDF forward kinematics, camera
frustum, table containment, and chassis clearance at perturbed docking poses.
It does not establish contact stability, collision-free arm transit, camera
occlusion, localization accuracy, or an empirical simulation success rate.
"""
import argparse
import ast
import itertools
import json
import math
import operator
from pathlib import Path
import random
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent
sys.path.insert(0, str(PACKAGE))
from k12_pick_place.ik import PlanarArmIK  # noqa: E402

XACRO = '{http://www.ros.org/wiki/xacro}'


def numbers(value):
    return np.array([float(v) for v in value.split()])


def expression(value, properties):
    """Resolve numeric xacro origins without importing ROS or executing code."""
    operations = {ast.Add: operator.add, ast.Sub: operator.sub,
                  ast.Mult: operator.mul, ast.Div: operator.truediv}

    def numeric(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return node.value
        if isinstance(node, ast.Name):
            return properties[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in operations:
            return operations[type(node.op)](numeric(node.left), numeric(node.right))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -numeric(node.operand)
        raise ValueError(f'Unsupported numeric xacro expression: {value}')

    return re.sub(r'\$\{([^}]+)\}',
                  lambda match: str(numeric(ast.parse(match[1], mode='eval').body)), value)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


class StationGeometry:
    def __init__(self):
        description = SOURCE / 'articubot_one_with_arm/description'
        self.robot = ET.parse(description / 'four_wheel.xacro').getroot()
        self.arm = ET.parse(description / 'arm.xacro').getroot()
        self.gripper = ET.parse(description / 'gripper.xacro').getroot()
        self.world = ET.parse(SOURCE / 'articubot_one_with_arm/worlds/og.world').getroot()
        self.models = {m.get('name'): m for m in self.world.findall('world/model')}
        self.task = yaml.safe_load((PACKAGE / 'config/amms_waypoints.yaml').read_text())[
            'amms_task_manager']['ros__parameters']
        self.commander = yaml.safe_load((PACKAGE / 'config/pick_place_service.yaml').read_text())[
            'k12_pick_place_commander']['ros__parameters']
        self.properties = {'pi': math.pi}
        for prop in self.arm.findall(f'{XACRO}property'):
            try:
                self.properties[prop.get('name')] = float(prop.get('value'))
            except ValueError:
                pass
        self.properties['palm_length'] = self.properties['k12_gripper_palm_length']
        self.properties['finger_length'] = self.properties['k12_gripper_finger_length']
        self.base_height = float(self.robot.find(
            "joint[@name='base_footprint_to_base_link']/origin").get('xyz').split()[2])
        mount = self.robot.find(f'{XACRO}k12_arm')
        self.mount = np.array([float(mount.get(f'mount_{axis}')) for axis in 'xyz'])
        camera_joint = next(
            j for j in self.robot.findall('joint') if j.find('child').get('link') == 'camera_1')
        self.camera = numbers(camera_joint.find('origin').get('xyz'))
        camera_rpy = numbers(camera_joint.find('origin').get('rpy'))
        self.camera_rotation = (rotation([0, 0, 1], camera_rpy[2])
                                @ rotation([0, 1, 0], camera_rpy[1])
                                @ rotation([1, 0, 0], camera_rpy[0]))
        sensor = ET.parse(description / 'four_wheel.gazebo').find("gazebo[@reference='camera_1']/sensor/camera")
        self.hfov = float(sensor.findtext('horizontal_fov'))
        self.width = int(sensor.findtext('image/width'))
        self.height = int(sensor.findtext('image/height'))
        self.focal = self.width / (2 * math.tan(self.hfov / 2))
        self.near = float(sensor.findtext('clip/near'))
        self.far = float(sensor.findtext('clip/far'))
        self.chassis_half = numbers(self.robot.find("link[@name='base_link']/collision/geometry/box").get('size')) / 2
        self.wheels = []
        for joint in self.robot.findall('joint'):
            if not joint.get('name').endswith('_wheel_joint'):
                continue
            child = joint.find('child').get('link')
            cylinder = self.robot.find(f"link[@name='{child}']/collision/geometry/cylinder")
            self.wheels.append((numbers(joint.find('origin').get('xyz')),
                                float(cylinder.get('radius')), float(cylinder.get('length'))))
        self.ik = PlanarArmIK()

    def model_geometry(self, name):
        model = self.models[name]
        return numbers(model.findtext('pose'))[:3], numbers(model.findtext('link/collision/geometry/box/size'))

    def point_in_base(self, world, dock):
        delta = np.array(world) - np.array([dock[0], dock[1], self.base_height])
        return rotation([0, 0, 1], -dock[2]) @ delta

    def point_in_arm(self, world, dock):
        return self.point_in_base(world, dock) - self.mount

    def independent_fk(self, joints):
        """Compose actual XML joint origins/axes independently of the IK formula."""
        transform = np.eye(4)
        names = ['arm_base_yaw_joint', 'arm_shoulder_pitch_joint', 'arm_elbow_pitch_joint',
                 'arm_wrist_pitch_joint', 'arm_wrist_roll_joint', 'end_effector_joint', 'gripper_tcp_joint']
        for name, angle in zip(names, list(joints) + [0, 0]):
            joint = self.arm.find(f".//joint[@name='{name}']")
            if joint is None:
                joint = self.gripper.find(f".//joint[@name='{name}']")
            origin = joint.find('origin')
            step = np.eye(4)
            step[:3, 3] = numbers(expression(origin.get('xyz'), self.properties))
            rpy = numbers(expression(origin.get('rpy', '0 0 0'), self.properties))
            step[:3, :3] = (rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1])
                            @ rotation([1, 0, 0], rpy[0]))
            axis = joint.find('axis')
            if axis is not None:
                step[:3, :3] = step[:3, :3] @ rotation(numbers(axis.get('xyz')), angle)
            transform = transform @ step
        return transform[:3, 3]

    def camera_pixels(self, center, size, dock):
        pixels = []
        for signs in itertools.product((-1, 1), repeat=3):
            corner = center + size / 2 * signs
            physical = self.camera_rotation.T @ (self.point_in_base(corner, dock) - self.camera)
            depth = physical[0]
            if not self.near < depth < self.far:
                return None
            pixels.append((self.width / 2 - self.focal * physical[1] / depth,
                           self.height / 2 - self.focal * physical[2] / depth))
        pixels = np.array(pixels)
        if np.any(pixels.min(axis=0) < 0) or np.any(pixels.max(axis=0) >= [self.width, self.height]):
            return None
        return pixels

    def base_clearance(self, station, dock):
        center, size = self.model_geometry(station)
        facing = 1 if center[1] > 0 else -1
        half_along_y = abs(math.sin(dock[2])) * self.chassis_half[0] + abs(math.cos(dock[2])) * self.chassis_half[1]
        chassis = facing * (center[1] - dock[1]) - size[1] / 2 - half_along_y
        # Cylindrical wheels extend farther forward than the chassis. Their
        # axis is local Y, so projected radius along world Y is r*abs(sin(yaw)).
        base_rotation = rotation([0, 0, 1], dock[2])
        wheel_edge = max(facing * (base_rotation @ position)[1]
                         + radius * abs(math.sin(dock[2])) + length / 2 * abs(math.cos(dock[2]))
                         for position, radius, length in self.wheels)
        wheel = facing * (center[1] - dock[1]) - size[1] / 2 - wheel_edge
        return chassis, wheel

    def cases(self):
        for index, color in enumerate(('red', 'green', 'blue'), 1):
            station = f'pickup_point_{index}_{color}'
            center, size = self.model_geometry(f'cube_{color}')
            grasp = center + [0, 0, self.task['object_grasp_z_bias']]
            nominal = np.array(self.task[f'waypoint_{station}'])
            standoff = float(self.task.get('object_nav_standoff_distance', 0.44))
            if center[1] > 0 and nominal[1] < center[1] - standoff:
                docked = np.array([center[0], center[1] - standoff, nominal[2]])
            else:
                docked = nominal
            yield f'pickup_{color}', station, docked, grasp, center, size, .05, .12
        center, _ = self.model_geometry('placing_station')
        base_offset = self.commander['place_table_release_xy_offset']
        for color, key in [('red', 'red_cube'), ('green', 'green_cylinder'), ('blue', 'blue_block')]:
            offset = self.commander[f'place_table_release_{key}_xy_offset']
            cube_size = self.model_geometry(f'cube_{color}')[1]
            target = np.array([center[0] + base_offset[0] + offset[0],
                               center[1] + base_offset[1] + offset[1],
                               self.commander['place_table_top_z'] + cube_size[2] / 2
                               + self.commander['grasp_z_bias'] + self.commander['release_clearance']])
            yield f'place_{color}', 'placing_station', self.task['waypoint_place_station'], target, None, cube_size, .025, .025


def verify(trials=200, translation=.010, yaw=.03, seed=18):
    geometry = StationGeometry()
    rng = random.Random(seed)
    offsets = list(itertools.product((-translation, 0, translation),
                                     (-translation, 0, translation), (-yaw, 0, yaw)))
    offsets += [(rng.uniform(-translation, translation), rng.uniform(-translation, translation),
                 rng.uniform(-yaw, yaw)) for _ in range(trials)]
    report = {'kind': 'offline geometry and kinematics (not physics simulation)',
              'translation_bound_per_axis_m': translation, 'yaw_bound_rad': yaw,
              'seed': seed, 'cases': {}, 'failures': []}
    for name, station, nominal, target, cube, size, approach, lift in geometry.cases():
        support, support_size = geometry.model_geometry(station)
        support_margin = min(support_size[:2] / 2 - np.abs(target[:2] - support[:2]) - size[:2] / 2)
        stats = {'trials': len(offsets), 'passed': 0, 'minimum_chassis_clearance_m': math.inf,
                 'minimum_wheel_clearance_m': math.inf,
                 'maximum_fk_error_m': 0.0, 'minimum_projected_bbox_area_px': math.inf,
                 'support_edge_margin_m': float(support_margin),
                 'nominal_tcp_arm_m': geometry.point_in_arm(target, nominal).tolist()}
        if support_margin < 0:
            report['failures'].append(f'{name}: cube overhangs its support')
        for offset in offsets:
            dock = np.array(nominal) + offset
            point = geometry.point_in_arm(target, dock)
            clearance, wheel_clearance = geometry.base_clearance(station, dock)
            stats['minimum_chassis_clearance_m'] = min(stats['minimum_chassis_clearance_m'], clearance)
            stats['minimum_wheel_clearance_m'] = min(stats['minimum_wheel_clearance_m'], wheel_clearance)
            plan = geometry.ik.pick_plan(point, approach=approach, lift=lift)
            error = 0.0
            if plan is not None:
                pre = plan['pre_grasp_joints']
                grasp = geometry.ik.solve(point, pitch=plan['pitch'], seed=pre)
                lifted = geometry.ik.solve(plan['lift'], pitch=plan['pitch'], seed=grasp)
                error = max(float(np.linalg.norm(geometry.independent_fk(q) - expected))
                            for q, expected in [(pre, plan['pre_grasp']), (grasp, point), (lifted, plan['lift'])])
                stats['maximum_fk_error_m'] = max(stats['maximum_fk_error_m'], error)
            visible = True
            if cube is not None:
                pixels = geometry.camera_pixels(cube, size, dock)
                visible = pixels is not None
                if visible:
                    stats['minimum_projected_bbox_area_px'] = min(stats['minimum_projected_bbox_area_px'],
                                                                  float(np.prod(np.ptp(pixels, axis=0))))
            if plan is not None and min(clearance, wheel_clearance) > 0 and error < 1e-8 and visible and support_margin >= 0:
                stats['passed'] += 1
            else:
                report['failures'].append(f'{name} offset={offset}: IK={plan is not None}, '
                                          f'chassis_clearance={clearance:.4f}, wheel_clearance={wheel_clearance:.4f}, '
                                          f'camera_frustum={visible}, FK_error={error:.3g}')
        if not math.isfinite(stats['minimum_projected_bbox_area_px']):
            stats['minimum_projected_bbox_area_px'] = None
        report['cases'][name] = stats
    finger_gap = geometry.properties['k12_gripper_finger_gap'] + 2 * geometry.properties['k12_gripper_finger_travel']
    cube_size = geometry.model_geometry('cube_red')[1][1]
    report['gripper_open_gap_m'] = finger_gap
    report['centered_cube_lateral_allowance_per_side_m'] = float((finger_gap - cube_size) / 2)
    report['passed'] = not report['failures']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=200, help='random perturbations per case, plus 27 boundary cases')
    parser.add_argument('--translation', type=float, default=.010)
    parser.add_argument('--yaw', type=float, default=.03)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.trials < 0 or args.translation < 0 or args.yaw < 0:
        parser.error('trial count and perturbation bounds must be nonnegative')
    report = verify(args.trials, args.translation, args.yaw)
    content = json.dumps(report, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content + '\n')
    print(content)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
