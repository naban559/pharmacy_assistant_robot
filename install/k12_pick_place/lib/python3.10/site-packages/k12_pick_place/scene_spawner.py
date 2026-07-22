import math
import os
import sys
import subprocess
from typing import Iterable, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node


def parse_xacro_file(path):
    travel = 0.0405
    gap = 0.0135
    try:
        with open(path, 'r') as f:
            for line in f:
                if 'k12_gripper_finger_travel' in line and 'value=' in line:
                    parts = line.split('value="')
                    if len(parts) > 1:
                        travel = float(parts[1].split('"')[0])
                elif 'k12_gripper_finger_gap' in line and 'value=' in line:
                    parts = line.split('value="')
                    if len(parts) > 1:
                        gap = float(parts[1].split('"')[0])
    except Exception:
        pass
    return travel, gap


def get_gripper_params():
    try:
        share_dir = get_package_share_directory('articubot_one')
        path = os.path.join(share_dir, 'description', 'arm.xacro')
        if os.path.exists(path):
            return parse_xacro_file(path)
    except Exception:
        pass
    
    path = '/home/nabhan/try_10/src/articubot_one_with_arm/description/arm.xacro'
    if os.path.exists(path):
        return parse_xacro_file(path)
    
    return 0.0405, 0.0135


class SceneSpawner(Node):
    def __init__(self):
        super().__init__('amms_scene_spawner')
        self.declare_parameter('spawn_tables', True)
        self.declare_parameter('spawn_objects', True)
        self.declare_parameter('pickup_table_position', [1.5, 0.0, 0.0])
        self.declare_parameter('place_table_position', [3.5, 0.0, 0.0])
        self.declare_parameter('spawn_timeout_sec', 120.0)
        self.client = self.create_client(GetEntityState, '/get_entity_state')
        self.package_share = get_package_share_directory('k12_pick_place')

    def run(self) -> bool:
        timeout = float(self.get_parameter('spawn_timeout_sec').value)
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Ignition Gazebo / Gazebo Sim detected (/get_entity_state service unavailable). Pre-spawned world SDF objects will be used.')
            return True

        pickup = tuple(float(v) for v in self.get_parameter('pickup_table_position').value)
        place = tuple(float(v) for v in self.get_parameter('place_table_position').value)
        
        # Parse gripper parameters and calculate cube size
        travel, gap = get_gripper_params()
        cube_size = gap + 0.75 * (2.0 * travel)
        self.get_logger().info(f'Parsed gripper travel={travel:.4f}, gap={gap:.4f}. Calculated cube size={cube_size:.4f}')

        # Create directories and write custom cube models
        self._write_cube_models(cube_size)

        models = []
        if bool(self.get_parameter('spawn_tables').value):
            models.extend([
                ('k12_object_table', 'k12_pick_table', pickup, 0.0),
                ('k12_place_table', 'k12_pick_table', place, 0.0),
            ])
        if bool(self.get_parameter('spawn_objects').value):
            # Place cubes neatly on the table at 10cm height (0.10m)
            # z position is table_top (0.10) + half cube height + small margin
            object_z = 0.10 + 0.5 * cube_size + 0.005
            object_x = pickup[0] - 0.06
            models.extend([
                ('k12_red_cube', 'k12_red_cube', (object_x, pickup[1] + 0.08, object_z), 0.0),
                ('k12_green_cylinder', 'k12_green_cylinder', (object_x, pickup[1], object_z), 0.0),
                ('k12_blue_block', 'k12_blue_block', (object_x, pickup[1] - 0.08, object_z), 0.0),
            ])

        for entity, model_dir, xyz, yaw in models:
            if not self._spawn(entity, model_dir, xyz, yaw, timeout):
                return False
        self.get_logger().info('AMMS scene spawn complete.')
        return True

    def _write_cube_models(self, size: float):
        # Calculate mass scaled by 1.5^3 from original 0.18 kg
        mass = 0.18 * 3.375
        # Cube inertia: 1/6 * mass * size^2
        inertia = (1.0 / 6.0) * mass * (size ** 2)

        cubes = {
            'k12_red_cube': {
                'color_ambient': '0.95 0.05 0.05 1',
                'color_diffuse': '0.95 0.05 0.05 1',
            },
            'k12_green_cylinder': {
                'color_ambient': '0.05 0.85 0.05 1',
                'color_diffuse': '0.05 0.85 0.05 1',
            },
            'k12_blue_block': {
                'color_ambient': '0.05 0.05 0.95 1',
                'color_diffuse': '0.05 0.05 0.95 1',
            }
        }

        for name, cfg in cubes.items():
            model_dir = os.path.join(self.package_share, 'models', name)
            os.makedirs(model_dir, exist_ok=True)

            sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass:.4f}</mass>
        <inertia>
          <ixx>{inertia:.6f}</ixx>
          <iyy>{inertia:.6f}</iyy>
          <izz>{inertia:.6f}</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <velocity_decay>
        <linear>0.6</linear>
        <angular>1.2</angular>
      </velocity_decay>
      <collision name="collision">
        <geometry>
          <box>
            <size>{size:.4f} {size:.4f} {size:.4f}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>3.0</mu>
              <mu2>3.0</mu2>
              <friction_model>pyramid_model</friction_model>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>5000</kp>
              <kd>20</kd>
              <max_vel>0.1</max_vel>
              <min_depth>0.0005</min_depth>
            </ode>
          </contact>
          <bounce>
            <restitution_coefficient>0.0</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size:.4f} {size:.4f} {size:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{cfg['color_ambient']}</ambient>
          <diffuse>{cfg['color_diffuse']}</diffuse>
          <specular>0.10 0.02 0.02 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
            config_content = f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Antigravity</name>
    <email>antigravity@gemini.com</email>
  </author>
  <description>Dynamically generated cube</description>
</model>
"""
            with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
                f.write(sdf_content)
            with open(os.path.join(model_dir, 'model.config'), 'w') as f:
                f.write(config_content)

    def _spawn(
        self,
        entity: str,
        model_dir: str,
        xyz: Iterable[float],
        yaw: float,
        timeout: float,
    ) -> bool:
        model_path = os.path.join(self.package_share, 'models', model_dir, 'model.sdf')
        x, y, z = xyz
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', model_path,
            '-name', entity,
            '-x', f'{x:.4f}',
            '-y', f'{y:.4f}',
            '-z', f'{z:.4f}',
            '-Y', f'{yaw:.4f}',
        ]
        self.get_logger().info(f"Spawning {entity} using command: {' '.join(cmd)}")
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode == 0:
            self.get_logger().info(f'Successfully spawned {entity}.')
            return True
        
        # Check if it failed because it already exists
        if 'already exists' in res.stderr.lower() or 'already exists' in res.stdout.lower():
            self.get_logger().warn(f'{entity} already exists; continuing.')
            return True

        self.get_logger().error(f'Could not spawn {entity}: {res.stderr or res.stdout}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SceneSpawner()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
