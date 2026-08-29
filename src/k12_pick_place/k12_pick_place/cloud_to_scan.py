import math
from typing import List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class CloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        self.declare_parameter('cloud_topic', '/velodyne_points')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'lidar_1')
        self.declare_parameter('transform_tolerance', 0.05)
        self.declare_parameter('min_height', -0.25)
        self.declare_parameter('max_height', 0.35)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.0087)
        self.declare_parameter('scan_time', 0.10)
        self.declare_parameter('range_min', 0.25)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('use_inf', True)
        self.declare_parameter('min_points_per_beam', 1)
        self.declare_parameter('use_median_range', True)
        self.declare_parameter('reject_isolated_returns', True)
        self.declare_parameter('isolation_window', 2)
        self.declare_parameter('isolation_max_range_delta', 0.75)
        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('self_filter_frame', 'base_link')
        self.declare_parameter('self_filter_min_x', -0.75)
        self.declare_parameter('self_filter_max_x', 0.64)
        self.declare_parameter('self_filter_min_y', -0.50)
        self.declare_parameter('self_filter_max_y', 0.50)
        self.declare_parameter('self_filter_min_z', -0.05)
        self.declare_parameter('self_filter_max_z', 0.65)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter('scan_topic').value,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self._last_transform_warning_sec = 0.0

    def _on_cloud(self, msg: PointCloud2) -> None:
        angle_min = float(self.get_parameter('angle_min').value)
        angle_max = float(self.get_parameter('angle_max').value)
        angle_increment = float(self.get_parameter('angle_increment').value)
        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        min_height = float(self.get_parameter('min_height').value)
        max_height = float(self.get_parameter('max_height').value)
        use_inf = bool(self.get_parameter('use_inf').value)
        min_points_per_beam = max(1, int(self.get_parameter('min_points_per_beam').value))
        use_median_range = bool(self.get_parameter('use_median_range').value)
        reject_isolated_returns = bool(self.get_parameter('reject_isolated_returns').value)
        isolation_window = max(1, int(self.get_parameter('isolation_window').value))
        isolation_max_range_delta = float(self.get_parameter('isolation_max_range_delta').value)
        target_frame = self.get_parameter('target_frame').value or msg.header.frame_id
        self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self_filter_frame = str(self.get_parameter('self_filter_frame').value or target_frame)
        self_min_x = float(self.get_parameter('self_filter_min_x').value)
        self_max_x = float(self.get_parameter('self_filter_max_x').value)
        self_min_y = float(self.get_parameter('self_filter_min_y').value)
        self_max_y = float(self.get_parameter('self_filter_max_y').value)
        self_min_z = float(self.get_parameter('self_filter_min_z').value)
        self_max_z = float(self.get_parameter('self_filter_max_z').value)
        transform = self._lookup_cloud_transform(msg, target_frame)
        if transform is None:
            return
        self_filter_transform = None
        if self_filter_enabled:
            self_filter_transform = self._lookup_cloud_transform(msg, self_filter_frame)
            if self_filter_transform is None:
                self.get_logger().warn(
                    f'Self-filter disabled for this cloud; no transform '
                    f'{self_filter_frame} <- {msg.header.frame_id}.'
                )

        beam_count = max(1, int(math.ceil((angle_max - angle_min) / angle_increment)))
        default_range = math.inf if use_inf else range_max + 1.0
        ranges: List[float] = [default_range] * beam_count
        candidates: List[List[float]] = [[] for _ in range(beam_count)]

        for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            raw_point = (float(point[0]), float(point[1]), float(point[2]))
            x, y, z = self._transform_point(
                transform,
                raw_point,
            )
            if z < min_height or z > max_height:
                continue

            if self_filter_enabled and self_filter_transform is not None:
                sx, sy, sz = self._transform_point(self_filter_transform, raw_point)
                if (
                    self_min_x <= sx <= self_max_x
                    and self_min_y <= sy <= self_max_y
                    and self_min_z <= sz <= self_max_z
                ):
                    continue

            if (
                self_filter_enabled
                and self_filter_transform is None
                and self_min_x <= x <= self_max_x
                and self_min_y <= y <= self_max_y
                and self_min_z <= z <= self_max_z
            ):
                continue

            distance = math.hypot(x, y)
            if distance < range_min or distance > range_max:
                continue

            angle = math.atan2(y, x)
            if angle < angle_min or angle > angle_max:
                continue

            index = int((angle - angle_min) / angle_increment)
            if 0 <= index < beam_count:
                candidates[index].append(distance)

        for index, values in enumerate(candidates):
            if len(values) < min_points_per_beam:
                continue
            values.sort()
            if use_median_range:
                ranges[index] = values[len(values) // 2]
            else:
                ranges[index] = values[0]

        if reject_isolated_returns:
            filtered = list(ranges)
            for index, value in enumerate(ranges):
                if not math.isfinite(value):
                    continue
                has_neighbor = False
                start = max(0, index - isolation_window)
                stop = min(beam_count, index + isolation_window + 1)
                for neighbor_index in range(start, stop):
                    if neighbor_index == index:
                        continue
                    neighbor = ranges[neighbor_index]
                    if math.isfinite(neighbor) and abs(neighbor - value) <= isolation_max_range_delta:
                        has_neighbor = True
                        break
                if not has_neighbor:
                    filtered[index] = default_range
            ranges = filtered

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = target_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_min + angle_increment * (beam_count - 1)
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = float(self.get_parameter('scan_time').value)
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _lookup_cloud_transform(
        self,
        msg: PointCloud2,
        target_frame: str,
    ) -> Optional[TransformStamped]:
        if not msg.header.frame_id:
            self.get_logger().warn('Dropping point cloud without a frame_id.')
            return None
        if msg.header.frame_id == target_frame:
            transform = TransformStamped()
            transform.header.frame_id = target_frame
            transform.child_frame_id = target_frame
            transform.transform.rotation.w = 1.0
            return transform

        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter('transform_tolerance').value)),
            )
        except Exception as exc:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_transform_warning_sec > 2.0:
                self.get_logger().warn(
                    f'Dropping point cloud; no transform {target_frame} <- '
                    f'{msg.header.frame_id}: {exc}'
                )
                self._last_transform_warning_sec = now_sec
            return None

    @staticmethod
    def _transform_point(
        transform: TransformStamped,
        point: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        x, y, z = point
        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Quaternion-vector multiply: q * p * q^-1, expanded to avoid an extra dependency.
        uvx = q.y * z - q.z * y
        uvy = q.z * x - q.x * z
        uvz = q.x * y - q.y * x
        uuvx = q.y * uvz - q.z * uvy
        uuvy = q.z * uvx - q.x * uvz
        uuvz = q.x * uvy - q.y * uvx
        scale = 2.0 * q.w
        return (
            x + scale * uvx + 2.0 * uuvx + tx,
            y + scale * uvy + 2.0 * uuvy + ty,
            z + scale * uvz + 2.0 * uuvz + tz,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloudToScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
