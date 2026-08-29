import math
from typing import Dict, Iterable, List, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def coerce_waypoints(
    names: Iterable[str],
    configured: Dict[str, Iterable[float]],
    defaults: Dict[str, Iterable[float]],
) -> Dict[str, Tuple[float, float, float]]:
    waypoints = {}
    for name in names:
        values = list(configured.get(name, defaults.get(name, [0.0, 0.0, 0.0])))
        if len(values) != 3:
            raise ValueError(f'waypoint_{name} must be [x, y, yaw].')
        waypoints[str(name)] = (float(values[0]), float(values[1]), float(values[2]))
    return waypoints


class WaypointManager(Node):
    def __init__(self):
        super().__init__('amms_waypoint_manager')

        defaults = {
            'home': [0.0, 0.0, 0.0],
            'pick_station': [1.90, -1.20, 0.0],
            'pick_clear': [1.20, -0.75, 0.0],
            'place_approach': [2.35, 0.65, 0.0],
            'place_station': [3.25, 0.0, 0.0],
            'place_clear': [3.25, 0.0, 0.0],
        }

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoint_names', list(defaults.keys()))
        self.declare_parameter('pose_array_topic', '/amms/waypoints')
        self.declare_parameter('marker_topic', '/amms/waypoint_markers')
        self.declare_parameter('publish_period_sec', 1.0)
        self.defaults = defaults

        self.waypoint_names = [str(name) for name in self.get_parameter('waypoint_names').value]
        for name in self.waypoint_names:
            self.declare_parameter(f'waypoint_{name}', defaults.get(name, [0.0, 0.0, 0.0]))

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseArray, self.get_parameter('pose_array_topic').value, qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_timer(float(self.get_parameter('publish_period_sec').value), self._publish)

    def _waypoints(self) -> Dict[str, Tuple[float, float, float]]:
        configured = {
            name: self.get_parameter(f'waypoint_{name}').value
            for name in self.waypoint_names
        }
        try:
            return coerce_waypoints(self.waypoint_names, configured, self.defaults)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return {}

    def _publish(self) -> None:
        frame_id = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()
        waypoints = self._waypoints()

        poses = PoseArray()
        poses.header.frame_id = frame_id
        poses.header.stamp = stamp

        markers = MarkerArray()
        for index, (name, waypoint) in enumerate(waypoints.items()):
            pose = self._pose_from_xyyaw(waypoint)
            poses.poses.append(pose)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'amms_waypoints'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.55
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.10
            marker.color.g = 0.55
            marker.color.b = 1.00
            marker.color.a = 0.90
            markers.markers.append(marker)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = 'amms_waypoint_labels'
            label.id = index + 1000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose
            label.pose.position.z = 0.65
            label.scale.z = 0.25
            label.color.r = 1.00
            label.color.g = 1.00
            label.color.b = 1.00
            label.color.a = 1.00
            label.text = name
            markers.markers.append(label)

        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)

    @staticmethod
    def _pose_from_xyyaw(values: Iterable[float]) -> Pose:
        x, y, yaw = values
        qx, qy, qz, qw = _quaternion_from_yaw(float(yaw))
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
