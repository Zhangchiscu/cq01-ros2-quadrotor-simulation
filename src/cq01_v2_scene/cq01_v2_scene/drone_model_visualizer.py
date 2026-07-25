#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import math
from typing import Deque, Iterable, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

Vector3 = Tuple[float, float, float]


def normalized_quaternion(q: Quaternion) -> Tuple[float, float, float, float]:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    """Rotate a vector by a quaternion without external dependencies."""
    qx, qy, qz, qw = normalized_quaternion(q)
    vx, vy, vz = vector

    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + qw*t + cross(q.xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def translated_point(origin: Vector3, offset: Vector3) -> Point:
    point = Point()
    point.x = origin[0] + offset[0]
    point.y = origin[1] + offset[1]
    point.z = origin[2] + offset[2]
    return point


class DroneModelVisualizer(Node):
    """Publish one recognizable quadrotor model and an actual trajectory line.

    RViz Odometry history renders every retained sample as a yellow arrow. This
    node replaces that ambiguous display with one enlarged quadrotor model at
    the current pose. The enlargement is visualization-only; planning and
    dynamics coordinates remain unchanged.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v34b_drone_model_visualizer')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('marker_topic', '/cq01_v34b/drone_model')
        self.declare_parameter('trajectory_topic', '/cq01_v34b/actual_trajectory')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('arm_half_span_m', 3.0)
        self.declare_parameter('body_length_m', 2.6)
        self.declare_parameter('body_width_m', 1.8)
        self.declare_parameter('body_height_m', 0.75)
        self.declare_parameter('rotor_diameter_m', 1.65)
        self.declare_parameter('max_trajectory_points', 3000)
        self.declare_parameter('minimum_path_step_m', 0.35)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.arm_half_span = float(self.get_parameter('arm_half_span_m').value)
        self.body_length = float(self.get_parameter('body_length_m').value)
        self.body_width = float(self.get_parameter('body_width_m').value)
        self.body_height = float(self.get_parameter('body_height_m').value)
        self.rotor_diameter = float(self.get_parameter('rotor_diameter_m').value)
        self.minimum_path_step = max(
            0.0, float(self.get_parameter('minimum_path_step_m').value)
        )
        maximum_points = max(
            10, int(self.get_parameter('max_trajectory_points').value)
        )
        self.trajectory: Deque[Vector3] = deque(maxlen=maximum_points)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, marker_qos
        )
        self.trajectory_publisher = self.create_publisher(
            Path, self.trajectory_topic, marker_qos
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        self.get_logger().info(
            'V3.4B quadrotor model visualization started: '
            f'odom={self.odom_topic}, marker={self.marker_topic}'
        )

    @staticmethod
    def _set_color(marker: Marker, rgba: Tuple[float, float, float, float]) -> None:
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

    def _base_marker(self, stamp, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'cq01_v34b_quadrotor'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _copy_orientation(target: Marker, source: Quaternion) -> None:
        qx, qy, qz, qw = normalized_quaternion(source)
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        origin = (float(position.x), float(position.y), float(position.z))
        stamp = self.get_clock().now().to_msg()

        if (
            not self.trajectory
            or math.dist(self.trajectory[-1], origin) >= self.minimum_path_step
        ):
            self.trajectory.append(origin)

        markers = MarkerArray()

        body = self._base_marker(stamp, 0, Marker.CUBE)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = origin
        self._copy_orientation(body, orientation)
        body.scale.x = self.body_length
        body.scale.y = self.body_width
        body.scale.z = self.body_height
        self._set_color(body, (0.08, 0.12, 0.18, 1.0))
        markers.markers.append(body)

        arm_vectors = (
            (self.arm_half_span, self.arm_half_span, 0.0),
            (-self.arm_half_span, -self.arm_half_span, 0.0),
            (self.arm_half_span, -self.arm_half_span, 0.0),
            (-self.arm_half_span, self.arm_half_span, 0.0),
        )
        rotated = [rotate_vector(orientation, vector) for vector in arm_vectors]

        arms = self._base_marker(stamp, 1, Marker.LINE_LIST)
        arms.scale.x = 0.42
        self._set_color(arms, (0.95, 0.52, 0.08, 1.0))
        arms.points.extend([
            translated_point(origin, rotated[0]),
            translated_point(origin, rotated[1]),
            translated_point(origin, rotated[2]),
            translated_point(origin, rotated[3]),
        ])
        markers.markers.append(arms)

        rotor_local_positions = (
            (self.arm_half_span, self.arm_half_span, 0.12),
            (-self.arm_half_span, -self.arm_half_span, 0.12),
            (self.arm_half_span, -self.arm_half_span, 0.12),
            (-self.arm_half_span, self.arm_half_span, 0.12),
        )
        rotor_colors = (
            (0.10, 0.85, 0.95, 0.82),
            (0.10, 0.85, 0.95, 0.82),
            (0.98, 0.25, 0.10, 0.82),
            (0.98, 0.25, 0.10, 0.82),
        )
        for rotor_index, (local, color) in enumerate(
            zip(rotor_local_positions, rotor_colors), start=2
        ):
            offset = rotate_vector(orientation, local)
            rotor = self._base_marker(stamp, rotor_index, Marker.CYLINDER)
            rotor_point = translated_point(origin, offset)
            rotor.pose.position.x = rotor_point.x
            rotor.pose.position.y = rotor_point.y
            rotor.pose.position.z = rotor_point.z
            self._copy_orientation(rotor, orientation)
            rotor.scale.x = self.rotor_diameter
            rotor.scale.y = self.rotor_diameter
            rotor.scale.z = 0.16
            self._set_color(rotor, color)
            markers.markers.append(rotor)

        nose_offset = rotate_vector(
            orientation, (0.62 * self.body_length, 0.0, 0.0)
        )
        nose = self._base_marker(stamp, 6, Marker.SPHERE)
        nose_point = translated_point(origin, nose_offset)
        nose.pose.position.x = nose_point.x
        nose.pose.position.y = nose_point.y
        nose.pose.position.z = nose_point.z
        nose.scale.x = nose.scale.y = nose.scale.z = 0.55
        self._set_color(nose, (0.15, 1.0, 0.20, 1.0))
        markers.markers.append(nose)

        self.marker_publisher.publish(markers)
        self.publish_trajectory(stamp)

    def publish_trajectory(self, stamp) -> None:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        for x, y, z in self.trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.trajectory_publisher.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneModelVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
