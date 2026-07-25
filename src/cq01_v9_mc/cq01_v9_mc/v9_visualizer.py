#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import math
from typing import Deque, Optional, Tuple

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


def normalized_quaternion(
    q: Quaternion,
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(
        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    )
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        q.x/norm,
        q.y/norm,
        q.z/norm,
        q.w/norm,
    )


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    qx, qy, qz, qw = normalized_quaternion(q)
    vx, vy, vz = v
    tx = 2.0*(qy*vz - qz*vy)
    ty = 2.0*(qz*vx - qx*vz)
    tz = 2.0*(qx*vy - qy*vx)
    return (
        vx + qw*tx + qy*tz - qz*ty,
        vy + qw*ty + qz*tx - qx*tz,
        vz + qw*tz + qx*ty - qy*tx,
    )


def translated_point(
    origin: Vector3,
    offset: Vector3,
) -> Point:
    p = Point()
    p.x = origin[0] + offset[0]
    p.y = origin[1] + offset[1]
    p.z = origin[2] + offset[2]
    return p


class V9Visualizer(Node):
    """Bounded, decimated visualization that remains responsive in RViz.

    The original visualizer republished an increasingly long Path on every
    odometry sample. Odom runs near 200 Hz, so late in the mission RViz was
    receiving thousands of poses hundreds of times per second.

    This node:
    - stores odometry continuously;
    - publishes the quadrotor model at 20 Hz;
    - publishes the full decimated trajectory at only 2 Hz;
    - retains the whole P0-P15 flight with 0.8 m spatial sampling.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v9_visualizer')

        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'marker_topic',
            '/cq01_v9/drone_model',
        )
        self.declare_parameter(
            'trajectory_topic',
            '/cq01_v9/actual_trajectory',
        )
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('arm_half_span_m', 0.55)
        self.declare_parameter('body_length_m', 0.65)
        self.declare_parameter('body_width_m', 0.42)
        self.declare_parameter('body_height_m', 0.24)
        self.declare_parameter('rotor_diameter_m', 0.45)
        self.declare_parameter('minimum_path_step_m', 0.80)
        self.declare_parameter('max_trajectory_points', 1600)
        self.declare_parameter('model_publish_rate_hz', 20.0)
        self.declare_parameter('path_publish_rate_hz', 2.0)

        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.marker_topic = str(
            self.get_parameter('marker_topic').value
        )
        self.trajectory_topic = str(
            self.get_parameter('trajectory_topic').value
        )
        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.arm_half_span = float(
            self.get_parameter('arm_half_span_m').value
        )
        self.body_length = float(
            self.get_parameter('body_length_m').value
        )
        self.body_width = float(
            self.get_parameter('body_width_m').value
        )
        self.body_height = float(
            self.get_parameter('body_height_m').value
        )
        self.rotor_diameter = float(
            self.get_parameter('rotor_diameter_m').value
        )
        self.minimum_path_step = max(
            0.10,
            float(
                self.get_parameter(
                    'minimum_path_step_m'
                ).value
            ),
        )
        max_points = max(
            100,
            int(
                self.get_parameter(
                    'max_trajectory_points'
                ).value
            ),
        )

        model_rate = max(
            5.0,
            float(
                self.get_parameter(
                    'model_publish_rate_hz'
                ).value
            ),
        )
        path_rate = max(
            0.5,
            float(
                self.get_parameter(
                    'path_publish_rate_hz'
                ).value
            ),
        )

        self.latest_odom: Optional[Odometry] = None
        self.trajectory: Deque[Vector3] = deque(
            maxlen=max_points
        )
        self.odom_count = 0
        self.model_publish_count = 0
        self.path_publish_count = 0

        visual_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            visual_qos,
        )
        self.path_pub = self.create_publisher(
            Path,
            self.trajectory_topic,
            visual_qos,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        self.model_timer = self.create_timer(
            1.0/model_rate,
            self.publish_model,
        )
        self.path_timer = self.create_timer(
            1.0/path_rate,
            self.publish_trajectory,
        )

        self.get_logger().warning(
            'V9-MC.5 visualization throttled: '
            f'model={model_rate:.1f} Hz, '
            f'path={path_rate:.1f} Hz, '
            f'path_step={self.minimum_path_step:.2f} m, '
            f'max_points={max_points}.'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_count += 1
        p = msg.pose.pose.position
        point = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        if (
            not self.trajectory
            or math.dist(
                self.trajectory[-1],
                point,
            ) >= self.minimum_path_step
        ):
            self.trajectory.append(point)

    @staticmethod
    def set_color(
        marker: Marker,
        rgba: Tuple[float, float, float, float],
    ) -> None:
        (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        ) = rgba

    def base_marker(
        self,
        stamp,
        marker_id: int,
        marker_type: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'cq01_v9_quadrotor'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def copy_orientation(
        marker: Marker,
        q: Quaternion,
    ) -> None:
        x, y, z, w = normalized_quaternion(q)
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w

    def publish_model(self) -> None:
        if self.latest_odom is None:
            return

        msg = self.latest_odom
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        origin = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        body = self.base_marker(
            stamp,
            0,
            Marker.CUBE,
        )
        (
            body.pose.position.x,
            body.pose.position.y,
            body.pose.position.z,
        ) = origin
        self.copy_orientation(body, q)
        body.scale.x = self.body_length
        body.scale.y = self.body_width
        body.scale.z = self.body_height
        self.set_color(
            body,
            (0.08, 0.12, 0.18, 1.0),
        )
        markers.markers.append(body)

        arm_vectors = (
            (
                self.arm_half_span,
                self.arm_half_span,
                0.0,
            ),
            (
                -self.arm_half_span,
                -self.arm_half_span,
                0.0,
            ),
            (
                self.arm_half_span,
                -self.arm_half_span,
                0.0,
            ),
            (
                -self.arm_half_span,
                self.arm_half_span,
                0.0,
            ),
        )
        rotated = [
            rotate_vector(q, vector)
            for vector in arm_vectors
        ]

        arms = self.base_marker(
            stamp,
            1,
            Marker.LINE_LIST,
        )
        arms.scale.x = 0.10
        self.set_color(
            arms,
            (0.95, 0.52, 0.08, 1.0),
        )
        arms.points.extend([
            translated_point(origin, rotated[0]),
            translated_point(origin, rotated[1]),
            translated_point(origin, rotated[2]),
            translated_point(origin, rotated[3]),
        ])
        markers.markers.append(arms)

        rotor_offsets = (
            (
                self.arm_half_span,
                self.arm_half_span,
                0.12,
            ),
            (
                -self.arm_half_span,
                -self.arm_half_span,
                0.12,
            ),
            (
                self.arm_half_span,
                -self.arm_half_span,
                0.12,
            ),
            (
                -self.arm_half_span,
                self.arm_half_span,
                0.12,
            ),
        )
        rotor_colors = (
            (0.10, 0.85, 0.95, 0.82),
            (0.10, 0.85, 0.95, 0.82),
            (0.98, 0.25, 0.10, 0.82),
            (0.98, 0.25, 0.10, 0.82),
        )

        for marker_id, (offset, color) in enumerate(
            zip(rotor_offsets, rotor_colors),
            start=2,
        ):
            rotor = self.base_marker(
                stamp,
                marker_id,
                Marker.CYLINDER,
            )
            point = translated_point(
                origin,
                rotate_vector(q, offset),
            )
            rotor.pose.position.x = point.x
            rotor.pose.position.y = point.y
            rotor.pose.position.z = point.z
            self.copy_orientation(rotor, q)
            rotor.scale.x = self.rotor_diameter
            rotor.scale.y = self.rotor_diameter
            rotor.scale.z = 0.08
            self.set_color(rotor, color)
            markers.markers.append(rotor)

        nose = self.base_marker(
            stamp,
            6,
            Marker.SPHERE,
        )
        nose_point = translated_point(
            origin,
            rotate_vector(
                q,
                (
                    0.62*self.body_length,
                    0.0,
                    0.0,
                ),
            ),
        )
        nose.pose.position.x = nose_point.x
        nose.pose.position.y = nose_point.y
        nose.pose.position.z = nose_point.z
        nose.scale.x = 0.18
        nose.scale.y = 0.18
        nose.scale.z = 0.18
        self.set_color(
            nose,
            (0.15, 1.0, 0.20, 1.0),
        )
        markers.markers.append(nose)

        self.marker_pub.publish(markers)
        self.model_publish_count += 1

    def publish_trajectory(self) -> None:
        if not self.trajectory:
            return

        stamp = self.get_clock().now().to_msg()
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

        self.path_pub.publish(path)
        self.path_publish_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V9Visualizer()
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
