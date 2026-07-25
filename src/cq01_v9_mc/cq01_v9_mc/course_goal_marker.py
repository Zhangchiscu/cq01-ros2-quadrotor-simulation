#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


class CourseGoalMarker(Node):
    """Read-only visualization of the controller's current target."""

    def __init__(self) -> None:
        super().__init__('cq01_course_goal_marker')

        self.declare_parameter('goal_topic', '/drone/goal')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'marker_topic',
            '/cq01_v9/current_goal_marker',
        )
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        marker_topic = str(
            self.get_parameter('marker_topic').value
        )

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            transient,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self.goal_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            20,
        )

        self.goal: Optional[PoseStamped] = None
        self.position: Optional[Point] = None

        rate = max(
            1.0,
            float(
                self.get_parameter(
                    'publish_rate_hz'
                ).value
            ),
        )
        self.timer = self.create_timer(
            1.0/rate,
            self.publish_markers,
        )

    def goal_callback(self, message: PoseStamped) -> None:
        self.goal = message

    def odom_callback(self, message: Odometry) -> None:
        self.position = message.pose.pose.position

    def publish_markers(self) -> None:
        if self.goal is None:
            return

        now = self.get_clock().now().to_msg()
        target = self.goal.pose.position

        array = MarkerArray()

        sphere = Marker()
        sphere.header.frame_id = self.frame_id
        sphere.header.stamp = now
        sphere.ns = 'cq01_current_goal'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = target
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 1.2
        sphere.scale.y = 1.2
        sphere.scale.z = 1.2
        sphere.color.r = 1.0
        sphere.color.g = 0.10
        sphere.color.b = 0.80
        sphere.color.a = 1.0
        array.markers.append(sphere)

        ring = Marker()
        ring.header = sphere.header
        ring.ns = sphere.ns
        ring.id = 1
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.position = target
        ring.pose.orientation.w = 1.0
        ring.scale.x = 2.4
        ring.scale.y = 2.4
        ring.scale.z = 0.12
        ring.color.r = 1.0
        ring.color.g = 0.85
        ring.color.b = 0.05
        ring.color.a = 0.90
        array.markers.append(ring)

        if self.position is not None:
            line = Marker()
            line.header = sphere.header
            line.ns = sphere.ns
            line.id = 2
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.18
            line.color.r = 0.15
            line.color.g = 0.95
            line.color.b = 1.0
            line.color.a = 0.85

            start = Point()
            start.x = float(self.position.x)
            start.y = float(self.position.y)
            start.z = float(self.position.z)

            end = Point()
            end.x = float(target.x)
            end.y = float(target.y)
            end.z = float(target.z)

            line.points.extend([start, end])
            array.markers.append(line)

        self.publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CourseGoalMarker()
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
