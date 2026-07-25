#!/usr/bin/env python3

from math import cos, sin
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class DroneVisualizationNode(Node):
    """Publish a simple quadrotor model and goal marker for RViz2."""

    def __init__(self) -> None:
        super().__init__('drone_visualization')

        self.declare_parameter('arm_length', 0.225)
        self.declare_parameter('propeller_radius', 0.08)

        self.arm_length = float(
            self.get_parameter('arm_length').value
        )
        self.propeller_radius = float(
            self.get_parameter('propeller_radius').value
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_goal: Optional[PoseStamped] = None

        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            20,
        )
        self.create_subscription(
            PoseStamped,
            '/drone/goal',
            self.goal_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/drone/markers',
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_markers,
        )

        self.get_logger().info(
            'Drone visualization node started'
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def goal_callback(self, message: PoseStamped) -> None:
        self.latest_goal = message

    @staticmethod
    def set_color(
        marker: Marker,
        red: float,
        green: float,
        blue: float,
        alpha: float = 1.0,
    ) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def make_marker(
        self,
        marker_id: int,
        marker_type: int,
        namespace: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200_000_000
        return marker

    def publish_markers(self) -> None:
        if self.latest_odom is None:
            return

        marker_array = MarkerArray()

        # Central fuselage
        body = self.make_marker(
            0,
            Marker.CUBE,
            'drone',
        )
        body.scale.x = 0.18
        body.scale.y = 0.12
        body.scale.z = 0.07
        self.set_color(body, 0.15, 0.45, 0.95)
        marker_array.markers.append(body)

        # X-config arms represented by two thin cubes
        for index, yaw in enumerate(
            (0.78539816339, -0.78539816339),
            start=1,
        ):
            arm = self.make_marker(
                index,
                Marker.CUBE,
                'drone',
            )
            arm.scale.x = self.arm_length * 2.0
            arm.scale.y = 0.025
            arm.scale.z = 0.025
            arm.pose.orientation.z = sin(yaw / 2.0)
            arm.pose.orientation.w = cos(yaw / 2.0)
            self.set_color(arm, 0.20, 0.20, 0.24)
            marker_array.markers.append(arm)

        # Four rotors
        rotor_positions = (
            (+1.0, +1.0),
            (+1.0, -1.0),
            (-1.0, -1.0),
            (-1.0, +1.0),
        )

        diagonal = self.arm_length / (2.0 ** 0.5)

        for rotor_index, (x_sign, y_sign) in enumerate(
            rotor_positions,
            start=10,
        ):
            rotor = self.make_marker(
                rotor_index,
                Marker.CYLINDER,
                'drone',
            )
            rotor.pose.position.x = x_sign * diagonal
            rotor.pose.position.y = y_sign * diagonal
            rotor.pose.position.z = 0.02
            rotor.scale.x = self.propeller_radius * 2.0
            rotor.scale.y = self.propeller_radius * 2.0
            rotor.scale.z = 0.012

            if rotor_index in (10, 12):
                self.set_color(rotor, 0.95, 0.30, 0.20, 0.90)
            else:
                self.set_color(rotor, 0.25, 0.85, 0.35, 0.90)

            marker_array.markers.append(rotor)

        # Forward direction arrow
        arrow = self.make_marker(
            20,
            Marker.ARROW,
            'drone',
        )
        arrow.pose.position.x = 0.10
        arrow.scale.x = 0.22
        arrow.scale.y = 0.04
        arrow.scale.z = 0.04
        self.set_color(arrow, 1.00, 0.85, 0.10)
        marker_array.markers.append(arrow)

        if self.latest_goal is not None:
            goal = Marker()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.ns = 'goal'
            goal.id = 100
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose = self.latest_goal.pose
            goal.scale.x = 0.20
            goal.scale.y = 0.20
            goal.scale.z = 0.20
            self.set_color(goal, 1.00, 0.20, 0.20, 0.85)
            goal.lifetime.sec = 0
            marker_array.markers.append(goal)

            goal_arrow = Marker()
            goal_arrow.header.frame_id = 'map'
            goal_arrow.header.stamp = (
                self.get_clock().now().to_msg()
            )
            goal_arrow.ns = 'goal'
            goal_arrow.id = 101
            goal_arrow.type = Marker.ARROW
            goal_arrow.action = Marker.ADD
            goal_arrow.pose = self.latest_goal.pose
            goal_arrow.scale.x = 0.45
            goal_arrow.scale.y = 0.06
            goal_arrow.scale.z = 0.06
            self.set_color(
                goal_arrow,
                1.00,
                0.55,
                0.05,
                0.95,
            )
            goal_arrow.lifetime.sec = 0
            marker_array.markers.append(goal_arrow)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneVisualizationNode()

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
