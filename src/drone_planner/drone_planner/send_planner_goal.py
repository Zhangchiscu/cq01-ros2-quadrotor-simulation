#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PlannerGoalSender(Node):
    def __init__(self, x: float, y: float, z: float) -> None:
        super().__init__("planner_goal_sender")
        self.publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0

        self.message = message
        self.publish_count = 0
        self.timer = self.create_timer(0.20, self.publish_goal)

        self.get_logger().info(
            f"Sending planner goal: ({x:.2f}, {y:.2f}, {z:.2f})"
        )

    def publish_goal(self) -> None:
        self.publisher.publish(self.message)
        self.publish_count += 1

        if self.publish_count >= 5:
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    values = sys.argv[1:]
    x = float(values[0]) if len(values) > 0 else 6.5
    y = float(values[1]) if len(values) > 1 else 0.0
    z = float(values[2]) if len(values) > 2 else 1.5

    node = PlannerGoalSender(x, y, z)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
