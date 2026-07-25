#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DefaultGoalPublisher(Node):
    """Publish the initial goal repeatedly for a short startup interval."""

    def __init__(self) -> None:
        super().__init__('default_goal_publisher')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 1.5)
        self.declare_parameter('startup_delay_s', 2.0)
        self.declare_parameter('publish_count', 10)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_z = float(self.get_parameter('goal_z').value)
        self.startup_delay_s = float(
            self.get_parameter('startup_delay_s').value
        )
        self.remaining_count = int(
            self.get_parameter('publish_count').value
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/drone/goal',
            10,
        )

        self.timer: Optional[object] = self.create_timer(
            self.startup_delay_s,
            self.begin_publishing,
        )
        self.publish_timer: Optional[object] = None

        self.get_logger().info(
            'Default goal scheduled: '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f}, '
            f'{self.goal_z:.2f})'
        )

    def begin_publishing(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        self.publish_goal()
        self.publish_timer = self.create_timer(
            0.20,
            self.publish_goal,
        )

    def publish_goal(self) -> None:
        if self.remaining_count <= 0:
            if self.publish_timer is not None:
                self.publish_timer.cancel()
                self.publish_timer = None
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = self.goal_x
        message.pose.position.y = self.goal_y
        message.pose.position.z = self.goal_z
        message.pose.orientation.w = 1.0

        self.publisher.publish(message)
        self.remaining_count -= 1

        if self.remaining_count == 9:
            self.get_logger().info(
                'Published default goal'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DefaultGoalPublisher()

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
