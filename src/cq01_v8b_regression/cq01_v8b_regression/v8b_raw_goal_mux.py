#!/usr/bin/env python3
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class V8BRawGoalMux(Node):
    """Event-forwarding mux with one publisher on /drone/goal_raw."""

    def __init__(self) -> None:
        super().__init__('cq01_v8b_raw_goal_mux')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.output = self.create_publisher(
            PoseStamped,
            '/drone/goal_raw',
            10,
        )
        self.status = self.create_publisher(
            String,
            '/cq01_v8b/raw_goal_mux_status',
            transient,
        )
        self.route_count = 0
        self.h14_count = 0
        self.last_source = 'none'

        self.create_subscription(
            PoseStamped,
            '/cq01_v8b/route_goal_raw',
            lambda message: self.forward('route', message),
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/cq01_v8b/h14_replan_goal_raw',
            lambda message: self.forward('h14_replan', message),
            10,
        )
        self.publish_status()

    def forward(
        self,
        source: str,
        message: PoseStamped,
    ) -> None:
        forwarded = PoseStamped()
        forwarded.header = message.header
        forwarded.pose = message.pose
        forwarded.header.stamp = self.get_clock().now().to_msg()
        self.output.publish(forwarded)

        if source == 'route':
            self.route_count += 1
        else:
            self.h14_count += 1
        self.last_source = source
        self.publish_status()

        if source == 'h14_replan':
            self.get_logger().warning(
                'V8-B H14 replan goal forwarded through the single '
                '/drone/goal_raw publisher.'
            )

    def publish_status(self) -> None:
        payload = {
            'state': 'running',
            'last_source': self.last_source,
            'route_forward_count': self.route_count,
            'h14_replan_forward_count': self.h14_count,
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8BRawGoalMux()
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
