#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

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


class IntegratedGoalMux(Node):
    """Select planner or embedded-pattern goals without command races."""

    def __init__(self) -> None:
        super().__init__('cq01_integrated_goal_mux')

        self.declare_parameter(
            'planner_goal_topic',
            '/drone/goal_planner',
        )
        self.declare_parameter(
            'pattern_goal_topic',
            '/drone/goal_pattern',
        )
        self.declare_parameter(
            'output_goal_topic',
            '/drone/goal',
        )
        self.declare_parameter(
            'mode_topic',
            '/cq01_integrated/control_mode',
        )
        self.declare_parameter('initial_mode', 'planner')
        self.declare_parameter('republish_rate_hz', 20.0)

        self.mode = str(
            self.get_parameter('initial_mode').value
        )
        self.latest_planner: Optional[PoseStamped] = None
        self.latest_pattern: Optional[PoseStamped] = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.output_pub = self.create_publisher(
            PoseStamped,
            str(
                self.get_parameter(
                    'output_goal_topic'
                ).value
            ),
            20,
        )
        self.create_subscription(
            PoseStamped,
            str(
                self.get_parameter(
                    'planner_goal_topic'
                ).value
            ),
            self.planner_callback,
            30,
        )
        self.create_subscription(
            PoseStamped,
            str(
                self.get_parameter(
                    'pattern_goal_topic'
                ).value
            ),
            self.pattern_callback,
            30,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mode_topic').value),
            self.mode_callback,
            transient_qos,
        )

        rate_hz = max(
            5.0,
            float(
                self.get_parameter(
                    'republish_rate_hz'
                ).value
            ),
        )
        self.timer = self.create_timer(
            1.0 / rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Integrated goal mux ready: mode={self.mode}.'
        )

    def planner_callback(
        self,
        message: PoseStamped,
    ) -> None:
        self.latest_planner = message
        if self.mode == 'planner':
            self.output_pub.publish(message)

    def pattern_callback(
        self,
        message: PoseStamped,
    ) -> None:
        self.latest_pattern = message
        if self.mode == 'pattern':
            self.output_pub.publish(message)

    def mode_callback(self, message: String) -> None:
        requested = str(message.data).strip().lower()
        if requested not in ('planner', 'pattern'):
            self.get_logger().warning(
                f'Ignoring invalid goal mode: {requested}'
            )
            return
        changed = requested != self.mode
        self.mode = requested
        if changed:
            self.get_logger().info(
                f'Goal source switched to: {self.mode}'
            )
        selected = (
            self.latest_planner
            if self.mode == 'planner'
            else self.latest_pattern
        )
        if selected is not None:
            self.output_pub.publish(selected)

    def timer_callback(self) -> None:
        selected = (
            self.latest_planner
            if self.mode == 'planner'
            else self.latest_pattern
        )
        if selected is None:
            return
        selected.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.output_pub.publish(selected)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegratedGoalMux()
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
