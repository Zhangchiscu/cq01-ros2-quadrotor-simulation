#!/usr/bin/env python3
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class V71H10ArmOnce(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v71_h10_arm_once')
        self.client = self.create_client(
            Trigger,
            '/cq01_v71_h10_manager/trigger',
        )
        self.requested = False
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self) -> None:
        if self.requested:
            return
        if not self.client.service_is_ready():
            return
        self.requested = True
        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self.done)

    def done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'H10 trigger service error: {exc}')
            os._exit(2)

        if not response.success:
            self.get_logger().error(
                f'H10 arm rejected: {response.message}'
            )
            os._exit(3)

        self.get_logger().warning(
            'H10 ARMED ONCE. The original manager now owns all '
            'three automatic disturbance/replan cycles.'
        )
        os._exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V71H10ArmOnce()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
