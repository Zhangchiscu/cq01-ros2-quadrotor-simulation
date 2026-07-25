#!/usr/bin/env python3
from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class V712StartRouteAndH10(Node):
    """Starts an already-running inactive dispatcher, then arms H10 once."""

    def __init__(self) -> None:
        super().__init__('cq01_v712_start_route_and_h10')

        self.route_client = self.create_client(
            Trigger,
            '/cq01_v71_route_dispatcher/start',
        )
        self.h10_client = self.create_client(
            Trigger,
            '/cq01_v71_h10_manager/trigger',
        )

        self.state = 'WAITING_SERVICES'
        self.route_future = None
        self.h10_future = None
        self.route_started_s: Optional[float] = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def timer_callback(self) -> None:
        if self.state == 'WAITING_SERVICES':
            if (
                self.route_client.service_is_ready()
                and self.h10_client.service_is_ready()
            ):
                self.get_logger().warning(
                    'Prelude exited. Calling the start service on the '
                    'already-running original dispatcher.'
                )
                self.route_future = self.route_client.call_async(
                    Trigger.Request()
                )
                self.route_future.add_done_callback(
                    self.route_done
                )
                self.state = 'STARTING_ROUTE'
            return

        if (
            self.state == 'ROUTE_STARTED'
            and self.route_started_s is not None
            and self.now_s() - self.route_started_s >= 0.5
        ):
            self.get_logger().warning(
                'Original dispatcher accepted start. Arming H10 once.'
            )
            self.h10_future = self.h10_client.call_async(
                Trigger.Request()
            )
            self.h10_future.add_done_callback(
                self.h10_done
            )
            self.state = 'ARMING_H10'

    def route_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Dispatcher start service failed: {exc}'
            )
            os._exit(2)

        if not response.success:
            self.get_logger().error(
                f'Dispatcher rejected start: {response.message}'
            )
            os._exit(3)

        self.get_logger().warning(
            f'DISPATCHER STARTED: {response.message}'
        )
        self.route_started_s = self.now_s()
        self.state = 'ROUTE_STARTED'

    def h10_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'H10 trigger service failed: {exc}'
            )
            os._exit(4)

        if not response.success:
            self.get_logger().error(
                f'H10 rejected arm: {response.message}'
            )
            os._exit(5)

        self.get_logger().warning(
            'H10 ARMED ONCE. Original H10 now owns all three cycles.'
        )
        os._exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V712StartRouteAndH10()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
