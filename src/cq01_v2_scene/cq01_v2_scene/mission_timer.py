#!/usr/bin/env python3
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class MissionTimer(Node):
    def __init__(self) -> None:
        super().__init__('cq01_mission_timer')
        self.declare_parameter(
            'status_topic', '/cq01_v34g/mission_status'
        )
        self.declare_parameter('target_time_s', 180.0)
        self.target_time = float(
            self.get_parameter('target_time_s').value
        )
        self.started_at = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self.callback,
            qos,
        )
        self.get_logger().info(
            f'Mission timer ready; target <= {self.target_time:.1f} s'
        )

    def callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(payload.get('state', ''))
        if state in ('start_requested', 'goal_sent'):
            if self.started_at is None:
                self.started_at = time.monotonic()
                self.get_logger().info('CQ-01 timing started')
            return
        if state not in ('mission_completed', 'stalled', 'planning_failed'):
            return
        if self.started_at is None:
            return
        elapsed = time.monotonic() - self.started_at
        self.get_logger().info(
            f'CQ-01 final state={state}; elapsed={elapsed:.2f} s; '
            f'target={self.target_time:.2f} s; '
            f'within_target={elapsed <= self.target_time}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
