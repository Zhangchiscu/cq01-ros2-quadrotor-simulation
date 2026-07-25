#!/usr/bin/env python3
from __future__ import annotations

import json
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SummitFinaleOrchestrator(Node):
    """Full P0-P15 A* route, then summit ascent and fast patterns."""

    def __init__(self) -> None:
        super().__init__('cq01_summit_finale_orchestrator')

        self.declare_parameter(
            'route_status_topic',
            '/cq01_summit/route_status',
        )
        self.declare_parameter(
            'pattern_status_topic',
            '/cq01_pattern/status',
        )
        self.declare_parameter(
            'integrated_status_topic',
            '/cq01_integrated/status',
        )
        self.declare_parameter(
            'control_mode_topic',
            '/cq01_integrated/control_mode',
        )
        self.declare_parameter(
            'pattern_start_service',
            '/cq01_v34p3_summit_pattern_manager/start',
        )
        self.declare_parameter('handoff_delay_s', 0.5)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    'integrated_status_topic'
                ).value
            ),
            transient_qos,
        )
        self.mode_pub = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    'control_mode_topic'
                ).value
            ),
            transient_qos,
        )

        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    'route_status_topic'
                ).value
            ),
            self.route_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    'pattern_status_topic'
                ).value
            ),
            self.pattern_callback,
            transient_qos,
        )

        self.pattern_client = self.create_client(
            Trigger,
            str(
                self.get_parameter(
                    'pattern_start_service'
                ).value
            ),
        )

        self.handoff_delay_s = max(
            0.1,
            float(
                self.get_parameter(
                    'handoff_delay_s'
                ).value
            ),
        )

        self.start_time_s = self.now_s()
        self.route_completed = False
        self.pattern_started = False
        self.pattern_completed = False
        self.failed = False
        self.failure_reason = ''
        self.start_pattern_after_s: Optional[float] = None
        self.pattern_request_in_flight = False

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.publish_mode('planner')
        self.publish_status('integrated_started')
        self.get_logger().info(
            'Summit-finale mission started: '
            'P0-P15 -> rapid ascent -> circle -> figure-eight.'
        )

    def now_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds / 1.0e9
        )

    @staticmethod
    def parse(message: String) -> Optional[Dict]:
        try:
            return json.loads(message.data)
        except json.JSONDecodeError:
            return None

    def publish_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.mode_pub.publish(message)

    def publish_status(
        self,
        state: str,
        **extra,
    ) -> None:
        payload = {
            'state': state,
            'elapsed_s': self.now_s() - self.start_time_s,
            'route_completed': self.route_completed,
            'pattern_started': self.pattern_started,
            'pattern_completed': self.pattern_completed,
            'failed': self.failed,
            'failure_reason': self.failure_reason,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)

    def fail(self, reason: str) -> None:
        if self.failed:
            return
        self.failed = True
        self.failure_reason = reason
        self.publish_status('integrated_failed')
        self.get_logger().error(reason)

    def route_callback(self, message: String) -> None:
        payload = self.parse(message)
        if payload is None or self.failed:
            return

        state = str(payload.get('state', ''))
        if state in ('planning_failed', 'stalled'):
            self.fail(f'Main P0-P15 route failed: {state}')
            return

        if (
            state == 'mission_completed'
            and not self.route_completed
        ):
            self.route_completed = True
            self.publish_mode('pattern')
            self.start_pattern_after_s = (
                self.now_s() + self.handoff_delay_s
            )
            self.publish_status(
                'p15_reached_waiting_rapid_ascent',
                reached_index=payload.get('reached_index'),
                reached_name=payload.get('reached_name'),
            )
            self.get_logger().info(
                'P15 summit reached; switching to '
                'rapid-ascent finale.'
            )

    def pattern_callback(self, message: String) -> None:
        payload = self.parse(message)
        if payload is None or self.failed:
            return

        state = str(payload.get('state', ''))
        if (
            state == 'running'
            and not self.pattern_started
        ):
            self.pattern_started = True
            self.publish_status(
                'summit_finale_running',
                pattern_phase=payload.get('phase'),
            )

        if (
            state == 'pattern_completed'
            and not self.pattern_completed
        ):
            self.pattern_completed = True
            self.publish_status(
                'integrated_completed',
                final_phase='summit_pattern_completed',
            )
            self.get_logger().info(
                'Complete P0-P15 plus summit finale finished.'
            )

    def timer_callback(self) -> None:
        if self.failed:
            self.publish_status('integrated_failed')
            return

        if (
            self.start_pattern_after_s is None
            or self.pattern_request_in_flight
            or self.now_s() < self.start_pattern_after_s
        ):
            return

        if not self.pattern_client.service_is_ready():
            return

        self.pattern_request_in_flight = True
        future = self.pattern_client.call_async(
            Trigger.Request()
        )
        future.add_done_callback(
            self.pattern_service_done
        )

    def pattern_service_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exception:
            self.fail(
                f'Pattern start service exception: {exception}'
            )
            return

        if not response.success:
            self.fail(
                f'Pattern start rejected: {response.message}'
            )
            return

        self.pattern_started = True
        self.start_pattern_after_s = None
        self.publish_status('summit_finale_start_accepted')
        self.get_logger().info(
            f'Summit finale accepted: {response.message}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SummitFinaleOrchestrator()
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
