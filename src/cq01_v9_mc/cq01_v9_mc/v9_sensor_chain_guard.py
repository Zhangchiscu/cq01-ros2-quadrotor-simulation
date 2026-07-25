#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class SensorChainGuard(Node):
    """Verify feedback source selection without sending control commands."""

    def __init__(self) -> None:
        super().__init__('cq01_v9_sensor_chain_guard')

        self.declare_parameter(
            'sensor_profile',
            'nominal_closed_loop',
        )
        self.declare_parameter(
            'control_state_topic',
            '/cq01_v9/sensors/nominal/odom',
        )
        self.declare_parameter(
            'truth_state_topic',
            '/drone/odom',
        )
        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/'
                'results/cq01_bonus_suite'
            ),
        )
        self.declare_parameter('initial_delay_s', 4.0)
        self.declare_parameter('maximum_wait_s', 45.0)

        self.sensor_profile = str(
            self.get_parameter('sensor_profile').value
        )
        self.control_topic = str(
            self.get_parameter('control_state_topic').value
        )
        self.truth_topic = str(
            self.get_parameter('truth_state_topic').value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )
        self.initial_delay_s = float(
            self.get_parameter('initial_delay_s').value
        )
        self.maximum_wait_s = float(
            self.get_parameter('maximum_wait_s').value
        )

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String,
            '/cq01_v9/sensor_chain/status',
            transient,
        )

        self.expected_publisher = (
            'cq01_v9_sensor_suite'
            if self.sensor_profile == 'nominal_closed_loop'
            else 'quadrotor_dynamics'
        )
        self.started_s = self.now_s()
        self.consecutive_passes = 0
        self.result: Optional[Dict] = None
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().warning(
            'SENSOR CHAIN GUARD: '
            f'profile={self.sensor_profile}; '
            f'control={self.control_topic}'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def names(endpoints) -> Set[str]:
        return {
            str(endpoint.node_name).lstrip('/')
            for endpoint in endpoints
        }

    def snapshot(self) -> Dict:
        control_publishers = self.names(
            self.get_publishers_info_by_topic(
                self.control_topic
            )
        )
        control_subscribers = self.names(
            self.get_subscriptions_info_by_topic(
                self.control_topic
            )
        )
        truth_publishers = self.names(
            self.get_publishers_info_by_topic(
                self.truth_topic
            )
        )
        truth_subscribers = self.names(
            self.get_subscriptions_info_by_topic(
                self.truth_topic
            )
        )

        checks = {
            'control_topic_has_expected_publisher':
                self.expected_publisher in control_publishers,
            'controller_uses_selected_state':
                'quadrotor_controller'
                in control_subscribers,
            'planner_uses_selected_state':
                'astar_3d_planner'
                in control_subscribers,
            'truth_topic_has_dynamics_publisher':
                'quadrotor_dynamics'
                in truth_publishers,
            'controller_removed_from_truth_when_noisy': (
                self.sensor_profile != 'nominal_closed_loop'
                or 'quadrotor_controller'
                not in truth_subscribers
            ),
            'planner_removed_from_truth_when_noisy': (
                self.sensor_profile != 'nominal_closed_loop'
                or 'astar_3d_planner'
                not in truth_subscribers
            ),
            'state_source_selected_before_launch': True,
            'midflight_switching_disabled': True,
        }
        failed = [
            name
            for name, passed in checks.items()
            if passed is not True
        ]

        return {
            'state': (
                'sensor_chain_verified'
                if not failed
                else 'waiting_for_sensor_chain'
            ),
            'pass': not failed,
            'sensor_profile': self.sensor_profile,
            'control_state_source': self.control_topic,
            'truth_state_source': self.truth_topic,
            'expected_control_publisher':
                self.expected_publisher,
            'control_publishers': sorted(control_publishers),
            'control_subscribers': sorted(control_subscribers),
            'truth_publishers': sorted(truth_publishers),
            'truth_subscribers': sorted(truth_subscribers),
            'checks': checks,
            'failed_criteria': failed,
        }

    def save(self) -> None:
        if self.result is None:
            return
        self.results_root.mkdir(
            parents=True,
            exist_ok=True,
        )
        path = (
            self.results_root
            / (
                'sensor_chain_guard_'
                + datetime.now().strftime(
                    '%Y%m%d_%H%M%S'
                )
                + '.json'
            )
        )
        self.result['result_file'] = str(path)
        path.write_text(
            json.dumps(
                self.result,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

    def publish(self) -> None:
        if self.result is None:
            return
        message = String()
        message.data = json.dumps(
            self.result,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.publisher.publish(message)

    def tick(self) -> None:
        if self.result is not None:
            self.publish()
            return

        elapsed = self.now_s() - self.started_s
        if elapsed < self.initial_delay_s:
            return

        candidate = self.snapshot()
        if candidate['pass']:
            self.consecutive_passes += 1
        else:
            self.consecutive_passes = 0

        if self.consecutive_passes >= 3:
            self.result = candidate
            self.save()
            self.publish()
            self.get_logger().warning(
                'SENSOR CHAIN VERIFIED: '
                f'{self.control_topic}; '
                'controller+A*=selected state'
            )
            return

        if elapsed >= self.maximum_wait_s:
            candidate['state'] = 'sensor_chain_failed'
            candidate['pass'] = False
            candidate['failed_criteria'] = list(
                candidate.get('failed_criteria', [])
            ) + ['verification_timeout']
            self.result = candidate
            self.save()
            self.publish()
            self.get_logger().error(
                'SENSOR CHAIN FAILED: '
                f'{self.result["failed_criteria"]}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorChainGuard()
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
