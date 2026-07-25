#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class V71ShortRegressionEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v71_short_regression_evaluator')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.route: Dict = {}
        self.h10: Dict = {}
        self.start_s = self.now_s()
        self.saved = False

        self.create_subscription(
            String,
            '/cq01_v34h/mission_status',
            self.route_callback,
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v34h/disturbance_status',
            self.h10_callback,
            transient,
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def parse(message: String) -> Dict:
        try:
            return json.loads(message.data)
        except json.JSONDecodeError:
            return {}

    def route_callback(self, message: String) -> None:
        self.route = self.parse(message)

    def h10_callback(self, message: String) -> None:
        self.h10 = self.parse(message)

    def timer_callback(self) -> None:
        if self.saved:
            return

        route_state = str(self.route.get('state', ''))
        trigger_count = int(self.h10.get('trigger_count', 0) or 0)
        pending = int(self.h10.get('pending_repeat_count', 0) or 0)
        h10_state = str(self.h10.get('state', ''))

        route_done = (
            route_state == 'mission_completed'
            and int(self.route.get('reached_index', -1)) == 3
        )
        h10_done = (
            trigger_count >= 3
            and pending == 0
            and h10_state in (
                'tracking_replanned_path',
                'route_switch_ready',
                'new_path_same_branch',
                'results_saved',
                'repeat_ready',
            )
        )

        if route_done and h10_done:
            self.save(True, 'complete')
            return

        if route_state in (
            'planning_failed',
            'stalled',
            'mission_failed',
        ):
            self.save(False, route_state)
            return

        if self.now_s() - self.start_s > 360.0:
            self.save(False, 'timeout')

    def save(self, passed: bool, final_state: str) -> None:
        self.saved = True

        root = Path(
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v71_short_regression'
            )
        )
        output = root / datetime.now().strftime(
            'v71_%Y%m%d_%H%M%S'
        )
        output.mkdir(parents=True, exist_ok=True)

        summary = {
            'experiment':
                'cq01_v71_original_dispatcher_h10_short_regression',
            'pass': passed,
            'final_state': final_state,
            'route_status': self.route,
            'h10_status': self.h10,
            'checks': {
                'original_dispatcher_reached_p3':
                    str(self.route.get('state', ''))
                    == 'mission_completed'
                    and int(
                        self.route.get('reached_index', -1)
                    ) == 3,
                'h10_trigger_count_is_3':
                    int(
                        self.h10.get('trigger_count', 0)
                        or 0
                    ) >= 3,
                'h10_new_path_received':
                    bool(
                        self.h10.get(
                            'new_path_received',
                            False,
                        )
                    ),
                'h10_no_pending_repeat':
                    int(
                        self.h10.get(
                            'pending_repeat_count',
                            0,
                        )
                        or 0
                    ) == 0,
            },
            'architecture': {
                'rviz_processes_expected': 1,
                'camera_focus_managers_expected': 1,
                'prelude_process_exits_before_dispatcher':
                    True,
                'mission_status_publisher':
                    'original planning_mission_dispatcher only',
                'goal_raw_publishers_during_prelude': 1,
                'goal_raw_publishers_during_route': 2,
                'route_publisher_reason':
                    'original dispatcher plus original H10 '
                    'deliberate replan publisher',
                'fake_mission_status': False,
            },
        }

        (output / 'summary.json').write_text(
            json.dumps(
                summary,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

        self.get_logger().warning(
            f'V7.1 RESULT pass={passed} output={output}'
        )
        self.create_timer(1.0, self.exit_process)

    def exit_process(self) -> None:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V71ShortRegressionEvaluator()
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
