#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class V8BRuntimeMonitor(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8b_runtime_monitor')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.results_root = Path(
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v8b_high_altitude_pattern'
            )
        )
        self.route: Dict = {}
        self.h14_status: Dict = {}
        self.pattern_status: Dict = {}
        self.pattern_result: Optional[Dict] = None
        self.integrated: Dict = {}
        self.raw_mux: Dict = {}
        self.reference_poses = 0
        self.astar_poses = 0
        self.pattern_reference_poses = 0
        self.actual_poses = 0
        self.maximum_goal_raw_publishers = 0
        self.maximum_external_wrench_publishers = 0
        self.h14_summary_path: Optional[Path] = None
        self.saved = False
        self.start_s = self.now_s()

        subscriptions = [
            ('route', '/cq01_v8b/route_status'),
            ('h14_status', '/cq01_v8b/h14/status'),
            ('pattern_status', '/cq01_v8b/pattern/status'),
            ('integrated', '/cq01_v8b/integrated/status'),
            ('raw_mux', '/cq01_v8b/raw_goal_mux_status'),
        ]
        for target, topic in subscriptions:
            self.create_subscription(
                String,
                topic,
                lambda message, target=target:
                    self.json_callback(target, message),
                transient,
            )

        self.create_subscription(
            String,
            '/cq01_v8b/pattern/result',
            self.pattern_result_callback,
            transient,
        )
        self.create_subscription(
            NavPath,
            '/cq01_v2/reference_path',
            lambda message:
                self.path_callback('reference', message),
            transient,
        )
        self.create_subscription(
            NavPath,
            '/drone/planned_path_3d',
            lambda message:
                self.path_callback('astar', message),
            transient,
        )
        self.create_subscription(
            NavPath,
            '/cq01_v8b/pattern/reference_path',
            lambda message:
                self.path_callback('pattern_reference', message),
            transient,
        )
        self.create_subscription(
            NavPath,
            '/cq01_v8b/actual_trajectory',
            lambda message:
                self.path_callback('actual', message),
            10,
        )

        self.timer = self.create_timer(
            0.5,
            self.timer_callback,
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def json_callback(self, target: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        setattr(self, target, payload)

        if target == 'h14_status':
            output_dir = payload.get('output_dir')
            if output_dir:
                self.h14_summary_path = (
                    Path(
                        os.path.expanduser(
                            str(output_dir)
                        )
                    )
                    / 'summary.json'
                )

    def pattern_result_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.pattern_result = payload

    def path_callback(
        self,
        target: str,
        message: NavPath,
    ) -> None:
        count = len(message.poses)
        if target == 'reference':
            self.reference_poses = max(
                self.reference_poses,
                count,
            )
        elif target == 'astar':
            self.astar_poses = max(
                self.astar_poses,
                count,
            )
        elif target == 'pattern_reference':
            self.pattern_reference_poses = max(
                self.pattern_reference_poses,
                count,
            )
        elif target == 'actual':
            self.actual_poses = max(
                self.actual_poses,
                count,
            )

    @staticmethod
    def read_json(path: Optional[Path]) -> Optional[Dict]:
        if path is None or not path.is_file():
            return None
        try:
            return json.loads(
                path.read_text(encoding='utf-8')
            )
        except Exception:
            return None

    def node_count(self, exact_name: str) -> int:
        target = (
            exact_name
            if exact_name.startswith('/')
            else '/' + exact_name
        )
        count = 0
        for name, namespace in (
            self.get_node_names_and_namespaces()
        ):
            namespace = (namespace or '/').rstrip('/')
            full = (
                f'{namespace}/{name}'
                if namespace
                else f'/{name}'
            )
            if full == target:
                count += 1
        return count

    @staticmethod
    def h14_functional_criteria(
        summary: Optional[Dict],
    ) -> Dict:
        source = (
            summary.get('criteria', {})
            if summary else {}
        )
        keys = (
            'gust_applied',
            'visible_deviation',
            'replan_requested',
            'new_path_received',
            'visible_new_path_branch',
            'tracking_recovered',
            'tilt_pass',
            'rpm_pass',
            'no_safety_abort',
        )
        return {
            key: source.get(key) is True
            for key in keys
        }

    def timer_callback(self) -> None:
        if self.saved:
            return

        self.maximum_goal_raw_publishers = max(
            self.maximum_goal_raw_publishers,
            self.count_publishers('/drone/goal_raw'),
        )
        self.maximum_external_wrench_publishers = max(
            self.maximum_external_wrench_publishers,
            self.count_publishers(
                '/drone/external_wrench'
            ),
        )

        h14_summary = self.read_json(
            self.h14_summary_path
        )
        route_done = (
            self.route.get('state') == 'mission_completed'
            and int(
                self.route.get('reached_index', -1)
            ) == 15
        )
        pattern_done = bool(
            self.pattern_result
            and self.pattern_result.get('pass') is True
        )

        if (
            route_done
            and h14_summary is not None
            and self.pattern_result is not None
            and self.integrated.get('state')
            in ('summit_pattern_completed', 'integrated_completed')
        ):
            self.save_result(
                h14_summary,
                'all_results_received',
            )
            return

        if self.route.get('state') in (
            'planning_failed',
            'stalled',
        ):
            self.save_result(
                h14_summary,
                str(self.route.get('state')),
            )
            return

        if self.integrated.get('state') == 'integrated_failed':
            self.save_result(
                h14_summary,
                'integrated_failed',
            )
            return

        if self.now_s() - self.start_s > 300.0:
            self.save_result(
                h14_summary,
                'timeout',
            )

    def save_result(
        self,
        h14_summary: Optional[Dict],
        final_state: str,
    ) -> None:
        if self.saved:
            return
        self.saved = True

        h14_functional = self.h14_functional_criteria(
            h14_summary
        )
        h14_functional_pass = all(
            h14_functional.values()
        )
        pattern_result = self.pattern_result or {}
        final_error = (
            pattern_result
            .get('overall_metrics', {})
            .get('final_hold_error_m')
        )

        node_counts = {
            'rviz':
                self.node_count('/cq01_v8b_rviz'),
            'dispatcher':
                self.node_count(
                    '/cq01_v8b_route_dispatcher'
                ),
            'raw_goal_mux':
                self.node_count(
                    '/cq01_v8b_raw_goal_mux'
                ),
            'h14':
                self.node_count(
                    '/cq01_v8b_high_altitude_wind'
                ),
            'goal_mux':
                self.node_count(
                    '/cq01_v8b_goal_mux'
                ),
            'pattern_manager':
                self.node_count(
                    '/cq01_v8b_summit_pattern_manager'
                ),
            'pattern_evaluator':
                self.node_count(
                    '/cq01_v8b_pattern_evaluator'
                ),
            'orchestrator':
                self.node_count(
                    '/cq01_v8b_summit_orchestrator'
                ),
            'camera':
                self.node_count(
                    '/cq01_v8b_camera_focus'
                ),
            'astar':
                self.node_count('/astar_3d_planner'),
            'controller':
                self.node_count('/quadrotor_controller'),
            'dynamics':
                self.node_count('/quadrotor_dynamics'),
        }

        criteria = {
            'h14_functional_pass':
                h14_functional_pass,
            'h14_original_result_received':
                h14_summary is not None,
            'h14_original_pass_preserved':
                h14_summary is not None
                and 'pass' in h14_summary,
            'route_completed_at_P15':
                self.route.get('state')
                == 'mission_completed'
                and int(
                    self.route.get(
                        'reached_index',
                        -1,
                    )
                ) == 15,
            'pattern_authoritative_pass':
                pattern_result.get('pass') is True,
            'final_error_at_most_0_3m':
                final_error is not None
                and float(final_error) <= 0.3,
            'integrated_sequence_completed':
                self.integrated.get('state')
                in (
                    'summit_pattern_completed',
                    'integrated_completed',
                ),
            'h14_replan_forwarded_by_mux':
                int(
                    self.raw_mux.get(
                        'h14_replan_forward_count',
                        0,
                    )
                ) >= 1,
            'full_standard_route_received':
                self.reference_poses >= 16,
            'astar_path_received':
                self.astar_poses >= 2,
            'pattern_reference_received':
                self.pattern_reference_poses >= 2,
            'actual_trajectory_received':
                self.actual_poses >= 2,
            'goal_raw_one_publisher':
                self.maximum_goal_raw_publishers <= 1,
            'external_wrench_one_publisher':
                self.maximum_external_wrench_publishers <= 1,
            'one_rviz':
                node_counts['rviz'] == 1,
            'one_dispatcher':
                node_counts['dispatcher'] == 1,
            'one_raw_goal_mux':
                node_counts['raw_goal_mux'] == 1,
            'one_h14':
                node_counts['h14'] == 1,
            'one_goal_mux':
                node_counts['goal_mux'] == 1,
            'one_pattern_manager':
                node_counts['pattern_manager'] == 1,
            'one_pattern_evaluator':
                node_counts['pattern_evaluator'] == 1,
            'one_orchestrator':
                node_counts['orchestrator'] == 1,
            'one_camera':
                node_counts['camera'] == 1,
        }

        failed = [
            key
            for key, value in criteria.items()
            if value is not True
        ]

        output_dir = (
            self.results_root
            / datetime.now().strftime(
                'v8b_%Y%m%d_%H%M%S'
            )
        )
        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )
        summary_path = output_dir / 'summary.json'

        summary = {
            'experiment':
                'cq01_v8b_h14_functional_replan_and_p31_pattern',
            'final_state':
                final_state,
            'pass':
                not failed,
            'failed_criteria':
                failed,
            'criteria':
                criteria,
            'route_status':
                self.route,
            'h14_status':
                self.h14_status,
            'h14_original_summary_path':
                str(self.h14_summary_path)
                if self.h14_summary_path else None,
            'h14_original_summary':
                h14_summary,
            'h14_original_pass':
                h14_summary.get('pass')
                if h14_summary else None,
            'h14_target_5m_reached':
                h14_summary.get(
                    'target_deviation_reached'
                )
                if h14_summary else None,
            'h14_functional_criteria':
                h14_functional,
            'h14_functional_pass':
                h14_functional_pass,
            'pattern_status':
                self.pattern_status,
            'pattern_authoritative_result':
                pattern_result,
            'integrated_status':
                self.integrated,
            'raw_goal_mux_status':
                self.raw_mux,
            'visual_topics': {
                'standard_route': {
                    'topic':
                        '/cq01_v2/reference_path',
                    'poses':
                        self.reference_poses,
                },
                'current_astar': {
                    'topic':
                        '/drone/planned_path_3d',
                    'poses':
                        self.astar_poses,
                },
                'pattern_reference': {
                    'topic':
                        '/cq01_v8b/pattern/reference_path',
                    'poses':
                        self.pattern_reference_poses,
                },
                'actual_trajectory': {
                    'topic':
                        '/cq01_v8b/actual_trajectory',
                    'poses':
                        self.actual_poses,
                },
            },
            'architecture': {
                'node_counts':
                    node_counts,
                'maximum_goal_raw_publishers':
                    self.maximum_goal_raw_publishers,
                'maximum_external_wrench_publishers':
                    self.maximum_external_wrench_publishers,
                'initial_position':
                    [309.0, -45.0, 73.0],
                'route_range':
                    [11, 15],
                'rviz_restart_present':
                    False,
                'odom_switching_present':
                    False,
                'fake_mission_status_present':
                    False,
            },
        }
        summary_path.write_text(
            json.dumps(
                summary,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

        self.get_logger().warning(
            'V8-B RESULT '
            f'pass={summary["pass"]}; '
            f'h14_original_pass={summary["h14_original_pass"]}; '
            f'h14_functional_pass={h14_functional_pass}; '
            f'pattern_pass={pattern_result.get("pass")}; '
            f'failed_criteria={failed}; '
            f'output={summary_path}'
        )
        self.get_logger().warning(
            'RViz remains open for inspection. '
            'Press Ctrl+C after reviewing the H14 replan and '
            'the summit circle/figure-eight.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8BRuntimeMonitor()
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
