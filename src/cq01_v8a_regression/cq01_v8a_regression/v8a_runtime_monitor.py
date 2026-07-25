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


class V8ARuntimeMonitor(Node):
    """Read-only integration monitor.

    It never publishes goals, wrenches, odometry, or mission status. The
    authoritative pass/fail remains the original H4 summary.json.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v8a_runtime_monitor')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.results_root = Path(
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v8a_low_altitude_regression'
            )
        )

        self.mission: Dict = {}
        self.disturbance: Dict = {}
        self.reference_path_poses = 0
        self.astar_path_poses = 0
        self.actual_trajectory_poses = 0
        self.maximum_goal_raw_publishers = 0
        self.saved = False
        self.start_s = self.now_s()
        self.pending_authoritative_path: Optional[Path] = None
        self.pending_since_s: Optional[float] = None

        self.status_pub = self.create_publisher(
            String,
            '/cq01_v8a/regression_status',
            transient,
        )

        self.create_subscription(
            String,
            '/cq01_v8a/mission_status',
            self.mission_callback,
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8a/disturbance_status',
            self.disturbance_callback,
            transient,
        )
        self.create_subscription(
            NavPath,
            '/cq01_v2/reference_path',
            self.reference_callback,
            transient,
        )
        self.create_subscription(
            NavPath,
            '/drone/planned_path_3d',
            self.astar_callback,
            transient,
        )
        self.create_subscription(
            NavPath,
            '/cq01_v8a/actual_trajectory',
            self.actual_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.publish_status('monitor_ready')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def parse_json(message: String) -> Dict:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return {}
        return payload if isinstance(payload, dict) else {}

    def mission_callback(self, message: String) -> None:
        self.mission = self.parse_json(message)

    def disturbance_callback(self, message: String) -> None:
        self.disturbance = self.parse_json(message)
        if self.disturbance.get('state') != 'results_saved':
            return
        output_dir = self.disturbance.get('output_dir')
        if not output_dir:
            return
        self.pending_authoritative_path = (
            Path(os.path.expanduser(str(output_dir))) / 'summary.json'
        )
        if self.pending_since_s is None:
            self.pending_since_s = self.now_s()

    def reference_callback(self, message: NavPath) -> None:
        self.reference_path_poses = max(
            self.reference_path_poses,
            len(message.poses),
        )

    def astar_callback(self, message: NavPath) -> None:
        self.astar_path_poses = max(
            self.astar_path_poses,
            len(message.poses),
        )

    def actual_callback(self, message: NavPath) -> None:
        self.actual_trajectory_poses = max(
            self.actual_trajectory_poses,
            len(message.poses),
        )

    def publish_status(self, state: str, **extra) -> None:
        payload = {
            'state': state,
            'mission_state': self.mission.get('state'),
            'mission_current_index': self.mission.get('current_index'),
            'disturbance_state': self.disturbance.get('state'),
            'reference_path_poses': self.reference_path_poses,
            'astar_path_poses': self.astar_path_poses,
            'actual_trajectory_poses': self.actual_trajectory_poses,
            'maximum_goal_raw_publishers':
                self.maximum_goal_raw_publishers,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)

    def node_count(self, exact_name: str) -> int:
        # rclpy.get_node_names() returns short names, while ros2 node list
        # displays fully qualified names. Count fully qualified names from
        # get_node_names_and_namespaces() to avoid false zero counts.
        target = exact_name
        if not target.startswith('/'):
            target = '/' + target

        count = 0
        for name, namespace in self.get_node_names_and_namespaces():
            namespace = namespace or '/'
            if not namespace.startswith('/'):
                namespace = '/' + namespace
            namespace = namespace.rstrip('/')
            fully_qualified = (
                f'{namespace}/{name}'
                if namespace
                else f'/{name}'
            )
            if fully_qualified == target:
                count += 1
        return count

    def timer_callback(self) -> None:
        if self.saved:
            self.publish_status('result_saved')
            return

        self.maximum_goal_raw_publishers = max(
            self.maximum_goal_raw_publishers,
            self.count_publishers('/drone/goal_raw'),
        )

        if self.mission.get('state') in (
            'planning_failed',
            'stalled',
        ):
            self.save_wrapper(
                authoritative=None,
                final_state=str(self.mission.get('state')),
            )
            return

        if self.pending_authoritative_path is not None:
            if self.pending_authoritative_path.is_file():
                try:
                    authoritative = json.loads(
                        self.pending_authoritative_path.read_text(
                            encoding='utf-8'
                        )
                    )
                except Exception as exception:
                    self.get_logger().error(
                        f'Cannot read authoritative H4 summary: {exception}'
                    )
                    authoritative = None
                self.save_wrapper(
                    authoritative=authoritative,
                    final_state='authoritative_h4_result_received',
                )
                return

            if (
                self.pending_since_s is not None
                and self.now_s() - self.pending_since_s > 8.0
            ):
                self.save_wrapper(
                    authoritative=None,
                    final_state='h4_summary_file_missing',
                )
                return

        if self.now_s() - self.start_s > 300.0:
            self.save_wrapper(
                authoritative=None,
                final_state='timeout',
            )
            return

        self.publish_status('running')

    def save_wrapper(
        self,
        authoritative: Optional[Dict],
        final_state: str,
    ) -> None:
        if self.saved:
            return
        self.saved = True

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = self.results_root / f'v8a_{timestamp}'
        output_dir.mkdir(parents=True, exist_ok=True)

        mission_completed_p4 = (
            self.mission.get('state') == 'mission_completed'
            and int(self.mission.get('reached_index', -1)) == 4
        )
        authoritative_pass = bool(
            authoritative
            and authoritative.get('pass') is True
        )

        node_counts = {
            'rviz': self.node_count('/cq01_v8a_rviz'),
            'dispatcher':
                self.node_count('/cq01_v8a_route_dispatcher'),
            'external_disturbance':
                self.node_count('/cq01_v8a_external_disturbance'),
            'runtime_monitor':
                self.node_count('/cq01_v8a_runtime_monitor'),
            'astar': self.node_count('/astar_3d_planner'),
            'controller': self.node_count('/quadrotor_controller'),
            'dynamics': self.node_count('/quadrotor_dynamics'),
        }

        criteria = {
            'authoritative_h4_pass': authoritative_pass,
            'mission_completed_at_p4': mission_completed_p4,
            'full_standard_route_received':
                self.reference_path_poses >= 16,
            'current_astar_path_received':
                self.astar_path_poses >= 2,
            'actual_trajectory_received':
                self.actual_trajectory_poses >= 2,
            'goal_raw_never_exceeded_one_publisher':
                self.maximum_goal_raw_publishers <= 1,
            'exactly_one_rviz':
                node_counts['rviz'] == 1,
            'exactly_one_dispatcher':
                node_counts['dispatcher'] == 1,
            'exactly_one_external_disturbance_manager':
                node_counts['external_disturbance'] == 1,
        }

        failed_criteria = [
            key
            for key, value in criteria.items()
            if value is not True
        ]

        wrapper = {
            'experiment':
                'cq01_v8a_low_altitude_authoritative_regression',
            'final_state': final_state,
            'pass': all(criteria.values()),
            'failed_criteria': failed_criteria,
            'criteria': criteria,
            'mission_status': self.mission,
            'disturbance_status': self.disturbance,
            'visual_topics': {
                'standard_route': {
                    'topic': '/cq01_v2/reference_path',
                    'maximum_pose_count':
                        self.reference_path_poses,
                    'rviz_color': 'green',
                },
                'current_astar': {
                    'topic': '/drone/planned_path_3d',
                    'maximum_pose_count':
                        self.astar_path_poses,
                    'rviz_color': 'red',
                },
                'actual_trajectory': {
                    'topic': '/cq01_v8a/actual_trajectory',
                    'maximum_pose_count':
                        self.actual_trajectory_poses,
                    'rviz_color': 'yellow',
                },
            },
            'architecture': {
                'node_counts_at_result': node_counts,
                'maximum_goal_raw_publishers':
                    self.maximum_goal_raw_publishers,
                'rviz_restart_mechanism_present': False,
                'odom_source_switching_present': False,
                'fake_mission_status_present': False,
                'h10_present': False,
                'h14_present': False,
                'prelude_present': False,
            },
            'authoritative_h4_summary_path':
                str(self.pending_authoritative_path)
                if self.pending_authoritative_path else None,
            'authoritative_h4_summary': authoritative,
        }

        summary_path = output_dir / 'summary.json'
        summary_path.write_text(
            json.dumps(wrapper, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

        self.get_logger().warning(
            'V8-A RESULT '
            f'pass={wrapper["pass"]}; '
            f'authoritative_h4_pass={authoritative_pass}; '
            f'failed_criteria={failed_criteria}; '
            f'output={summary_path}'
        )
        self.get_logger().warning(
            'RViz remains open for inspection. Press Ctrl+C after '
            'reviewing the green standard route, red A* path, yellow '
            'actual trajectory, and disturbance evidence.'
        )
        self.publish_status(
            'result_saved',
            pass_value=wrapper['pass'],
            summary_path=str(summary_path),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8ARuntimeMonitor()
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
