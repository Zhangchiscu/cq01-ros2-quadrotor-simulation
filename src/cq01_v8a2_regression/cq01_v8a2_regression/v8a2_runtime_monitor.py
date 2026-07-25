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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class V8A2RuntimeMonitor(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8a2_runtime_monitor')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.results_root = Path(os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v8a2_dual_location'))
        self.mission: Dict = {}
        self.low: Dict = {}
        self.climb: Dict = {}
        self.gate: Dict = {}
        self.mux: Dict = {}
        self.reference_poses = 0
        self.astar_poses = 0
        self.actual_poses = 0
        self.max_goal_publishers = 0
        self.max_wrench_publishers = 0
        self.low_summary_path: Optional[Path] = None
        self.climb_summary_path: Optional[Path] = None
        self.saved = False
        self.start_s = self.now_s()

        self.create_subscription(String, '/cq01_v8a2/mission_status', lambda m: self.json_cb('mission', m), transient)
        self.create_subscription(String, '/cq01_v8a2/low/status', lambda m: self.json_cb('low', m), transient)
        self.create_subscription(String, '/cq01_v8a2/climb/status', lambda m: self.json_cb('climb', m), transient)
        self.create_subscription(String, '/cq01_v8a2/low_gate_status', lambda m: self.json_cb('gate', m), transient)
        self.create_subscription(String, '/cq01_v8a2/wrench_mux_status', lambda m: self.json_cb('mux', m), transient)
        self.create_subscription(NavPath, '/cq01_v2/reference_path', lambda m: self.path_cb('reference', m), transient)
        self.create_subscription(NavPath, '/drone/planned_path_3d', lambda m: self.path_cb('astar', m), transient)
        self.create_subscription(NavPath, '/cq01_v8a2/actual_trajectory', lambda m: self.path_cb('actual', m), 10)
        self.timer = self.create_timer(0.5, self.timer_cb)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def json_cb(self, target: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        setattr(self, target, payload)
        output = payload.get('output_dir')
        if output:
            path = Path(os.path.expanduser(str(output))) / 'summary.json'
            if target == 'low':
                self.low_summary_path = path
            elif target == 'climb':
                self.climb_summary_path = path

    def path_cb(self, target: str, message: NavPath) -> None:
        count = len(message.poses)
        if target == 'reference':
            self.reference_poses = max(self.reference_poses, count)
        elif target == 'astar':
            self.astar_poses = max(self.astar_poses, count)
        else:
            self.actual_poses = max(self.actual_poses, count)

    def fq_node_count(self, exact: str) -> int:
        target = exact if exact.startswith('/') else '/' + exact
        count = 0
        for name, namespace in self.get_node_names_and_namespaces():
            ns = (namespace or '/').rstrip('/')
            fq = f'{ns}/{name}' if ns else f'/{name}'
            if fq == target:
                count += 1
        return count

    @staticmethod
    def read_summary(path: Optional[Path]) -> Optional[Dict]:
        if path is None or not path.is_file():
            return None
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except Exception:
            return None

    def timer_cb(self) -> None:
        if self.saved:
            return
        self.max_goal_publishers = max(self.max_goal_publishers, self.count_publishers('/drone/goal_raw'))
        self.max_wrench_publishers = max(self.max_wrench_publishers, self.count_publishers('/drone/external_wrench'))
        low_summary = self.read_summary(self.low_summary_path)
        climb_summary = self.read_summary(self.climb_summary_path)

        mission_done = self.mission.get('state') == 'mission_completed' and int(self.mission.get('reached_index', -1)) == 4
        if mission_done and low_summary is not None and climb_summary is not None:
            self.save_result(low_summary, climb_summary, 'both_authoritative_results_received')
            return
        if self.mission.get('state') in ('planning_failed', 'stalled'):
            self.save_result(low_summary, climb_summary, str(self.mission.get('state')))
            return
        if self.now_s() - self.start_s > 360.0:
            self.save_result(low_summary, climb_summary, 'timeout')

    def save_result(self, low_summary: Optional[Dict], climb_summary: Optional[Dict], final_state: str) -> None:
        if self.saved:
            return
        self.saved = True
        node_counts = {
            'rviz': self.fq_node_count('/cq01_v8a2_rviz'),
            'dispatcher': self.fq_node_count('/cq01_v8a2_route_dispatcher'),
            'low_disturbance': self.fq_node_count('/cq01_v8a2_low_disturbance'),
            'climb_disturbance': self.fq_node_count('/cq01_v8a2_climb_disturbance'),
            'wrench_mux': self.fq_node_count('/cq01_v8a2_wrench_mux'),
            'camera': self.fq_node_count('/cq01_v8a2_camera_focus'),
            'astar': self.fq_node_count('/astar_3d_planner'),
            'controller': self.fq_node_count('/quadrotor_controller'),
            'dynamics': self.fq_node_count('/quadrotor_dynamics'),
        }
        criteria = {
            'low_city_authoritative_pass': bool(low_summary and low_summary.get('pass') is True),
            'climb_authoritative_pass': bool(climb_summary and climb_summary.get('pass') is True),
            'low_gate_trigger_accepted': bool(self.gate.get('trigger_accepted', False)),
            'mux_low_selected': bool(self.mux.get('low_selected_ever', False)),
            'mux_climb_selected': bool(self.mux.get('climb_selected_ever', False)),
            'mux_no_collision': self.mux.get('collision_ever') is False,
            'mission_completed_at_p4': self.mission.get('state') == 'mission_completed' and int(self.mission.get('reached_index', -1)) == 4,
            'full_standard_route_received': self.reference_poses >= 16,
            'current_astar_received': self.astar_poses >= 2,
            'actual_trajectory_received': self.actual_poses >= 2,
            'goal_raw_one_publisher': self.max_goal_publishers <= 1,
            'external_wrench_one_publisher': self.max_wrench_publishers <= 1,
            'one_rviz': node_counts['rviz'] == 1,
            'one_dispatcher': node_counts['dispatcher'] == 1,
            'one_low_disturbance': node_counts['low_disturbance'] == 1,
            'one_climb_disturbance': node_counts['climb_disturbance'] == 1,
            'one_wrench_mux': node_counts['wrench_mux'] == 1,
            'one_camera': node_counts['camera'] == 1,
        }
        failed = [key for key, value in criteria.items() if value is not True]
        output = self.results_root / datetime.now().strftime('v8a2_%Y%m%d_%H%M%S')
        output.mkdir(parents=True, exist_ok=True)
        summary = {
            'experiment': 'cq01_v8a2_dual_location_crosswind_regression',
            'final_state': final_state,
            'pass': not failed,
            'failed_criteria': failed,
            'criteria': criteria,
            'mission_status': self.mission,
            'low_gate_status': self.gate,
            'wrench_mux_status': self.mux,
            'low_status': self.low,
            'climb_status': self.climb,
            'low_summary_path': str(self.low_summary_path) if self.low_summary_path else None,
            'climb_summary_path': str(self.climb_summary_path) if self.climb_summary_path else None,
            'low_authoritative_summary': low_summary,
            'climb_authoritative_summary': climb_summary,
            'visual_topics': {
                'standard_route': {'topic': '/cq01_v2/reference_path', 'poses': self.reference_poses, 'color': 'green'},
                'current_astar': {'topic': '/drone/planned_path_3d', 'poses': self.astar_poses, 'color': 'red'},
                'actual_trajectory': {'topic': '/cq01_v8a2/actual_trajectory', 'poses': self.actual_poses, 'color': 'yellow'},
                'low_markers': '/cq01_v8a2/low/markers',
                'climb_markers': '/cq01_v8a2/climb/markers',
            },
            'architecture': {
                'node_counts': node_counts,
                'max_goal_raw_publishers': self.max_goal_publishers,
                'max_external_wrench_publishers': self.max_wrench_publishers,
                'rviz_restart_present': False,
                'odom_switching_present': False,
                'fake_mission_status_present': False,
            },
        }
        summary_path = output / 'summary.json'
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        self.get_logger().warning(
            f'V8-A2 RESULT pass={summary["pass"]}; failed_criteria={failed}; output={summary_path}'
        )
        self.get_logger().warning('RViz remains open for inspection. Press Ctrl+C after reviewing both wind locations.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A2RuntimeMonitor()
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
