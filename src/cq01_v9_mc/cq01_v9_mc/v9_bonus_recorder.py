#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64, String


Point3 = Tuple[float, float, float]


class V9BonusRecorder(Node):
    """Unified read-only telemetry recorder for the CQ01 closed loop."""

    def __init__(self) -> None:
        super().__init__('cq01_v9_bonus_recorder')
        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/cq01_bonus_suite'
            ),
        )
        self.declare_parameter(
            'workspace_root',
            os.path.expanduser('~/drone_task/your_drone_sim_ws'),
        )
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('vehicle_profile', 'nominal')
        self.declare_parameter('sensor_profile', 'parallel_monitor')
        self.declare_parameter('control_state_source', '/drone/odom')
        self.declare_parameter('maximum_rpm', 10000.0)

        self.results_root = Path(
            os.path.expanduser(str(self.get_parameter('results_root').value))
        )
        self.workspace_root = Path(
            os.path.expanduser(str(self.get_parameter('workspace_root').value))
        )
        self.vehicle_profile = str(
            self.get_parameter('vehicle_profile').value
        )
        self.sensor_profile = str(
            self.get_parameter('sensor_profile').value
        )
        self.control_state_source = str(
            self.get_parameter('control_state_source').value
        )
        self.maximum_rpm = float(self.get_parameter('maximum_rpm').value)
        rate = max(1.0, float(self.get_parameter('sample_rate_hz').value))

        profile_tag = self.vehicle_profile
        if self.sensor_profile not in ('truth', 'parallel_monitor'):
            profile_tag += f'_{self.sensor_profile}'
        self.run_dir = self.results_root / datetime.now().strftime(
            f'{profile_tag}_full_%Y%m%d_%H%M%S'
        )
        self.run_dir.mkdir(parents=True, exist_ok=True)

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, '/cq01_v9/bonus_recorder/status', transient
        )

        self.position: Optional[Point3] = None
        self.velocity: Optional[Point3] = None
        self.goal: Optional[Point3] = None
        self.rpm: Tuple[float, ...] = ()
        self.clearance_m: Optional[float] = None
        self.force_n = 0.0
        self.statuses: Dict[str, Dict[str, Any]] = {}
        self.samples: List[Dict[str, Any]] = []
        self.planning_times_ms: List[float] = []
        self.last_planner_signature = None
        self.last_sample_position: Optional[Point3] = None
        self.path_length_m = 0.0
        self.minimum_clearance_m = float('inf')
        self.maximum_rpm_seen = 0.0
        self.maximum_speed_mps = 0.0
        self.maximum_force_n = 0.0
        self.rpm_saturation_samples = 0
        self.start_time_s = self.now_s()
        self.finalized = False
        self.finalize_reason = ''

        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, 30)
        self.create_subscription(PoseStamped, '/drone/goal', self.goal_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/drone/motor_rpm', self.rpm_cb, 20
        )
        self.create_subscription(
            Float64,
            '/cq01_v9/course/body_clearance',
            self.clearance_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            '/drone/external_wrench_applied',
            self.wrench_cb,
            20,
        )

        status_topics = {
            'prelude': '/cq01_v9/prelude/status',
            'route': '/cq01_v9/route/status',
            'sensors': '/cq01_v9/sensors/status',
            'sensor_chain': '/cq01_v9/sensor_chain/status',
            'low_strong': '/cq01_v9/low_strong/status',
            'low_replan': '/cq01_v9/low_replan/status',
            'climb': '/cq01_v9/climb/status',
            'h14': '/cq01_v9/h14/status',
            'dynamic': '/cq01_v9/dynamic/status',
            'pattern': '/cq01_v9/pattern/status',
            'pattern_result': '/cq01_v9/pattern/result',
            'integrated': '/cq01_v9/integrated/status',
            'planner': '/drone/planner_3d/status',
            'course_clearance': '/cq01_v9/course/clearance_status',
            'profile': '/cq01_v9/profile/status',
        }
        for key, topic in status_topics.items():
            self.create_subscription(
                String,
                topic,
                lambda message, status_key=key: self.status_cb(
                    status_key, message
                ),
                transient,
            )

        self.write_manifest()
        self.sample_timer = self.create_timer(1.0 / rate, self.sample)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().warning(
            f'Unified bonus recorder started: {self.run_dir}'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def _point_from_odom(message: Odometry) -> Point3:
        point = message.pose.pose.position
        return float(point.x), float(point.y), float(point.z)

    @staticmethod
    def _speed_from_odom(message: Odometry) -> Point3:
        value = message.twist.twist.linear
        return float(value.x), float(value.y), float(value.z)

    def odom_cb(self, message: Odometry) -> None:
        self.position = self._point_from_odom(message)
        self.velocity = self._speed_from_odom(message)

    def goal_cb(self, message: PoseStamped) -> None:
        point = message.pose.position
        self.goal = float(point.x), float(point.y), float(point.z)

    def rpm_cb(self, message: Float32MultiArray) -> None:
        self.rpm = tuple(float(v) for v in message.data[:4])

    def clearance_cb(self, message: Float64) -> None:
        self.clearance_m = float(message.data)

    def wrench_cb(self, message: WrenchStamped) -> None:
        force = message.wrench.force
        self.force_n = math.sqrt(
            force.x * force.x + force.y * force.y + force.z * force.z
        )

    def status_cb(self, key: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {'raw': message.data}
        self.statuses[key] = payload

        if key == 'planner':
            planning_time = payload.get('planning_time_ms')
            signature = (
                payload.get('state'),
                payload.get('reason'),
                planning_time,
                payload.get('path_point_count'),
            )
            if planning_time is not None and signature != self.last_planner_signature:
                try:
                    self.planning_times_ms.append(float(planning_time))
                    self.last_planner_signature = signature
                except (TypeError, ValueError):
                    pass

    def sample(self) -> None:
        if self.finalized or self.position is None:
            return

        elapsed = self.now_s() - self.start_time_s
        if self.last_sample_position is not None:
            self.path_length_m += math.dist(
                self.last_sample_position, self.position
            )
        self.last_sample_position = self.position

        speed = math.sqrt(sum(value * value for value in (self.velocity or (0, 0, 0))))
        error = math.dist(self.position, self.goal) if self.goal else None
        max_rpm = max(self.rpm) if self.rpm else 0.0

        if self.clearance_m is not None:
            self.minimum_clearance_m = min(
                self.minimum_clearance_m, self.clearance_m
            )
        self.maximum_rpm_seen = max(self.maximum_rpm_seen, max_rpm)
        self.maximum_speed_mps = max(self.maximum_speed_mps, speed)
        self.maximum_force_n = max(self.maximum_force_n, self.force_n)
        if max_rpm >= 0.98 * self.maximum_rpm:
            self.rpm_saturation_samples += 1

        ground = self.statuses.get('route', {})
        planner = self.statuses.get('planner', {})
        pattern = self.statuses.get('pattern', {})
        row = {
            'time_s': elapsed,
            'x_m': self.position[0],
            'y_m': self.position[1],
            'z_m': self.position[2],
            'vx_mps': (self.velocity or (0, 0, 0))[0],
            'vy_mps': (self.velocity or (0, 0, 0))[1],
            'vz_mps': (self.velocity or (0, 0, 0))[2],
            'speed_mps': speed,
            'goal_x_m': self.goal[0] if self.goal else '',
            'goal_y_m': self.goal[1] if self.goal else '',
            'goal_z_m': self.goal[2] if self.goal else '',
            'position_error_m': '' if error is None else error,
            'rpm_1': self.rpm[0] if len(self.rpm) > 0 else '',
            'rpm_2': self.rpm[1] if len(self.rpm) > 1 else '',
            'rpm_3': self.rpm[2] if len(self.rpm) > 2 else '',
            'rpm_4': self.rpm[3] if len(self.rpm) > 3 else '',
            'max_rpm': max_rpm,
            'clearance_m': '' if self.clearance_m is None else self.clearance_m,
            'applied_force_n': self.force_n,
            'route_state': ground.get('state', ''),
            'route_index': ground.get('current_index', ''),
            'planner_state': planner.get('state', ''),
            'planning_time_ms': planner.get('planning_time_ms', ''),
            'pattern_state': pattern.get('state', ''),
        }
        self.samples.append(row)

        pattern_result = self.statuses.get('pattern_result', {})
        integrated = self.statuses.get('integrated', {})
        completed = (
            pattern_result.get('pass') is True
            and integrated.get('state') in (
                'summit_pattern_completed',
                'integrated_completed',
            )
        )
        if completed:
            self.finalize('integrated_completed')

    def write_manifest(self) -> None:
        commit = 'unknown'
        try:
            commit = subprocess.check_output(
                ['git', '-C', str(self.workspace_root), 'rev-parse', 'HEAD'],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=2,
            ).strip()
        except Exception:
            pass

        manifest = {
            'experiment': f'cq01_chongqing_bonus_{self.vehicle_profile}_{self.sensor_profile}_full',
            'start_time': datetime.now().isoformat(timespec='seconds'),
            'vehicle_profile': self.vehicle_profile,
            'sensor_profile': self.sensor_profile,
            'workspace_root': str(self.workspace_root),
            'git_commit': commit,
            'control_state_source': self.control_state_source,
            'read_only_recorder': True,
            'control_publishers_created': 0,
            'topics_recorded': [
                '/drone/odom',
                '/drone/goal',
                '/drone/motor_rpm',
                '/drone/planner_3d/status',
                '/drone/external_wrench_applied',
                '/cq01_v9/course/body_clearance',
                '/cq01_v9/*/status',
            ],
        }
        (self.run_dir / 'experiment_manifest.json').write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

    def publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                'state': 'completed' if self.finalized else 'recording',
                'read_only': True,
                'publishes_control_topics': False,
                'sample_count': len(self.samples),
                'path_length_m': self.path_length_m,
                'minimum_clearance_m': (
                    None
                    if not math.isfinite(self.minimum_clearance_m)
                    else self.minimum_clearance_m
                ),
                'maximum_rpm': self.maximum_rpm_seen,
                'output_dir': str(self.run_dir),
                'finalize_reason': self.finalize_reason,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)

    def finalize(self, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self.finalize_reason = reason

        if self.samples:
            with (self.run_dir / 'telemetry.csv').open(
                'w', newline='', encoding='utf-8'
            ) as handle:
                writer = csv.DictWriter(
                    handle, fieldnames=list(self.samples[0].keys())
                )
                writer.writeheader()
                writer.writerows(self.samples)

        duration_s = self.samples[-1]['time_s'] if self.samples else 0.0
        final_error = None
        if self.position is not None and self.goal is not None:
            final_error = math.dist(self.position, self.goal)

        planning_mean = (
            sum(self.planning_times_ms) / len(self.planning_times_ms)
            if self.planning_times_ms
            else None
        )
        planning_max = max(self.planning_times_ms) if self.planning_times_ms else None
        sensor_metrics = self.statuses.get('sensors', {}).get('metrics', {})
        route = self.statuses.get('route', {})
        pattern_result = self.statuses.get('pattern_result', {})

        checks = {
            'route_P0_P15_completed': route.get('state') == 'mission_completed',
            'summit_pattern_passed': pattern_result.get('pass') is True,
            'telemetry_recorded': len(self.samples) >= 50,
            'clearance_recorded': math.isfinite(self.minimum_clearance_m),
            'rpm_recorded': self.maximum_rpm_seen > 0.0,
            'rpm_not_saturated': self.rpm_saturation_samples == 0,
            'planner_timing_recorded': len(self.planning_times_ms) > 0,
            'profile_verified': self.statuses.get('profile', {}).get('pass') is True,
            'sensor_chain_verified': (
                self.sensor_profile in ('truth', 'parallel_monitor')
                or self.statuses.get('sensor_chain', {}).get('pass') is True
            ),
            'h14_high_wind_replan_retracking': (
                self.statuses.get('h14', {}).get('pass_result') is True
            ),
            'sensor_status_source_matches_launch': (
                self.statuses.get('sensors', {}).get('control_state_source')
                == self.control_state_source
            ),
        }
        failed = [name for name, passed in checks.items() if passed is not True]

        summary = {
            'experiment': f'cq01_chongqing_bonus_{self.vehicle_profile}_{self.sensor_profile}_full',
            'pass': not failed,
            'failed_criteria': failed,
            'checks': checks,
            'vehicle_profile': self.vehicle_profile,
            'sensor_profile': self.sensor_profile,
            'control_state_source': self.control_state_source,
            'metrics': {
                'duration_s': duration_s,
                'sample_count': len(self.samples),
                'path_length_m': self.path_length_m,
                'final_position_error_m': final_error,
                'minimum_body_clearance_m': (
                    None
                    if not math.isfinite(self.minimum_clearance_m)
                    else self.minimum_clearance_m
                ),
                'maximum_rpm': self.maximum_rpm_seen,
                'rpm_saturation_samples': self.rpm_saturation_samples,
                'maximum_speed_mps': self.maximum_speed_mps,
                'maximum_applied_force_n': self.maximum_force_n,
                'planning_event_count': len(self.planning_times_ms),
                'planning_time_mean_ms': planning_mean,
                'planning_time_max_ms': planning_max,
                'nominal_sensor_position_rmse_m': sensor_metrics.get(
                    'nominal_position_rmse_m'
                ),
                'stress_sensor_position_rmse_m': sensor_metrics.get(
                    'stress_position_rmse_m'
                ),
            },
            'module_status': self.statuses,
            'read_only_recorder': True,
            'control_publishers_created': 0,
        }
        (self.run_dir / 'bonus_summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )
        self.generate_figures()
        self.publish_status()
        self.get_logger().warning(
            f'CQ01 BONUS RECORDER RESULT pass={summary["pass"]}; '
            f'failed={failed}; output={self.run_dir}'
        )

    def generate_figures(self) -> None:
        if not self.samples:
            return
        times = [float(row['time_s']) for row in self.samples]

        def numeric(field: str) -> List[float]:
            result: List[float] = []
            for row in self.samples:
                value = row[field]
                result.append(float('nan') if value == '' else float(value))
            return result

        figures = [
            ('position_error.png', 'Position error', 'Error (m)', numeric('position_error_m')),
            ('altitude.png', 'Altitude', 'Z (m)', numeric('z_m')),
            ('maximum_rpm.png', 'Maximum motor RPM', 'RPM', numeric('max_rpm')),
            ('body_clearance.png', 'Body clearance', 'Clearance (m)', numeric('clearance_m')),
        ]
        for filename, title, ylabel, values in figures:
            figure = plt.figure(figsize=(9, 4.8))
            axis = figure.add_subplot(111)
            axis.plot(times, values)
            axis.set_xlabel('Time (s)')
            axis.set_ylabel(ylabel)
            axis.set_title(title)
            axis.grid(True)
            figure.tight_layout()
            figure.savefig(self.run_dir / filename, dpi=160)
            plt.close(figure)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V9BonusRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finalized:
            node.finalize('interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
