#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


def rmse(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return math.sqrt(
        sum(value * value for value in values)
        / len(values)
    )


class PatternTrackingEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('cq01_pattern_tracking_evaluator')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'status_topic',
            '/cq01_pattern/status',
        )
        self.declare_parameter(
            'rpm_topic',
            '/drone/motor_rpm',
        )
        self.declare_parameter(
            'marker_topic',
            '/cq01_pattern/evaluator_markers',
        )
        self.declare_parameter(
            'results_root',
            '~/drone_task/your_drone_sim_ws/results/'
            'cq01_pattern_tracking',
        )
        self.declare_parameter(
            'maximum_duration_s',
            360.0,
        )
        self.declare_parameter(
            'sample_rate_hz',
            20.0,
        )
        self.declare_parameter(
            'maximum_rpm',
            10000.0,
        )
        self.declare_parameter(
            'rpm_saturation_ratio',
            0.98,
        )
        self.declare_parameter(
            'shutdown_after_result_s',
            1.0,
        )
        self.declare_parameter(
            'shutdown_on_complete',
            True,
        )
        self.declare_parameter(
            'result_topic',
            '/cq01_pattern/result',
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )
        self.maximum_duration_s = max(
            30.0,
            float(
                self.get_parameter(
                    'maximum_duration_s'
                ).value
            ),
        )
        self.maximum_rpm = max(
            1.0,
            float(self.get_parameter('maximum_rpm').value),
        )
        self.rpm_saturation_ratio = max(
            0.5,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'rpm_saturation_ratio'
                    ).value
                ),
            ),
        )
        self.shutdown_after_result_s = max(
            0.2,
            float(
                self.get_parameter(
                    'shutdown_after_result_s'
                ).value
            ),
        )
        self.shutdown_on_complete = bool(
            self.get_parameter(
                'shutdown_on_complete'
            ).value
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            100,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self.status_callback,
            transient_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('rpm_topic').value),
            self.rpm_callback,
            30,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self.result_pub = self.create_publisher(
            String,
            str(self.get_parameter('result_topic').value),
            transient_qos,
        )

        sample_rate = max(
            5.0,
            float(self.get_parameter('sample_rate_hz').value),
        )
        self.sample_timer = self.create_timer(
            1.0 / sample_rate,
            self.sample_callback,
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_status: Optional[Dict] = None
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]

        self.started = False
        self.start_time_s: Optional[float] = None
        self.completion_time_s: Optional[float] = None
        self.finalized = False

        self.last_phase = ''
        self.phase_start_position: Dict[str, Point3] = {}
        self.phase_end_position: Dict[str, Point3] = {}
        self.previous_phase_position: Dict[str, Point3] = {}
        self.phase_actual_path_length = {
            'circle': 0.0,
            'figure8': 0.0,
        }

        self.samples: List[Dict] = []
        self.phase_tracking_errors = {
            'circle': [],
            'figure8': [],
        }
        self.phase_geometric_errors = {
            'circle': [],
            'figure8': [],
        }
        self.phase_altitude_errors = {
            'circle': [],
            'figure8': [],
        }

        self.maximum_tilt_deg = 0.0
        self.maximum_rpm_observed = 0.0
        self.rpm_sample_count = 0
        self.rpm_saturation_count = 0

        self.center: Optional[Point3] = None
        self.pattern_start: Optional[Point3] = None
        self.circle_radius = 30.0
        self.circle_period = 80.0
        self.figure8_x_radius = 30.0
        self.figure8_y_radius = 18.0
        self.figure8_period = 100.0
        self.figure8_lookup: List[Point3] = []

        self.get_logger().info(
            'CQ-01 circle/figure-eight evaluator ready.'
        )

    def now_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds / 1.0e9
        )

    @staticmethod
    def position_from_odom(message: Odometry) -> Point3:
        p = message.pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    @staticmethod
    def tilt_deg(message: Odometry) -> float:
        q = message.pose.pose.orientation
        sinr_cosp = 2.0 * (
            q.w * q.x + q.y * q.z
        )
        cosr_cosp = 1.0 - 2.0 * (
            q.x * q.x + q.y * q.y
        )
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (
            q.w * q.y - q.z * q.x
        )
        pitch = (
            math.copysign(math.pi / 2.0, sinp)
            if abs(sinp) >= 1.0
            else math.asin(sinp)
        )
        return max(
            abs(math.degrees(roll)),
            abs(math.degrees(pitch)),
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        if self.started:
            self.maximum_tilt_deg = max(
                self.maximum_tilt_deg,
                self.tilt_deg(message),
            )

    def configure_geometry(self, payload: Dict) -> None:
        if self.center is not None:
            return
        center = payload.get('center')
        start = payload.get('pattern_start')
        if not center or not start:
            return
        self.center = tuple(float(v) for v in center)
        self.pattern_start = tuple(
            float(v) for v in start
        )
        self.circle_radius = float(
            payload.get(
                'circle_radius_m',
                self.circle_radius,
            )
        )
        self.circle_period = float(
            payload.get(
                'circle_period_s',
                self.circle_period,
            )
        )
        self.figure8_x_radius = float(
            payload.get(
                'figure8_x_radius_m',
                self.figure8_x_radius,
            )
        )
        self.figure8_y_radius = float(
            payload.get(
                'figure8_y_radius_m',
                self.figure8_y_radius,
            )
        )
        self.figure8_period = float(
            payload.get(
                'figure8_period_s',
                self.figure8_period,
            )
        )
        self.figure8_lookup = []
        for index in range(1440):
            theta = (
                2.0 * math.pi * index / 1439.0
            )
            self.figure8_lookup.append((
                self.center[0]
                + self.figure8_x_radius
                * math.cos(theta),
                self.center[1]
                + self.figure8_y_radius
                * math.sin(2.0 * theta),
                self.center[2],
            ))

    def status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        self.latest_status = payload
        self.configure_geometry(payload)

        state = str(payload.get('state', ''))
        phase = str(payload.get('phase', ''))

        if (
            not self.started
            and state == 'running'
        ):
            self.started = True
            self.start_time_s = self.now_s()
            self.get_logger().info(
                'Pattern tracking evaluation started.'
            )

        if phase != self.last_phase:
            if (
                self.last_phase
                in ('circle', 'figure8')
                and self.latest_odom is not None
            ):
                self.phase_end_position[
                    self.last_phase
                ] = self.position_from_odom(
                    self.latest_odom
                )
            if (
                phase in ('circle', 'figure8')
                and self.latest_odom is not None
            ):
                self.phase_start_position[
                    phase
                ] = self.position_from_odom(
                    self.latest_odom
                )
                self.previous_phase_position[
                    phase
                ] = self.phase_start_position[
                    phase
                ]
            self.last_phase = phase

        if (
            state == 'pattern_completed'
            and self.completion_time_s is None
        ):
            if (
                self.last_phase
                in ('circle', 'figure8')
                and self.latest_odom is not None
            ):
                self.phase_end_position[
                    self.last_phase
                ] = self.position_from_odom(
                    self.latest_odom
                )
            self.completion_time_s = self.now_s()
            self.get_logger().info(
                'Pattern completion received; '
                'preparing final result.'
            )

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        if len(message.data) < 4:
            return
        self.latest_rpm = [
            float(value) for value in message.data[:4]
        ]
        if not self.started:
            return
        threshold = (
            self.maximum_rpm
            * self.rpm_saturation_ratio
        )
        self.rpm_sample_count += 4
        self.rpm_saturation_count += sum(
            value >= threshold
            for value in self.latest_rpm
        )
        self.maximum_rpm_observed = max(
            self.maximum_rpm_observed,
            max(self.latest_rpm),
        )

    def circle_geometric_error(
        self,
        actual: Point3,
    ) -> float:
        assert self.center is not None
        radial_error = abs(
            math.hypot(
                actual[0] - self.center[0],
                actual[1] - self.center[1],
            )
            - self.circle_radius
        )
        altitude_error = (
            actual[2] - self.center[2]
        )
        return math.hypot(
            radial_error,
            altitude_error,
        )

    def figure8_geometric_error(
        self,
        actual: Point3,
    ) -> float:
        if not self.figure8_lookup:
            return float('nan')
        return math.sqrt(
            min(
                (
                    actual[0] - point[0]
                ) ** 2
                + (
                    actual[1] - point[1]
                ) ** 2
                + (
                    actual[2] - point[2]
                ) ** 2
                for point in self.figure8_lookup
            )
        )

    def sample_callback(self) -> None:
        now = self.now_s()

        if (
            self.started
            and self.start_time_s is not None
            and not self.finalized
            and (
                now - self.start_time_s
                > self.maximum_duration_s
            )
        ):
            self.save_result('timeout')
            return

        if (
            self.completion_time_s is not None
            and not self.finalized
            and (
                now - self.completion_time_s
                >= self.shutdown_after_result_s
            )
        ):
            self.save_result('pattern_completed')
            return

        if (
            not self.started
            or self.latest_odom is None
            or self.latest_status is None
            or self.center is None
        ):
            return

        phase = str(
            self.latest_status.get('phase', '')
        )
        if phase not in ('circle', 'figure8'):
            self.publish_markers()
            return

        reference_raw = self.latest_status.get(
            'reference'
        )
        if not reference_raw:
            return

        reference = tuple(
            float(value) for value in reference_raw
        )
        actual = self.position_from_odom(
            self.latest_odom
        )
        tracking_error = math.dist(
            actual,
            reference,
        )
        geometric_error = (
            self.circle_geometric_error(actual)
            if phase == 'circle'
            else self.figure8_geometric_error(actual)
        )
        altitude_error = abs(
            actual[2] - self.center[2]
        )

        previous = self.previous_phase_position.get(
            phase
        )
        if previous is not None:
            step = math.dist(previous, actual)
            if step < 2.0:
                self.phase_actual_path_length[
                    phase
                ] += step
        self.previous_phase_position[phase] = actual

        self.phase_tracking_errors[phase].append(
            tracking_error
        )
        self.phase_geometric_errors[phase].append(
            geometric_error
        )
        self.phase_altitude_errors[phase].append(
            altitude_error
        )

        elapsed = (
            now - self.start_time_s
            if self.start_time_s is not None
            else 0.0
        )
        self.samples.append({
            'time_s': elapsed,
            'phase': phase,
            'phase_progress': float(
                self.latest_status.get(
                    'phase_progress',
                    0.0,
                )
            ),
            'actual_x_m': actual[0],
            'actual_y_m': actual[1],
            'actual_z_m': actual[2],
            'reference_x_m': reference[0],
            'reference_y_m': reference[1],
            'reference_z_m': reference[2],
            'reference_tracking_error_m':
                tracking_error,
            'geometric_cross_track_error_m':
                geometric_error,
            'altitude_error_m': altitude_error,
            'tilt_deg': self.tilt_deg(
                self.latest_odom
            ),
            'maximum_rpm': max(self.latest_rpm),
        })
        self.publish_markers()

    def figure8_reference_length(self) -> float:
        if not self.figure8_lookup:
            return 0.0
        return sum(
            math.dist(first, second)
            for first, second in zip(
                self.figure8_lookup[:-1],
                self.figure8_lookup[1:],
            )
        )

    def closure_error(
        self,
        phase: str,
    ) -> Optional[float]:
        start = self.phase_start_position.get(phase)
        end = self.phase_end_position.get(phase)
        if start is None or end is None:
            return None
        return math.dist(start, end)

    def phase_metrics(
        self,
        phase: str,
        reference_length: float,
    ) -> Dict:
        tracking = self.phase_tracking_errors[phase]
        geometric = self.phase_geometric_errors[phase]
        altitude = self.phase_altitude_errors[phase]
        actual_length = self.phase_actual_path_length[
            phase
        ]
        return {
            'sample_count': len(tracking),
            'reference_tracking_rmse_m': rmse(tracking),
            'reference_tracking_max_m':
                max(tracking) if tracking else None,
            'geometric_cross_track_rmse_m':
                rmse(geometric),
            'geometric_cross_track_max_m':
                max(geometric) if geometric else None,
            'altitude_rmse_m': rmse(altitude),
            'altitude_max_error_m':
                max(altitude) if altitude else None,
            'closure_error_m':
                self.closure_error(phase),
            'reference_path_length_m':
                reference_length,
            'actual_path_length_m':
                actual_length,
            'actual_to_reference_length_ratio':
                (
                    actual_length / reference_length
                    if reference_length > 1.0e-9
                    else None
                ),
        }

    def build_summary(
        self,
        final_state: str,
    ) -> Dict:
        circle = self.phase_metrics(
            'circle',
            2.0 * math.pi * self.circle_radius,
        )
        figure8 = self.phase_metrics(
            'figure8',
            self.figure8_reference_length(),
        )

        current_position = (
            self.position_from_odom(self.latest_odom)
            if self.latest_odom is not None
            else None
        )
        final_error = (
            math.dist(
                current_position,
                self.pattern_start,
            )
            if (
                current_position is not None
                and self.pattern_start is not None
            )
            else None
        )
        elapsed = (
            self.now_s() - self.start_time_s
            if self.start_time_s is not None
            else 0.0
        )
        rpm_fraction = (
            self.rpm_saturation_count
            / self.rpm_sample_count
            if self.rpm_sample_count > 0
            else 0.0
        )

        thresholds = {
            'circle_cross_track_rmse_m': 2.5,
            'figure8_cross_track_rmse_m': 3.0,
            'cross_track_max_m': 6.0,
            'closure_error_m': 2.5,
            'altitude_rmse_m': 1.5,
            'final_error_m': 0.30,
            'maximum_tilt_deg': 45.0,
            'rpm_saturation_fraction': 0.001,
            'minimum_phase_samples': 250,
        }

        def value_or_inf(value):
            return (
                float('inf')
                if value is None
                else float(value)
            )

        criteria = {
            'pattern_completed':
                final_state == 'pattern_completed',
            'circle_sample_count_pass':
                circle['sample_count']
                >= thresholds['minimum_phase_samples'],
            'figure8_sample_count_pass':
                figure8['sample_count']
                >= thresholds['minimum_phase_samples'],
            'circle_cross_track_rmse_pass':
                value_or_inf(
                    circle[
                        'geometric_cross_track_rmse_m'
                    ]
                )
                <= thresholds[
                    'circle_cross_track_rmse_m'
                ],
            'figure8_cross_track_rmse_pass':
                value_or_inf(
                    figure8[
                        'geometric_cross_track_rmse_m'
                    ]
                )
                <= thresholds[
                    'figure8_cross_track_rmse_m'
                ],
            'circle_cross_track_max_pass':
                value_or_inf(
                    circle[
                        'geometric_cross_track_max_m'
                    ]
                )
                <= thresholds['cross_track_max_m'],
            'figure8_cross_track_max_pass':
                value_or_inf(
                    figure8[
                        'geometric_cross_track_max_m'
                    ]
                )
                <= thresholds['cross_track_max_m'],
            'circle_closure_pass':
                value_or_inf(
                    circle['closure_error_m']
                )
                <= thresholds['closure_error_m'],
            'figure8_closure_pass':
                value_or_inf(
                    figure8['closure_error_m']
                )
                <= thresholds['closure_error_m'],
            'circle_altitude_pass':
                value_or_inf(
                    circle['altitude_rmse_m']
                )
                <= thresholds['altitude_rmse_m'],
            'figure8_altitude_pass':
                value_or_inf(
                    figure8['altitude_rmse_m']
                )
                <= thresholds['altitude_rmse_m'],
            'final_error_pass':
                value_or_inf(final_error)
                <= thresholds['final_error_m'],
            'tilt_pass':
                self.maximum_tilt_deg
                <= thresholds['maximum_tilt_deg'],
            'rpm_saturation_pass':
                rpm_fraction
                <= thresholds[
                    'rpm_saturation_fraction'
                ],
        }

        return {
            'experiment':
                'cq01_mountain_city_circle_figure8_tracking',
            'final_state': final_state,
            'geometry': {
                'center': list(self.center)
                if self.center is not None
                else None,
                'pattern_start': list(self.pattern_start)
                if self.pattern_start is not None
                else None,
                'circle_radius_m': self.circle_radius,
                'circle_period_s': self.circle_period,
                'figure8_x_radius_m':
                    self.figure8_x_radius,
                'figure8_y_radius_m':
                    self.figure8_y_radius,
                'figure8_period_s':
                    self.figure8_period,
                'designed_pattern_zone_clearance_m':
                    44.1,
                'designed_transit_clearance_m':
                    55.3,
            },
            'thresholds': thresholds,
            'phase_metrics': {
                'circle': circle,
                'figure8': figure8,
            },
            'overall_metrics': {
                'mission_time_s': elapsed,
                'final_hold_error_m': final_error,
                'maximum_tilt_deg':
                    self.maximum_tilt_deg,
                'maximum_rpm':
                    self.maximum_rpm_observed,
                'rpm_saturation_fraction':
                    rpm_fraction,
                'sample_count': len(self.samples),
            },
            'criteria': criteria,
            'pass': all(criteria.values()),
        }

    def output_directory(self) -> Path:
        timestamp = (
            self.get_clock().now()
            .to_msg()
        )
        seconds = int(timestamp.sec)
        import datetime
        label = datetime.datetime.fromtimestamp(
            seconds
        ).strftime('%Y%m%d_%H%M%S')
        directory = (
            self.results_root
            / f'cq01_circle_figure8_{label}'
        )
        directory.mkdir(
            parents=True,
            exist_ok=True,
        )
        return directory

    def save_samples(self, directory: Path) -> None:
        if not self.samples:
            return
        path = directory / 'samples.csv'
        with path.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=list(
                    self.samples[0].keys()
                ),
            )
            writer.writeheader()
            writer.writerows(self.samples)

    def save_plots(self, directory: Path) -> None:
        if not self.samples or self.center is None:
            return

        circle_ref_x = []
        circle_ref_y = []
        for index in range(361):
            theta = 2.0 * math.pi * index / 360.0
            circle_ref_x.append(
                self.center[0]
                + self.circle_radius * math.cos(theta)
            )
            circle_ref_y.append(
                self.center[1]
                + self.circle_radius * math.sin(theta)
            )

        figure_ref_x = [
            point[0] for point in self.figure8_lookup
        ]
        figure_ref_y = [
            point[1] for point in self.figure8_lookup
        ]

        plt.figure(figsize=(8, 7))
        plt.plot(
            circle_ref_x,
            circle_ref_y,
            linestyle='--',
            label='Circle reference',
        )
        plt.plot(
            figure_ref_x,
            figure_ref_y,
            linestyle='--',
            label='Figure-eight reference',
        )
        for phase in ('circle', 'figure8'):
            selected = [
                sample for sample in self.samples
                if sample['phase'] == phase
            ]
            plt.plot(
                [
                    sample['actual_x_m']
                    for sample in selected
                ],
                [
                    sample['actual_y_m']
                    for sample in selected
                ],
                label=f'{phase} actual',
            )
        plt.axis('equal')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            directory / 'top_down_tracking.png',
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(9, 4.8))
        for phase in ('circle', 'figure8'):
            selected = [
                sample for sample in self.samples
                if sample['phase'] == phase
            ]
            plt.plot(
                [
                    sample['time_s']
                    for sample in selected
                ],
                [
                    sample[
                        'geometric_cross_track_error_m'
                    ]
                    for sample in selected
                ],
                label=phase,
            )
        plt.xlabel('Mission time (s)')
        plt.ylabel('Geometric cross-track error (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            directory / 'cross_track_error.png',
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(9, 4.8))
        plt.plot(
            [
                sample['time_s']
                for sample in self.samples
            ],
            [
                sample['actual_z_m']
                for sample in self.samples
            ],
            label='Actual altitude',
        )
        plt.plot(
            [
                sample['time_s']
                for sample in self.samples
            ],
            [
                sample['reference_z_m']
                for sample in self.samples
            ],
            linestyle='--',
            label='Reference altitude',
        )
        plt.xlabel('Mission time (s)')
        plt.ylabel('Z (m)')
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            directory / 'altitude_tracking.png',
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(9, 4.8))
        plt.plot(
            [
                sample['time_s']
                for sample in self.samples
            ],
            [
                sample['tilt_deg']
                for sample in self.samples
            ],
        )
        plt.axhline(45.0, linestyle='--')
        plt.xlabel('Mission time (s)')
        plt.ylabel('Tilt (deg)')
        plt.tight_layout()
        plt.savefig(
            directory / 'tilt_history.png',
            dpi=180,
        )
        plt.close()

    def save_result(self, final_state: str) -> None:
        if self.finalized:
            return
        self.finalized = True

        if (
            self.last_phase
            in ('circle', 'figure8')
            and self.latest_odom is not None
            and self.last_phase
            not in self.phase_end_position
        ):
            self.phase_end_position[
                self.last_phase
            ] = self.position_from_odom(
                self.latest_odom
            )

        summary = self.build_summary(final_state)
        directory = self.output_directory()
        with (
            directory / 'summary.json'
        ).open('w', encoding='utf-8') as file:
            json.dump(
                summary,
                file,
                ensure_ascii=False,
                indent=2,
            )
        self.save_samples(directory)
        self.save_plots(directory)

        self.get_logger().info(
            'CQ-01 pattern result: '
            f'pass={summary["pass"]}, '
            f'circle_rmse='
            f'{summary["phase_metrics"]["circle"]["geometric_cross_track_rmse_m"]}, '
            f'figure8_rmse='
            f'{summary["phase_metrics"]["figure8"]["geometric_cross_track_rmse_m"]}, '
            f'final_error='
            f'{summary["overall_metrics"]["final_hold_error_m"]}'
        )
        result_message = String()
        result_message.data = json.dumps(
            summary,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.result_pub.publish(result_message)

        self.get_logger().info(
            f'Results saved to: {directory}'
        )

        if self.shutdown_on_complete:
            self.create_timer(
                0.5,
                self.force_exit,
            )
        else:
            self.get_logger().info(
                'Pattern result published; evaluator remains '
                'alive for the integrated P3-P15 mission.'
            )

    def force_exit(self) -> None:
        self.get_logger().info(
            'Pattern evaluation complete; '
            'exiting evaluator for launch shutdown.'
        )
        os._exit(0)

    @staticmethod
    def rgba(
        red: float,
        green: float,
        blue: float,
        alpha: float,
    ) -> ColorRGBA:
        result = ColorRGBA()
        result.r = red
        result.g = green
        result.b = blue
        result.a = alpha
        return result

    @staticmethod
    def point(point: Point3) -> Point:
        result = Point()
        result.x = point[0]
        result.y = point[1]
        result.z = point[2]
        return result

    def publish_markers(self) -> None:
        if (
            self.latest_odom is None
            or self.latest_status is None
        ):
            return
        reference_raw = self.latest_status.get(
            'reference'
        )
        if not reference_raw:
            return
        reference = tuple(
            float(value) for value in reference_raw
        )
        actual = self.position_from_odom(
            self.latest_odom
        )
        error = math.dist(actual, reference)
        phase = str(
            self.latest_status.get('phase', '')
        )

        array = MarkerArray()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = (
            self.get_clock().now().to_msg()
        )
        line.ns = 'tracking_error_vector'
        line.id = 1
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.35
        line.color = self.rgba(
            1.0, 0.20, 0.10, 0.95
        )
        line.points = [
            self.point(actual),
            self.point(reference),
        ]
        array.markers.append(line)

        text = Marker()
        text.header = line.header
        text.ns = 'tracking_metrics'
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = actual[0]
        text.pose.position.y = actual[1]
        text.pose.position.z = actual[2] + 5.0
        text.pose.orientation.w = 1.0
        text.scale.z = 2.5
        text.color = self.rgba(
            1.0, 1.0, 1.0, 1.0
        )
        text.text = (
            f'{phase.upper()}\n'
            f'ERR {error:.2f} m\n'
            f'TILT {self.maximum_tilt_deg:.1f} deg'
        )
        array.markers.append(text)

        self.marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatternTrackingEvaluator()
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
