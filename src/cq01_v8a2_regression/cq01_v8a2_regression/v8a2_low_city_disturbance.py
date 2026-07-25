#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_roll_pitch(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


class CQ01IntegratedDisturbanceManager(Node):
    """Inject and evaluate a real external wrench during the CQ-01 mission.

    The node never publishes navigation goals. It only:
      1. listens to the active CQ-01 mission and A* path,
      2. injects a configurable map-frame pulse wrench,
      3. publishes an RViz arrow/text indicator,
      4. evaluates cross-track recovery and mission continuation,
      5. saves JSON/CSV/plots for the integrated disturbance run.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v34h_integrated_disturbance_manager')

        self.declare_parameter('trigger_mode', 'waypoint')
        self.declare_parameter('trigger_after_reached_index', 3)
        self.declare_parameter('trigger_delay_s', 1.0)
        self.declare_parameter('duration_s', 1.0)
        self.declare_parameter('ramp_time_s', 0.15)
        self.declare_parameter('force_x_n', 0.0)
        self.declare_parameter('force_y_n', 2.5)
        self.declare_parameter('force_z_n', 0.0)
        self.declare_parameter('torque_x_nm', 0.0)
        self.declare_parameter('torque_y_nm', 0.0)
        self.declare_parameter('torque_z_nm', 0.0)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('recovery_tolerance_m', 0.60)
        self.declare_parameter('recovery_hold_s', 1.0)
        self.declare_parameter('maximum_tilt_deg', 45.0)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter('saturation_ratio_threshold', 0.98)
        self.declare_parameter('maximum_saturation_fraction', 0.05)
        self.declare_parameter('mission_status_topic', '/cq01_v34h/mission_status')
        self.declare_parameter('disturbance_status_topic', '/cq01_v34h/disturbance_status')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('path_topic', '/drone/planned_path_3d')
        self.declare_parameter('rpm_topic', '/drone/motor_rpm')
        self.declare_parameter('command_wrench_topic', '/drone/external_wrench')
        self.declare_parameter(
            'applied_wrench_topic',
            '/drone/external_wrench_applied',
        )
        self.declare_parameter('active_topic', '/drone/disturbance_active')
        self.declare_parameter(
            'marker_topic',
            '/cq01_v34h/disturbance_markers',
        )
        # Visualization gain only. It does not change the physical force.
        self.declare_parameter('visual_force_scale_m_per_n', 8.0)
        self.declare_parameter('visual_afterglow_s', 4.0)
        self.declare_parameter('visual_recovery_hold_s', 20.0)
        self.declare_parameter('trail_sample_period_s', 0.10)
        self.declare_parameter('status_heartbeat_s', 1.0)
        self.declare_parameter('manual_countdown_s', 3.0)
        self.declare_parameter('marker_publish_rate_hz', 8.0)
        self.declare_parameter('visual_evidence_hold_s', 12.0)
        self.declare_parameter('allow_retrigger', True)
        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_integrated_disturbance'
            ),
        )

        self.trigger_mode = str(
            self.get_parameter('trigger_mode').value
        ).strip().lower()
        if self.trigger_mode not in ('manual', 'waypoint'):
            raise ValueError(
                "trigger_mode must be 'manual' or 'waypoint'"
            )

        self.trigger_after_reached_index = int(
            self.get_parameter('trigger_after_reached_index').value
        )
        self.trigger_delay_s = max(
            0.0,
            float(self.get_parameter('trigger_delay_s').value),
        )
        self.duration_s = max(
            0.05,
            float(self.get_parameter('duration_s').value),
        )
        self.ramp_time_s = max(
            0.0,
            min(
                float(self.get_parameter('ramp_time_s').value),
                self.duration_s / 2.0,
            ),
        )
        self.force = [
            float(self.get_parameter('force_x_n').value),
            float(self.get_parameter('force_y_n').value),
            float(self.get_parameter('force_z_n').value),
        ]
        self.torque = [
            float(self.get_parameter('torque_x_nm').value),
            float(self.get_parameter('torque_y_nm').value),
            float(self.get_parameter('torque_z_nm').value),
        ]
        self.recovery_tolerance_m = float(
            self.get_parameter('recovery_tolerance_m').value
        )
        self.recovery_hold_s = float(
            self.get_parameter('recovery_hold_s').value
        )
        self.maximum_tilt_deg = float(
            self.get_parameter('maximum_tilt_deg').value
        )
        self.maximum_rpm = float(
            self.get_parameter('maximum_rpm').value
        )
        self.saturation_ratio_threshold = float(
            self.get_parameter('saturation_ratio_threshold').value
        )
        self.maximum_saturation_fraction = float(
            self.get_parameter('maximum_saturation_fraction').value
        )
        self.visual_force_scale_m_per_n = max(
            1.0,
            float(
                self.get_parameter(
                    'visual_force_scale_m_per_n'
                ).value
            ),
        )
        self.visual_afterglow_s = max(
            0.0,
            float(self.get_parameter('visual_afterglow_s').value),
        )
        self.visual_recovery_hold_s = max(
            0.0,
            float(
                self.get_parameter(
                    'visual_recovery_hold_s'
                ).value
            ),
        )
        self.trail_sample_period_s = max(
            0.02,
            float(
                self.get_parameter(
                    'trail_sample_period_s'
                ).value
            ),
        )
        self.status_heartbeat_s = max(
            0.2,
            float(
                self.get_parameter(
                    'status_heartbeat_s'
                ).value
            ),
        )
        self.manual_countdown_s = max(
            0.0,
            float(self.get_parameter('manual_countdown_s').value),
        )
        marker_rate_hz = max(
            1.0,
            float(
                self.get_parameter(
                    'marker_publish_rate_hz'
                ).value
            ),
        )
        self.marker_period_s = 1.0 / marker_rate_hz
        self.visual_evidence_hold_s = max(
            2.0,
            float(
                self.get_parameter(
                    'visual_evidence_hold_s'
                ).value
            ),
        )
        self.allow_retrigger = bool(
            self.get_parameter('allow_retrigger').value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )

        rate_hz = max(
            10.0,
            float(self.get_parameter('publish_rate_hz').value),
        )
        self.period_s = 1.0 / rate_hz

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter('command_wrench_topic').value),
            20,
        )
        self.active_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('active_topic').value),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('disturbance_status_topic').value),
            transient_qos,
        )
        self.focus_tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            String,
            str(self.get_parameter('mission_status_topic').value),
            self.mission_status_callback,
            transient_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            50,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('path_topic').value),
            self.path_callback,
            transient_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('rpm_topic').value),
            self.rpm_callback,
            20,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter('applied_wrench_topic').value),
            self.applied_wrench_callback,
            20,
        )

        self.trigger_service = self.create_service(
            Trigger,
            '~/trigger',
            self.trigger_callback,
        )
        self.finalize_service = self.create_service(
            Trigger,
            '~/finalize',
            self.finalize_callback,
        )
        self.reset_service = self.create_service(
            Trigger,
            '~/reset',
            self.reset_callback,
        )

        self.latest_position: Optional[Tuple[float, float, float]] = None
        self.latest_roll_deg = 0.0
        self.latest_pitch_deg = 0.0
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.latest_applied_force = [0.0, 0.0, 0.0]
        self.path_points: List[Tuple[float, float, float]] = []
        self.recent_cross_track: Deque[Tuple[float, float]] = deque(maxlen=500)

        self.triggered_once = False
        self.scheduled_trigger_time: Optional[float] = None
        self.disturbance_start_time: Optional[float] = None
        self.disturbance_end_time: Optional[float] = None
        self.experiment_start_time: Optional[float] = None
        self.baseline_cross_track_m: Optional[float] = None
        self.recovery_time_s: Optional[float] = None
        self.recovery_candidate_start: Optional[float] = None
        self.mission_completed_after_trigger = False
        self.mission_final_state: Optional[str] = None
        self.samples: List[Dict[str, float]] = []
        self.saved_output_dir: Optional[Path] = None
        self.latest_mission_payload: Dict = {}

        # Presentation state. None of these values alter the dynamics.
        self.last_status_state: Optional[str] = None
        self.last_status_publish_time = -1.0e9
        self.last_marker_publish_time = -1.0e9
        self.visual_phase = 'idle'
        self.trigger_position: Optional[
            Tuple[float, float, float]
        ] = None
        self.focus_position: Optional[
            Tuple[float, float, float]
        ] = None
        self.disturbance_trail: List[
            Tuple[float, float, float]
        ] = []
        self.last_trail_sample_time = -1.0e9
        self.peak_cross_track_m = 0.0
        self.peak_position: Optional[
            Tuple[float, float, float]
        ] = None
        self.peak_reference_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.timer = self.create_timer(self.period_s, self.timer_callback)
        self.publish_status('ready')
        self.get_logger().info(
            'CQ-01 integrated disturbance manager ready: '
            f'mode={self.trigger_mode}, '
            f'force={self.force} N, duration={self.duration_s:.2f}s. '
            'Manual service: ~/trigger'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def publish_status(
        self,
        state: str,
        force: bool = False,
        **extra,
    ) -> None:
        now = self.now_s()
        same_state = state == self.last_status_state
        heartbeat_due = (
            now - self.last_status_publish_time
            >= self.status_heartbeat_s
        )
        if same_state and not force and not heartbeat_due:
            return

        payload = {
            'state': state,
            'trigger_mode': self.trigger_mode,
            'triggered_once': self.triggered_once,
            'scheduled': self.scheduled_trigger_time is not None,
            'active': self.is_disturbance_active(now),
            'visual_phase': self.visual_phase,
            'baseline_cross_track_m':
                self.baseline_cross_track_m,
            'peak_cross_track_m': self.peak_cross_track_m,
            'recovery_time_s': self.recovery_time_s,
            'mission_completed_after_trigger':
                self.mission_completed_after_trigger,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)
        self.last_status_state = state
        self.last_status_publish_time = now

    def mission_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest_mission_payload = payload
        state = str(payload.get('state', ''))

        if (
            self.trigger_mode == 'waypoint'
            and not self.triggered_once
            and self.scheduled_trigger_time is None
            and state == 'advancing'
            and int(payload.get('reached_index', -999))
            == self.trigger_after_reached_index
        ):
            self.scheduled_trigger_time = (
                self.now_s() + self.trigger_delay_s
            )
            self.visual_phase = 'scheduled'
            self.publish_status(
                'trigger_scheduled',
                reached_index=self.trigger_after_reached_index,
                delay_s=self.trigger_delay_s,
            )
            self.get_logger().info(
                'CQ-01 disturbance scheduled after waypoint '
                f'{self.trigger_after_reached_index}, '
                f'delay={self.trigger_delay_s:.2f}s'
            )

        if state in (
            'mission_completed',
            'stalled',
            'planning_failed',
        ):
            self.mission_final_state = state
            if self.triggered_once:
                self.mission_completed_after_trigger = (
                    state == 'mission_completed'
                )
                self.visual_phase = (
                    'completed'
                    if state == 'mission_completed'
                    else 'failed'
                )
                self.save_results(final_state=state)
                self.publish_status(
                    'results_saved',
                    mission_final_state=state,
                    output_dir=str(self.saved_output_dir)
                    if self.saved_output_dir else None,
                )

    def path_callback(self, message: NavPath) -> None:
        self.path_points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

    def rpm_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 4:
            self.latest_rpm = [
                float(value) for value in message.data[:4]
            ]

    def applied_wrench_callback(
        self,
        message: WrenchStamped,
    ) -> None:
        self.latest_applied_force = [
            float(message.wrench.force.x),
            float(message.wrench.force.y),
            float(message.wrench.force.z),
        ]

    def nearest_path_point(
        self,
        position: Tuple[float, float, float],
    ) -> Tuple[Optional[float], Optional[Tuple[float, float, float]]]:
        if not self.path_points:
            return None, None
        nearest = min(
            self.path_points,
            key=lambda point: math.dist(position, point),
        )
        return math.dist(position, nearest), nearest

    def cross_track_error(
        self,
        position: Tuple[float, float, float],
    ) -> Optional[float]:
        distance, _nearest = self.nearest_path_point(position)
        return distance

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.latest_position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        if (
            not self.triggered_once
            and self.scheduled_trigger_time is None
        ):
            self.focus_position = self.latest_position
        q = message.pose.pose.orientation
        roll, pitch = quaternion_to_roll_pitch(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        self.latest_roll_deg = math.degrees(roll)
        self.latest_pitch_deg = math.degrees(pitch)

        cross_track, nearest_reference = self.nearest_path_point(
            self.latest_position
        )
        now = self.now_s()
        if cross_track is not None:
            self.recent_cross_track.append((now, cross_track))

        if not self.triggered_once:
            return
        if self.experiment_start_time is None:
            return

        # V8-A2 low-city isolation: once this first disturbance has
        # recovered, retain mission-status monitoring but stop adding
        # samples. The later P3-P4 disturbance therefore cannot alter
        # the low-city impulse, tilt, RPM, or cross-track metrics.
        if self.recovery_time_s is not None:
            return

        elapsed = now - self.experiment_start_time

        if (
            now - self.last_trail_sample_time
            >= self.trail_sample_period_s
        ):
            self.disturbance_trail.append(self.latest_position)
            self.last_trail_sample_time = now

        if (
            cross_track is not None
            and cross_track >= self.peak_cross_track_m
        ):
            self.peak_cross_track_m = cross_track
            self.peak_position = self.latest_position
            self.peak_reference_position = nearest_reference

        force_norm = math.sqrt(
            sum(value * value for value in self.latest_applied_force)
        )
        rpm_max = max(self.latest_rpm) if self.latest_rpm else 0.0
        speed = math.sqrt(
            message.twist.twist.linear.x ** 2
            + message.twist.twist.linear.y ** 2
            + message.twist.twist.linear.z ** 2
        )
        self.samples.append({
            'time_s': elapsed,
            'x_m': self.latest_position[0],
            'y_m': self.latest_position[1],
            'z_m': self.latest_position[2],
            'speed_mps': float(speed),
            'cross_track_error_m':
                float(cross_track)
                if cross_track is not None else float('nan'),
            'roll_deg': self.latest_roll_deg,
            'pitch_deg': self.latest_pitch_deg,
            'tilt_deg': max(
                abs(self.latest_roll_deg),
                abs(self.latest_pitch_deg),
            ),
            'force_x_n': self.latest_applied_force[0],
            'force_y_n': self.latest_applied_force[1],
            'force_z_n': self.latest_applied_force[2],
            'force_norm_n': force_norm,
            'rpm_1': self.latest_rpm[0],
            'rpm_2': self.latest_rpm[1],
            'rpm_3': self.latest_rpm[2],
            'rpm_4': self.latest_rpm[3],
            'rpm_max': rpm_max,
        })

        if (
            self.disturbance_end_time is not None
            and now >= self.disturbance_end_time
            and self.recovery_time_s is None
            and cross_track is not None
        ):
            threshold = max(
                self.recovery_tolerance_m,
                (self.baseline_cross_track_m or 0.0)
                + self.recovery_tolerance_m,
            )
            if cross_track <= threshold:
                if self.recovery_candidate_start is None:
                    self.recovery_candidate_start = now
                elif (
                    now - self.recovery_candidate_start
                    >= self.recovery_hold_s
                ):
                    self.recovery_time_s = (
                        self.recovery_candidate_start
                        - self.disturbance_end_time
                    )
                    self.visual_phase = 'recovered'
                    self.publish_status(
                        'recovered',
                        recovery_time_s=self.recovery_time_s,
                        threshold_m=threshold,
                    )
                    self.get_logger().info(
                        'CQ-01 disturbance recovery detected: '
                        f'{self.recovery_time_s:.3f}s'
                    )
            else:
                self.recovery_candidate_start = None

    def trigger_callback(self, _request, response):
        now = self.now_s()

        if (
            self.scheduled_trigger_time is not None
            or self.is_disturbance_active(now)
        ):
            response.success = False
            response.message = (
                'Disturbance is already scheduled or active'
            )
            return response

        if self.triggered_once:
            if not self.allow_retrigger:
                response.success = False
                response.message = (
                    'Disturbance already triggered; call ~/reset first'
                )
                return response
            self.clear_experiment_state(
                publish_status=False,
                delete_markers=True,
            )

        self.focus_position = self.latest_position
        self.scheduled_trigger_time = (
            now + self.manual_countdown_s
        )
        self.visual_phase = 'scheduled'
        response.success = True
        response.message = (
            'CQ-01 disturbance accepted; physical force starts in '
            f'{self.manual_countdown_s:.1f} s'
        )
        self.publish_status(
            'manual_trigger_requested',
            force=True,
            countdown_s=self.manual_countdown_s,
        )
        return response

    def finalize_callback(self, _request, response):
        if not self.triggered_once:
            response.success = False
            response.message = 'No disturbance has been triggered'
            return response
        self.save_results(final_state='manual_finalize')
        response.success = True
        response.message = (
            f'Results saved to {self.saved_output_dir}'
        )
        return response

    def reset_callback(self, _request, response):
        self.clear_experiment_state(
            publish_status=True,
            delete_markers=True,
        )
        response.success = True
        response.message = 'Integrated disturbance state reset'
        return response

    def publish_delete_all_markers(self) -> None:
        array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        array.markers.append(marker)
        self.marker_pub.publish(array)

    def clear_experiment_state(
        self,
        publish_status: bool = True,
        delete_markers: bool = False,
    ) -> None:
        self.triggered_once = False
        self.scheduled_trigger_time = None
        self.disturbance_start_time = None
        self.disturbance_end_time = None
        self.experiment_start_time = None
        self.baseline_cross_track_m = None
        self.recovery_time_s = None
        self.recovery_candidate_start = None
        self.mission_completed_after_trigger = False
        self.mission_final_state = None
        self.samples = []
        self.saved_output_dir = None
        self.visual_phase = 'idle'
        self.trigger_position = None
        self.focus_position = self.latest_position
        self.disturbance_trail = []
        self.last_trail_sample_time = -1.0e9
        self.peak_cross_track_m = 0.0
        self.peak_position = None
        self.peak_reference_position = None
        self.last_status_state = None
        self.last_status_publish_time = -1.0e9
        self.last_marker_publish_time = -1.0e9

        # Publish only the physical zero command here. Marker generation
        # is deliberately excluded so the reset service returns promptly.
        self.publish_zero_wrench()
        if delete_markers:
            self.publish_delete_all_markers()
        if publish_status:
            self.publish_status('reset', force=True)

    def compute_baseline(self, now: float) -> Optional[float]:
        values = [
            value
            for timestamp, value in self.recent_cross_track
            if now - 1.0 <= timestamp <= now
        ]
        if not values:
            return None
        return sum(values) / len(values)

    def start_disturbance(self, now: float) -> None:
        if self.latest_position is None:
            self.scheduled_trigger_time = now + 0.2
            return
        self.triggered_once = True
        self.scheduled_trigger_time = None
        self.experiment_start_time = now
        self.visual_phase = 'active'
        self.trigger_position = self.latest_position
        self.disturbance_trail = [self.latest_position]
        self.last_trail_sample_time = now
        self.peak_cross_track_m = 0.0
        self.peak_position = None
        self.peak_reference_position = None
        self.disturbance_start_time = now
        self.disturbance_end_time = now + self.duration_s
        self.baseline_cross_track_m = self.compute_baseline(now)
        self.samples = []
        self.publish_status(
            'disturbance_started',
            force_n=self.force,
            duration_s=self.duration_s,
        )
        self.get_logger().warning(
            'CQ-01 CROSSWIND ACTIVE: '
            f'force={self.force} N, duration={self.duration_s:.2f}s'
        )

    def envelope(self, elapsed: float) -> float:
        if elapsed < 0.0 or elapsed > self.duration_s:
            return 0.0
        if self.ramp_time_s <= 0.0:
            return 1.0
        if elapsed < self.ramp_time_s:
            return elapsed / self.ramp_time_s
        remaining = self.duration_s - elapsed
        if remaining < self.ramp_time_s:
            return remaining / self.ramp_time_s
        return 1.0

    def is_disturbance_active(self, now: float) -> bool:
        return (
            self.disturbance_start_time is not None
            and self.disturbance_end_time is not None
            and self.disturbance_start_time
            <= now <= self.disturbance_end_time
        )

    def publish_wrench(
        self,
        force: List[float],
        torque: List[float],
        active: bool,
    ) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x = force[0]
        message.wrench.force.y = force[1]
        message.wrench.force.z = force[2]
        message.wrench.torque.x = torque[0]
        message.wrench.torque.y = torque[1]
        message.wrench.torque.z = torque[2]
        self.wrench_pub.publish(message)

        active_message = Bool()
        active_message.data = active
        self.active_pub.publish(active_message)

    def publish_zero_wrench(self) -> None:
        self.publish_wrench(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            False,
        )

    def publish_focus_transform(self) -> None:
        position = self.focus_position or self.latest_position
        if position is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'cq01_disturbance_focus'
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.w = 1.0
        self.focus_tf_broadcaster.sendTransform(transform)

    def marker_update_due(self, now: float) -> bool:
        if (
            now - self.last_marker_publish_time
            < self.marker_period_s
        ):
            return False
        self.last_marker_publish_time = now
        return True

    def make_marker_header(self):
        header = WrenchStamped().header
        header.frame_id = 'world'
        header.stamp = self.get_clock().now().to_msg()
        return header

    def append_text_marker(
        self,
        array: MarkerArray,
        marker_id: int,
        text_value: str,
        position: Tuple[float, float, float],
        scale_z: float,
        color: Tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'cq01_disturbance_evidence'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale_z
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text_value
        array.markers.append(marker)

    def publish_markers(
        self,
        force: List[float],
        phase: str,
    ) -> None:
        if self.latest_position is None:
            return

        array = MarkerArray()

        # Critical: delete markers from the previous phase first.
        # H3 kept different marker IDs alive, which caused all text
        # states to overlap on screen.
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        x, y, z = self.latest_position
        anchor = (
            self.trigger_position
            or self.focus_position
            or self.latest_position
        )
        ax, ay, az = anchor
        now = self.now_s()
        force_norm = math.sqrt(sum(value * value for value in force))
        current_error, nearest = self.nearest_path_point(
            self.latest_position
        )

        def header():
            result = WrenchStamped().header
            result.frame_id = 'world'
            result.stamp = self.get_clock().now().to_msg()
            return result

        def text_marker(
            marker_id: int,
            value: str,
            position: Tuple[float, float, float],
            size: float,
            color: Tuple[float, float, float, float],
        ) -> None:
            marker = Marker()
            marker.header = header()
            marker.ns = 'cq01_disturbance_clean'
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.z = size
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.text = value
            array.markers.append(marker)

        # Actual post-trigger trajectory: thin enough not to cover the map.
        if len(self.disturbance_trail) >= 2:
            trail = Marker()
            trail.header = header()
            trail.ns = 'cq01_disturbance_clean'
            trail.id = 10
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = 0.28
            trail.color.r = 1.0
            trail.color.g = 0.05
            trail.color.b = 0.85
            trail.color.a = 0.90
            trail.points = [
                Point(x=px, y=py, z=pz)
                for px, py, pz in self.disturbance_trail[-300:]
            ]
            array.markers.append(trail)

        # Current shortest distance to the active A* path.
        if (
            nearest is not None
            and phase not in ('scheduled', 'idle')
        ):
            deviation = Marker()
            deviation.header = header()
            deviation.ns = 'cq01_disturbance_clean'
            deviation.id = 11
            deviation.type = Marker.LINE_LIST
            deviation.action = Marker.ADD
            deviation.scale.x = 0.16
            deviation.color.r = 0.10
            deviation.color.g = 1.0
            deviation.color.b = 1.0
            deviation.color.a = 0.90
            deviation.points = [
                Point(
                    x=nearest[0],
                    y=nearest[1],
                    z=nearest[2],
                ),
                Point(x=x, y=y, z=z),
            ]
            array.markers.append(deviation)

        # Peak deviation: one restrained marker and one small label.
        if (
            self.peak_position is not None
            and self.peak_reference_position is not None
        ):
            peak_line = Marker()
            peak_line.header = header()
            peak_line.ns = 'cq01_disturbance_clean'
            peak_line.id = 12
            peak_line.type = Marker.LINE_LIST
            peak_line.action = Marker.ADD
            peak_line.scale.x = 0.24
            peak_line.color.r = 1.0
            peak_line.color.g = 0.05
            peak_line.color.b = 0.05
            peak_line.color.a = 0.95
            peak_line.points = [
                Point(
                    x=self.peak_reference_position[0],
                    y=self.peak_reference_position[1],
                    z=self.peak_reference_position[2],
                ),
                Point(
                    x=self.peak_position[0],
                    y=self.peak_position[1],
                    z=self.peak_position[2],
                ),
            ]
            array.markers.append(peak_line)

            text_marker(
                13,
                f'PEAK {self.peak_cross_track_m:.2f} m',
                (
                    self.peak_position[0],
                    self.peak_position[1],
                    self.peak_position[2] + 2.5,
                ),
                0.95,
                (1.0, 0.20, 0.10, 1.0),
            )

        if phase == 'scheduled':
            remaining = max(
                0.0,
                (self.scheduled_trigger_time or now) - now,
            )
            text_marker(
                20,
                f'WIND IN {remaining:.1f} s',
                (ax, ay, az + 6.0),
                1.30,
                (1.0, 0.90, 0.10, 1.0),
            )

        elif phase == 'active':
            arrow = Marker()
            arrow.header = header()
            arrow.ns = 'cq01_disturbance_clean'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=x, y=y, z=z + 0.5),
                Point(
                    x=x + self.visual_force_scale_m_per_n * force[0],
                    y=y + self.visual_force_scale_m_per_n * force[1],
                    z=z + 0.5
                    + self.visual_force_scale_m_per_n * force[2],
                ),
            ]
            arrow.scale.x = 0.55
            arrow.scale.y = 1.20
            arrow.scale.z = 1.50
            arrow.color.r = 1.0
            arrow.color.g = 0.10
            arrow.color.b = 0.02
            arrow.color.a = 1.0
            array.markers.append(arrow)

            text_marker(
                2,
                f'WIND {force_norm:.2f} N',
                (ax, ay, az + 6.0),
                1.35,
                (1.0, 0.72, 0.05, 1.0),
            )

        elif phase in ('afterglow', 'recovering'):
            # Historical direction only, clearly separated from the
            # current zero-force dynamics.
            old_arrow = Marker()
            old_arrow.header = header()
            old_arrow.ns = 'cq01_disturbance_clean'
            old_arrow.id = 30
            old_arrow.type = Marker.ARROW
            old_arrow.action = Marker.ADD
            old_arrow.points = [
                Point(x=ax, y=ay, z=az + 0.5),
                Point(
                    x=ax
                    + self.visual_force_scale_m_per_n
                    * self.force[0],
                    y=ay
                    + self.visual_force_scale_m_per_n
                    * self.force[1],
                    z=az + 0.5
                    + self.visual_force_scale_m_per_n
                    * self.force[2],
                ),
            ]
            old_arrow.scale.x = 0.40
            old_arrow.scale.y = 0.90
            old_arrow.scale.z = 1.10
            old_arrow.color.r = 1.0
            old_arrow.color.g = 0.35
            old_arrow.color.b = 0.05
            old_arrow.color.a = 0.45
            array.markers.append(old_arrow)

            deviation_value = (
                f'{current_error:.2f} m'
                if current_error is not None else '--'
            )
            text_marker(
                31,
                f'RECOVERING | deviation {deviation_value}',
                (ax, ay, az + 6.0),
                1.15,
                (0.20, 1.0, 1.0, 1.0),
            )

        elif phase in ('recovered', 'completed'):
            text_marker(
                40,
                (
                    'RECOVERED'
                    f' | peak {self.peak_cross_track_m:.2f} m'
                ),
                (ax, ay, az + 6.0),
                1.25,
                (0.20, 1.0, 0.30, 1.0),
            )

        elif phase == 'failed':
            text_marker(
                50,
                'DISTURBANCE TEST FAILED',
                (ax, ay, az + 6.0),
                1.25,
                (1.0, 0.05, 0.05, 1.0),
            )

        self.marker_pub.publish(array)

    def timer_callback(self) -> None:
        now = self.now_s()
        self.publish_focus_transform()
        markers_due = self.marker_update_due(now)

        if (
            self.scheduled_trigger_time is not None
            and not self.triggered_once
            and now >= self.scheduled_trigger_time
        ):
            self.start_disturbance(now)

        if self.is_disturbance_active(now):
            elapsed = now - float(self.disturbance_start_time)
            gain = max(0.0, min(1.0, self.envelope(elapsed)))
            force = [gain * value for value in self.force]
            torque = [gain * value for value in self.torque]
            self.visual_phase = 'active'
            self.publish_wrench(force, torque, True)
            if markers_due:
                self.publish_markers(force, 'active')
            self.publish_status(
                'disturbance_active',
                applied_force_n=force,
            )
            return

        # Continue publishing a real zero wrench after the pulse.
        self.publish_zero_wrench()

        if not self.triggered_once:
            if self.scheduled_trigger_time is not None:
                self.visual_phase = 'scheduled'
                if markers_due:
                    self.publish_markers(
                        [0.0, 0.0, 0.0],
                        'scheduled',
                    )
                remaining = max(
                    0.0,
                    self.scheduled_trigger_time - now,
                )
                self.publish_status(
                    'trigger_scheduled',
                    countdown_s=remaining,
                )
            return

        if self.disturbance_end_time is None:
            return

        elapsed_after_end = now - self.disturbance_end_time

        # Never hide the evidence immediately just because numerical
        # recovery was detected in a fraction of a second.
        if elapsed_after_end <= self.visual_evidence_hold_s:
            self.visual_phase = (
                'afterglow'
                if elapsed_after_end <= self.visual_afterglow_s
                else 'recovering'
            )
            if markers_due:
                self.publish_markers(
                    [0.0, 0.0, 0.0],
                    self.visual_phase,
                )
            self.publish_status(
                'disturbance_evidence_hold',
                evidence_remaining_s=max(
                    0.0,
                    self.visual_evidence_hold_s
                    - elapsed_after_end,
                ),
                recovery_time_s=self.recovery_time_s,
            )
            return

        if self.mission_final_state is not None:
            self.visual_phase = (
                'completed'
                if self.mission_final_state
                == 'mission_completed'
                else 'failed'
            )
        elif self.recovery_time_s is not None:
            self.visual_phase = 'recovered'
        else:
            self.visual_phase = 'recovering'

        if markers_due:
            self.publish_markers(
                [0.0, 0.0, 0.0],
                self.visual_phase,
            )

        self.publish_status(
            'results_saved'
            if self.saved_output_dir is not None
            else self.visual_phase,
            recovery_time_s=self.recovery_time_s,
        )

    def compute_metrics(self, final_state: str) -> Dict:
        finite_cross_track = [
            sample['cross_track_error_m']
            for sample in self.samples
            if math.isfinite(sample['cross_track_error_m'])
        ]
        peak_cross_track = (
            max(finite_cross_track)
            if finite_cross_track else None
        )
        final_cross_track = (
            sum(finite_cross_track[-50:])
            / len(finite_cross_track[-50:])
            if finite_cross_track else None
        )
        max_tilt = max(
            (sample['tilt_deg'] for sample in self.samples),
            default=0.0,
        )
        rpm_threshold = (
            self.saturation_ratio_threshold * self.maximum_rpm
        )
        rpm_values = [
            sample[f'rpm_{index}']
            for sample in self.samples
            for index in range(1, 5)
        ]
        saturation_fraction = (
            sum(value >= rpm_threshold for value in rpm_values)
            / len(rpm_values)
            if rpm_values else 0.0
        )
        impulse = 0.0
        for previous, current in zip(
            self.samples[:-1],
            self.samples[1:],
        ):
            dt = current['time_s'] - previous['time_s']
            f0 = math.sqrt(
                previous['force_x_n'] ** 2
                + previous['force_y_n'] ** 2
                + previous['force_z_n'] ** 2
            )
            f1 = math.sqrt(
                current['force_x_n'] ** 2
                + current['force_y_n'] ** 2
                + current['force_z_n'] ** 2
            )
            impulse += 0.5 * (f0 + f1) * max(0.0, dt)

        criteria = {
            'disturbance_applied': impulse > 0.05,
            'recovered': self.recovery_time_s is not None,
            'tilt_pass': max_tilt <= self.maximum_tilt_deg,
            'rpm_saturation_pass':
                saturation_fraction
                <= self.maximum_saturation_fraction,
            'mission_continued_to_completion':
                self.mission_completed_after_trigger,
        }
        return {
            'experiment': 'cq01_v8a2_low_city_external_disturbance',
            'final_state': final_state,
            'trigger_mode': self.trigger_mode,
            'trigger_after_reached_index':
                self.trigger_after_reached_index,
            'command_force_n': self.force,
            'command_torque_nm': self.torque,
            'duration_s': self.duration_s,
            'baseline_cross_track_m':
                self.baseline_cross_track_m,
            'peak_cross_track_error_m': peak_cross_track,
            'final_cross_track_error_m': final_cross_track,
            'recovery_time_s': self.recovery_time_s,
            'disturbance_impulse_n_s': impulse,
            'maximum_tilt_deg': max_tilt,
            'rpm_saturation_fraction': saturation_fraction,
            'mission_completed_after_trigger':
                self.mission_completed_after_trigger,
            'criteria': criteria,
            'pass': all(criteria.values()),
            'sample_count': len(self.samples),
            'visualization': {
                'force_scale_m_per_n':
                    self.visual_force_scale_m_per_n,
                'afterglow_s': self.visual_afterglow_s,
                'recovery_hold_s':
                    self.visual_recovery_hold_s,
                'peak_deviation_marker_m':
                    self.peak_cross_track_m,
            },
        }

    def save_results(self, final_state: str) -> None:
        if self.saved_output_dir is not None:
            return
        metrics = self.compute_metrics(final_state)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = (
            self.results_root / f'cq01_disturbance_{timestamp}'
        )
        output_dir.mkdir(parents=True, exist_ok=True)

        (output_dir / 'summary.json').write_text(
            json.dumps(metrics, indent=2, ensure_ascii=False),
            encoding='utf-8',
        )
        if self.samples:
            with (output_dir / 'samples.csv').open(
                'w',
                newline='',
                encoding='utf-8',
            ) as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=list(self.samples[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.samples)

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            times = [sample['time_s'] for sample in self.samples]
            if times:
                plt.figure()
                plt.plot(
                    times,
                    [
                        sample['cross_track_error_m']
                        for sample in self.samples
                    ],
                    label='Cross-track error',
                )
                plt.axhline(
                    self.recovery_tolerance_m,
                    linestyle='--',
                    label='Recovery tolerance',
                )
                plt.xlabel('Time after trigger (s)')
                plt.ylabel('Cross-track error (m)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'cross_track_recovery.png',
                    dpi=180,
                )
                plt.close()

                plt.figure()
                plt.plot(
                    times,
                    [sample['force_norm_n'] for sample in self.samples],
                    label='Applied force norm',
                )
                plt.xlabel('Time after trigger (s)')
                plt.ylabel('Force (N)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'disturbance_profile.png',
                    dpi=180,
                )
                plt.close()

                plt.figure()
                plt.plot(
                    times,
                    [sample['roll_deg'] for sample in self.samples],
                    label='Roll',
                )
                plt.plot(
                    times,
                    [sample['pitch_deg'] for sample in self.samples],
                    label='Pitch',
                )
                plt.xlabel('Time after trigger (s)')
                plt.ylabel('Angle (deg)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'attitude_response.png',
                    dpi=180,
                )
                plt.close()

                plt.figure()
                for index in range(1, 5):
                    plt.plot(
                        times,
                        [
                            sample[f'rpm_{index}']
                            for sample in self.samples
                        ],
                        label=f'RPM {index}',
                    )
                plt.xlabel('Time after trigger (s)')
                plt.ylabel('Motor RPM')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'motor_rpm_response.png',
                    dpi=180,
                )
                plt.close()
        except Exception as exception:
            self.get_logger().warning(
                f'Plots were not generated: {exception}'
            )

        self.saved_output_dir = output_dir
        self.get_logger().info(
            'CQ-01 integrated disturbance result: '
            f'pass={metrics["pass"]}, '
            f'peak_cross_track='
            f'{metrics["peak_cross_track_error_m"]}, '
            f'recovery={metrics["recovery_time_s"]}, '
            f'mission_completed='
            f'{metrics["mission_completed_after_trigger"]}'
        )
        self.get_logger().info(
            f'Results saved to: {output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01IntegratedDisturbanceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.triggered_once:
            node.save_results(final_state='keyboard_interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
