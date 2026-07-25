#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, String
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


def point_segment_distance_xy(
    point: Tuple[float, float, float],
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
) -> Tuple[float, Tuple[float, float, float]]:
    px, py, _ = point
    ax, ay, az = a
    bx, by, bz = b
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-12:
        return math.hypot(px - ax, py - ay), a

    t = ((px - ax) * dx + (py - ay) * dy) / denominator
    t = max(0.0, min(1.0, t))
    reference = (
        ax + t * dx,
        ay + t * dy,
        az + t * (bz - az),
    )
    return math.hypot(px - reference[0], py - reference[1]), reference


class V8A3LowCityStrongWind(Node):
    """Closed-loop P1-P2 strong-wind deviation experiment.

    The node never publishes navigation goals. It freezes the active A* path
    at the trigger point, ramps a map-frame -Y force, and stops the force when
    net horizontal cross-track deviation reaches 8.0 m. Independent hard
    limits stop the force at 10.5 m, 44 deg tilt, insufficient obstacle
    clearance, sustained RPM saturation, or 4.5 s active duration.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v8a3_low_city_strong_wind')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.declare_parameter('gate_x_m', 49.0)
        self.declare_parameter('gate_y_m', -93.0)
        self.declare_parameter('gate_z_m', 21.0)
        self.declare_parameter('gate_radius_xy_m', 2.5)
        self.declare_parameter('gate_tolerance_z_m', 3.5)

        self.declare_parameter('wind_direction_x', 0.0)
        self.declare_parameter('wind_direction_y', -1.0)
        self.declare_parameter('wind_direction_z', 0.0)
        self.declare_parameter('initial_force_n', 4.0)
        self.declare_parameter('force_ramp_n_per_s', 2.0)
        self.declare_parameter('maximum_force_n', 11.5)
        self.declare_parameter('ramp_in_s', 0.30)
        self.declare_parameter('maximum_active_time_s', 4.5)

        self.declare_parameter('target_net_deviation_m', 8.0)
        self.declare_parameter('preferred_max_deviation_m', 10.0)
        self.declare_parameter('hard_stop_deviation_m', 10.5)
        self.declare_parameter('recovery_above_baseline_m', 1.2)
        self.declare_parameter('recovery_hold_s', 1.0)
        self.declare_parameter('maximum_recovery_time_s', 25.0)

        self.declare_parameter('maximum_tilt_deg', 44.0)
        self.declare_parameter('minimum_clearance_m', 2.5)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter('rpm_saturation_ratio', 0.98)
        self.declare_parameter('maximum_saturation_fraction', 0.05)

        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v8a3_strong_wind/low_strong_wind'
            ),
        )

        self.gate = (
            float(self.get_parameter('gate_x_m').value),
            float(self.get_parameter('gate_y_m').value),
            float(self.get_parameter('gate_z_m').value),
        )
        self.gate_radius_xy = float(
            self.get_parameter('gate_radius_xy_m').value
        )
        self.gate_tolerance_z = float(
            self.get_parameter('gate_tolerance_z_m').value
        )

        direction = np.array(
            [
                float(self.get_parameter('wind_direction_x').value),
                float(self.get_parameter('wind_direction_y').value),
                float(self.get_parameter('wind_direction_z').value),
            ],
            dtype=float,
        )
        norm = float(np.linalg.norm(direction))
        if norm <= 1.0e-9:
            raise RuntimeError('Strong-wind direction must be non-zero.')
        self.wind_direction = direction / norm

        self.initial_force_n = float(
            self.get_parameter('initial_force_n').value
        )
        self.force_ramp_n_per_s = float(
            self.get_parameter('force_ramp_n_per_s').value
        )
        self.maximum_force_n = float(
            self.get_parameter('maximum_force_n').value
        )
        self.ramp_in_s = float(self.get_parameter('ramp_in_s').value)
        self.maximum_active_time_s = float(
            self.get_parameter('maximum_active_time_s').value
        )

        self.target_net_deviation_m = float(
            self.get_parameter('target_net_deviation_m').value
        )
        self.preferred_max_deviation_m = float(
            self.get_parameter('preferred_max_deviation_m').value
        )
        self.hard_stop_deviation_m = float(
            self.get_parameter('hard_stop_deviation_m').value
        )
        self.recovery_above_baseline_m = float(
            self.get_parameter('recovery_above_baseline_m').value
        )
        self.recovery_hold_s = float(
            self.get_parameter('recovery_hold_s').value
        )
        self.maximum_recovery_time_s = float(
            self.get_parameter('maximum_recovery_time_s').value
        )

        self.maximum_tilt_deg = float(
            self.get_parameter('maximum_tilt_deg').value
        )
        self.minimum_clearance_m = float(
            self.get_parameter('minimum_clearance_m').value
        )
        self.maximum_rpm = float(
            self.get_parameter('maximum_rpm').value
        )
        self.rpm_saturation_ratio = float(
            self.get_parameter('rpm_saturation_ratio').value
        )
        self.maximum_saturation_fraction = float(
            self.get_parameter('maximum_saturation_fraction').value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )

        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/cq01_v8a3/low/wrench_cmd',
            20,
        )
        self.active_pub = self.create_publisher(
            Bool,
            '/cq01_v8a3/low/active',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/cq01_v8a3/low/status',
            transient,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/cq01_v8a3/low/markers',
            10,
        )
        self.frozen_path_pub = self.create_publisher(
            NavPath,
            '/cq01_v8a3/low/frozen_reference_path',
            transient,
        )

        self.create_subscription(
            String,
            '/cq01_v8a3/mission_status',
            self.mission_callback,
            transient,
        )
        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            NavPath,
            '/drone/planned_path_3d',
            self.path_callback,
            transient,
        )
        self.create_subscription(
            Float32MultiArray,
            '/drone/motor_rpm',
            self.rpm_callback,
            50,
        )
        self.create_subscription(
            WrenchStamped,
            '/drone/external_wrench_applied',
            self.applied_wrench_callback,
            50,
        )
        self.create_subscription(
            PointCloud2,
            '/map_generator/global_cloud',
            self.map_callback,
            transient,
        )

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.marker_timer = self.create_timer(
            0.10,
            self.publish_markers,
        )
        self.status_timer = self.create_timer(
            0.50,
            self.publish_status,
        )

        self.state = 'waiting_for_P1'
        self.mission: Dict = {}
        self.maximum_reached_index = -1
        self.segment_armed = False
        self.triggered_once = False
        self.trigger_position: Optional[
            Tuple[float, float, float]
        ] = None

        self.position: Optional[Tuple[float, float, float]] = None
        self.latest_tilt_deg = 0.0
        self.maximum_observed_tilt_deg = 0.0
        self.latest_rpm: List[float] = []
        self.latest_path: List[Tuple[float, float, float]] = []
        self.frozen_path: List[Tuple[float, float, float]] = []
        self.local_obstacle_points: Optional[np.ndarray] = None
        self.map_ready = False

        self.recent_cross_track: Deque[
            Tuple[float, float]
        ] = deque(maxlen=300)
        self.latest_cross_track_m: Optional[float] = None
        self.latest_reference: Optional[
            Tuple[float, float, float]
        ] = None
        self.baseline_cross_track_m: Optional[float] = None
        self.peak_cross_track_m = 0.0
        self.peak_net_deviation_m = 0.0
        self.peak_position: Optional[
            Tuple[float, float, float]
        ] = None
        self.peak_reference: Optional[
            Tuple[float, float, float]
        ] = None

        self.active_start_s: Optional[float] = None
        self.force_stop_s: Optional[float] = None
        self.recovery_candidate_s: Optional[float] = None
        self.recovery_time_s: Optional[float] = None
        self.stop_reason: Optional[str] = None
        self.safety_abort_reason: Optional[str] = None
        self.target_reached = False

        self.minimum_obstacle_clearance_m: Optional[float] = None
        self.last_clearance_update_s = -1.0e9
        self.applied_impulse_n_s = 0.0
        self.last_applied_time_s: Optional[float] = None
        self.rpm_samples = 0
        self.rpm_saturated_samples = 0
        self.samples: List[Dict] = []
        self.local_trail: List[Tuple[float, float, float]] = []
        self.last_trail_s = -1.0e9
        self.output_dir: Optional[Path] = None
        self.result_saved = False
        self.last_timer_s = self.now_s()

        self.get_logger().warning(
            'V8-A3 low-city strong wind ready: '
            'gate=(49,-93,21), direction=-Y, '
            'target=8.0m, preferred=8-10m, hard_stop=10.5m.'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def mission_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        self.mission = payload

        if 'reached_index' in payload:
            try:
                self.maximum_reached_index = max(
                    self.maximum_reached_index,
                    int(payload.get('reached_index', -1)),
                )
            except (TypeError, ValueError):
                pass

        try:
            current_index = int(
                payload.get('current_index', -1)
            )
        except (TypeError, ValueError):
            current_index = -1

        if (
            bool(payload.get('active', False))
            and current_index == 2
            and self.maximum_reached_index >= 1
            and self.maximum_reached_index < 2
        ):
            if not self.segment_armed:
                self.segment_armed = True
                self.state = 'armed_P1_to_P2'
                self.get_logger().warning(
                    'V8-A3 P1->P2 STRONG-WIND SEGMENT ARMED. '
                    'Camera should lock before entering the gate.'
                )

        if (
            self.triggered_once
            and self.maximum_reached_index >= 2
            and self.state == 'active'
            and not self.target_reached
        ):
            self.stop_force(
                'P2_reached_before_8m_target',
                safety_abort=True,
            )

        if (
            self.maximum_reached_index >= 2
            and not self.triggered_once
            and not self.result_saved
        ):
            self.stop_reason = 'gate_missed_before_P2'
            self.safety_abort_reason = self.stop_reason
            self.state = 'failed_waiting_for_P4'

        if (
            payload.get('state') == 'mission_completed'
            and int(payload.get('reached_index', -1)) == 4
        ):
            if not self.triggered_once:
                self.stop_reason = self.stop_reason or 'never_triggered'
                self.safety_abort_reason = (
                    self.safety_abort_reason or self.stop_reason
                )
            self.save_results_if_ready()

    def path_callback(self, message: NavPath) -> None:
        points = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
                float(p.pose.position.z),
            )
            for p in message.poses
        ]
        if len(points) >= 2:
            self.latest_path = points

    def map_callback(self, message: PointCloud2) -> None:
        try:
            array = point_cloud2.read_points_numpy(
                message,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
            array = np.asarray(array, dtype=np.float32)
        except Exception:
            values = list(
                point_cloud2.read_points(
                    message,
                    field_names=('x', 'y', 'z'),
                    skip_nans=True,
                )
            )
            array = np.asarray(values, dtype=np.float32)

        if array.size == 0:
            return
        array = array.reshape((-1, 3))
        mask = (
            (array[:, 0] >= 25.0)
            & (array[:, 0] <= 80.0)
            & (array[:, 1] >= -120.0)
            & (array[:, 1] <= -65.0)
            & (array[:, 2] >= 0.0)
            & (array[:, 2] <= 55.0)
        )
        self.local_obstacle_points = array[mask]
        self.map_ready = (
            self.local_obstacle_points is not None
            and len(self.local_obstacle_points) > 0
        )
        self.get_logger().info(
            'V8-A3 local safety cloud ready: '
            f'{len(self.local_obstacle_points)} points.'
        )

    def rpm_callback(self, message: Float32MultiArray) -> None:
        self.latest_rpm = [float(value) for value in message.data]

    def applied_wrench_callback(
        self,
        message: WrenchStamped,
    ) -> None:
        now = self.now_s()
        if self.last_applied_time_s is None:
            self.last_applied_time_s = now
            return
        dt = max(0.0, min(0.1, now - self.last_applied_time_s))
        self.last_applied_time_s = now
        if not self.triggered_once:
            return
        force = message.wrench.force
        magnitude = math.sqrt(
            force.x * force.x
            + force.y * force.y
            + force.z * force.z
        )
        if self.state in ('active', 'recovery'):
            self.applied_impulse_n_s += magnitude * dt

    def nearest_frozen_path(
        self,
        position: Tuple[float, float, float],
    ) -> Tuple[Optional[float], Optional[Tuple[float, float, float]]]:
        if len(self.frozen_path) < 2:
            return None, None
        best_distance = float('inf')
        best_reference = None
        for a, b in zip(self.frozen_path[:-1], self.frozen_path[1:]):
            distance, reference = point_segment_distance_xy(
                position,
                a,
                b,
            )
            if distance < best_distance:
                best_distance = distance
                best_reference = reference
        return best_distance, best_reference

    def nearest_latest_path(
        self,
        position: Tuple[float, float, float],
    ) -> Tuple[Optional[float], Optional[Tuple[float, float, float]]]:
        if len(self.latest_path) < 2:
            return None, None
        best_distance = float('inf')
        best_reference = None
        for a, b in zip(self.latest_path[:-1], self.latest_path[1:]):
            distance, reference = point_segment_distance_xy(
                position,
                a,
                b,
            )
            if distance < best_distance:
                best_distance = distance
                best_reference = reference
        return best_distance, best_reference

    def odom_callback(self, message: Odometry) -> None:
        now = self.now_s()
        p = message.pose.pose.position
        self.position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        q = message.pose.pose.orientation
        roll, pitch = quaternion_to_roll_pitch(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        self.latest_tilt_deg = math.degrees(
            max(abs(roll), abs(pitch))
        )
        if self.triggered_once:
            self.maximum_observed_tilt_deg = max(
                self.maximum_observed_tilt_deg,
                self.latest_tilt_deg,
            )

        if self.frozen_path:
            cross_track, reference = self.nearest_frozen_path(
                self.position
            )
        else:
            cross_track, reference = self.nearest_latest_path(
                self.position
            )

        self.latest_cross_track_m = cross_track
        self.latest_reference = reference
        if cross_track is not None:
            self.recent_cross_track.append((now, cross_track))

        if (
            self.triggered_once
            and cross_track is not None
            and self.baseline_cross_track_m is not None
        ):
            net = max(
                0.0,
                cross_track - self.baseline_cross_track_m,
            )
            if cross_track >= self.peak_cross_track_m:
                self.peak_cross_track_m = cross_track
                self.peak_net_deviation_m = net
                self.peak_position = self.position
                self.peak_reference = reference

        if (
            self.triggered_once
            and now - self.last_trail_s >= 0.10
        ):
            self.local_trail.append(self.position)
            self.last_trail_s = now

        if (
            self.triggered_once
            and self.state in ('active', 'recovery')
        ):
            self.update_clearance(now)
            saturated = bool(
                self.latest_rpm
                and max(abs(value) for value in self.latest_rpm)
                >= self.rpm_saturation_ratio * self.maximum_rpm
            )
            self.rpm_samples += 1
            if saturated:
                self.rpm_saturated_samples += 1
            self.samples.append(
                {
                    'time_s': now,
                    'x_m': self.position[0],
                    'y_m': self.position[1],
                    'z_m': self.position[2],
                    'cross_track_m':
                        cross_track if cross_track is not None else float('nan'),
                    'net_deviation_m':
                        max(
                            0.0,
                            (cross_track or 0.0)
                            - (self.baseline_cross_track_m or 0.0),
                        ),
                    'tilt_deg': self.latest_tilt_deg,
                    'minimum_clearance_m':
                        self.minimum_obstacle_clearance_m
                        if self.minimum_obstacle_clearance_m is not None
                        else float('nan'),
                    'maximum_rpm':
                        max(self.latest_rpm)
                        if self.latest_rpm else float('nan'),
                    'state': self.state,
                }
            )

    def update_clearance(self, now: float) -> None:
        if (
            self.position is None
            or not self.map_ready
            or self.local_obstacle_points is None
            or now - self.last_clearance_update_s < 0.10
        ):
            return
        self.last_clearance_update_s = now
        p = np.asarray(self.position, dtype=np.float32)
        distances = np.linalg.norm(
            self.local_obstacle_points - p,
            axis=1,
        )
        if distances.size == 0:
            return
        clearance = float(np.min(distances))
        if (
            self.minimum_obstacle_clearance_m is None
            or clearance < self.minimum_obstacle_clearance_m
        ):
            self.minimum_obstacle_clearance_m = clearance

    def inside_gate(self) -> bool:
        if self.position is None:
            return False
        dx = self.position[0] - self.gate[0]
        dy = self.position[1] - self.gate[1]
        dz = abs(self.position[2] - self.gate[2])
        return (
            math.hypot(dx, dy) <= self.gate_radius_xy
            and dz <= self.gate_tolerance_z
        )

    def compute_baseline(self, now: float) -> Optional[float]:
        values = [
            value
            for timestamp, value in self.recent_cross_track
            if now - 1.5 <= timestamp <= now
        ]
        if not values:
            return self.latest_cross_track_m
        return float(np.median(np.asarray(values, dtype=float)))

    def publish_frozen_path(self) -> None:
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'world'
        from geometry_msgs.msg import PoseStamped
        for point in self.frozen_path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.frozen_path_pub.publish(message)

    def start_experiment(self, now: float) -> None:
        if self.triggered_once:
            return
        if len(self.latest_path) < 2:
            return
        if not self.map_ready:
            self.state = 'armed_waiting_for_safety_cloud'
            return

        self.triggered_once = True
        self.trigger_position = self.position
        self.frozen_path = list(self.latest_path)
        self.baseline_cross_track_m = self.compute_baseline(now)
        if self.baseline_cross_track_m is None:
            self.baseline_cross_track_m = 0.0
        self.peak_cross_track_m = self.baseline_cross_track_m
        self.peak_net_deviation_m = 0.0
        self.active_start_s = now
        self.state = 'active'
        self.publish_frozen_path()

        self.get_logger().warning(
            'V8-A3 LOW-CITY STRONG WIND ACTIVE: '
            f'trigger={self.trigger_position}, '
            f'baseline={self.baseline_cross_track_m:.3f}m, '
            'direction=-Y, adaptive force 4.0->11.5N, '
            'target net deviation=8.0m.'
        )

    def force_magnitude(self, elapsed: float) -> float:
        target = min(
            self.maximum_force_n,
            self.initial_force_n
            + self.force_ramp_n_per_s * elapsed,
        )
        ramp_gain = min(
            1.0,
            max(0.0, elapsed / max(0.05, self.ramp_in_s)),
        )
        return target * ramp_gain

    def publish_wrench(
        self,
        force: Tuple[float, float, float],
        active: bool,
    ) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x = float(force[0])
        message.wrench.force.y = float(force[1])
        message.wrench.force.z = float(force[2])
        self.wrench_pub.publish(message)

        active_message = Bool()
        active_message.data = bool(active)
        self.active_pub.publish(active_message)

    def publish_zero(self) -> None:
        self.publish_wrench((0.0, 0.0, 0.0), False)

    def stop_force(
        self,
        reason: str,
        safety_abort: bool = False,
    ) -> None:
        if self.state != 'active':
            return
        self.publish_zero()
        self.force_stop_s = self.now_s()
        self.stop_reason = reason
        if safety_abort:
            self.safety_abort_reason = reason
            self.state = 'failed_waiting_for_P4'
            self.get_logger().error(
                f'V8-A3 STRONG WIND SAFETY STOP: {reason}; '
                f'net={self.peak_net_deviation_m:.3f}m, '
                f'tilt={self.maximum_observed_tilt_deg:.2f}deg, '
                f'clearance={self.minimum_obstacle_clearance_m}.'
            )
        else:
            self.state = 'recovery'
            self.get_logger().warning(
                f'V8-A3 8M TARGET REACHED; WIND STOPPED. '
                f'peak_net={self.peak_net_deviation_m:.3f}m, '
                f'peak_cross_track={self.peak_cross_track_m:.3f}m.'
            )

    def current_net_deviation(self) -> float:
        if (
            self.latest_cross_track_m is None
            or self.baseline_cross_track_m is None
        ):
            return 0.0
        return max(
            0.0,
            self.latest_cross_track_m
            - self.baseline_cross_track_m,
        )

    def timer_callback(self) -> None:
        now = self.now_s()
        dt = max(0.0, min(0.05, now - self.last_timer_s))
        self.last_timer_s = now

        if (
            self.segment_armed
            and not self.triggered_once
            and self.inside_gate()
        ):
            self.start_experiment(now)

        if self.state == 'active':
            if self.active_start_s is None:
                self.stop_force('invalid_active_start', True)
                return

            elapsed = now - self.active_start_s
            net = self.current_net_deviation()

            if net >= self.hard_stop_deviation_m:
                self.stop_force(
                    'hard_stop_deviation_exceeded',
                    True,
                )
                return
            if net >= self.target_net_deviation_m:
                self.target_reached = True
                self.stop_force('target_8m_reached', False)
                return
            if self.latest_tilt_deg >= self.maximum_tilt_deg:
                self.stop_force('maximum_tilt_exceeded', True)
                return
            if (
                self.minimum_obstacle_clearance_m is not None
                and self.minimum_obstacle_clearance_m
                < self.minimum_clearance_m
            ):
                self.stop_force(
                    'minimum_obstacle_clearance_violated',
                    True,
                )
                return
            if elapsed >= self.maximum_active_time_s:
                self.stop_force(
                    'maximum_active_time_without_8m',
                    True,
                )
                return

            saturation_fraction = (
                self.rpm_saturated_samples / self.rpm_samples
                if self.rpm_samples else 0.0
            )
            if (
                self.rpm_samples >= 20
                and saturation_fraction
                > self.maximum_saturation_fraction
            ):
                self.stop_force(
                    'rpm_saturation_fraction_exceeded',
                    True,
                )
                return

            magnitude = self.force_magnitude(elapsed)
            force = tuple(
                float(value * magnitude)
                for value in self.wind_direction
            )
            self.publish_wrench(force, True)

        elif self.state == 'recovery':
            self.publish_zero()
            if (
                self.force_stop_s is not None
                and now - self.force_stop_s
                > self.maximum_recovery_time_s
            ):
                self.safety_abort_reason = 'recovery_timeout'
                self.stop_reason = self.stop_reason or 'recovery_timeout'
                self.state = 'failed_waiting_for_P4'
                return

            if (
                self.latest_cross_track_m is not None
                and self.baseline_cross_track_m is not None
                and self.latest_cross_track_m
                <= self.baseline_cross_track_m
                + self.recovery_above_baseline_m
            ):
                if self.recovery_candidate_s is None:
                    self.recovery_candidate_s = now
                elif (
                    now - self.recovery_candidate_s
                    >= self.recovery_hold_s
                ):
                    self.recovery_time_s = (
                        self.recovery_candidate_s
                        - (self.force_stop_s or self.recovery_candidate_s)
                    )
                    self.state = 'recovered_waiting_for_P4'
                    self.get_logger().warning(
                        'V8-A3 LOW-CITY STRONG WIND RECOVERED: '
                        f'recovery_time={self.recovery_time_s:.3f}s, '
                        f'peak_net={self.peak_net_deviation_m:.3f}m.'
                    )
            else:
                self.recovery_candidate_s = None
        else:
            self.publish_zero()

        if (
            self.mission.get('state') == 'mission_completed'
            and int(self.mission.get('reached_index', -1)) == 4
        ):
            self.save_results_if_ready()

    def marker_header(self):
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        return header

    def text_marker(
        self,
        marker_id: int,
        text: str,
        position: Tuple[float, float, float],
        scale: float = 1.5,
        color: Tuple[float, float, float, float] = (
            1.0, 1.0, 1.0, 1.0
        ),
    ) -> Marker:
        marker = Marker()
        marker.header = self.marker_header()
        marker.ns = 'v8a3_low_strong_wind'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        return marker

    def publish_markers(self) -> None:
        array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        gate = Marker()
        gate.header = self.marker_header()
        gate.ns = 'v8a3_low_strong_wind'
        gate.id = 1
        gate.type = Marker.CYLINDER
        gate.action = Marker.ADD
        gate.pose.position.x = self.gate[0]
        gate.pose.position.y = self.gate[1]
        gate.pose.position.z = self.gate[2]
        gate.pose.orientation.w = 1.0
        gate.scale.x = 2.0 * self.gate_radius_xy
        gate.scale.y = 2.0 * self.gate_radius_xy
        gate.scale.z = 0.18
        gate.color.r = 0.10
        gate.color.g = 0.75
        gate.color.b = 1.0
        gate.color.a = 0.08
        array.markers.append(gate)

        if len(self.local_trail) >= 2:
            trail = Marker()
            trail.header = self.marker_header()
            trail.ns = 'v8a3_low_strong_wind'
            trail.id = 2
            trail.type = Marker.LINE_STRIP
            trail.action = Marker.ADD
            trail.scale.x = 0.75
            trail.color.r = 1.0
            trail.color.g = 0.30
            trail.color.b = 0.05
            trail.color.a = 1.0
            for x, y, z in self.local_trail:
                p = Point()
                p.x, p.y, p.z = x, y, z
                trail.points.append(p)
            array.markers.append(trail)

        if self.position is not None and self.state == 'active':
            elapsed = (
                self.now_s() - self.active_start_s
                if self.active_start_s is not None else 0.0
            )
            magnitude = self.force_magnitude(elapsed)
            arrow = Marker()
            arrow.header = self.marker_header()
            arrow.ns = 'v8a3_low_strong_wind'
            arrow.id = 3
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.75
            arrow.scale.y = 1.35
            arrow.scale.z = 1.35
            arrow.color.r = 0.0
            arrow.color.g = 0.65
            arrow.color.b = 1.0
            arrow.color.a = 1.0

            start = Point()
            start.x, start.y, start.z = self.position
            end = Point()
            visual_scale = 1.6
            end.x = (
                self.position[0]
                + self.wind_direction[0] * magnitude * visual_scale
            )
            end.y = (
                self.position[1]
                + self.wind_direction[1] * magnitude * visual_scale
            )
            end.z = (
                self.position[2]
                + self.wind_direction[2] * magnitude * visual_scale
            )
            arrow.points = [start, end]
            array.markers.append(arrow)

        if (
            self.position is not None
            and self.latest_reference is not None
            and self.triggered_once
        ):
            line = Marker()
            line.header = self.marker_header()
            line.ns = 'v8a3_low_strong_wind'
            line.id = 4
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.55
            line.color.r = 1.0
            line.color.g = 0.95
            line.color.b = 0.10
            line.color.a = 1.0

            a = Point()
            a.x, a.y, a.z = self.latest_reference
            b = Point()
            b.x, b.y, b.z = self.position
            line.points = [a, b]
            array.markers.append(line)

        if (
            self.peak_position is not None
            and self.peak_reference is not None
        ):
            peak_line = Marker()
            peak_line.header = self.marker_header()
            peak_line.ns = 'v8a3_low_strong_wind'
            peak_line.id = 5
            peak_line.type = Marker.LINE_LIST
            peak_line.action = Marker.ADD
            peak_line.scale.x = 0.85
            peak_line.color.r = 1.0
            peak_line.color.g = 0.10
            peak_line.color.b = 0.10
            peak_line.color.a = 1.0
            a = Point()
            a.x, a.y, a.z = self.peak_reference
            b = Point()
            b.x, b.y, b.z = self.peak_position
            peak_line.points = [a, b]
            array.markers.append(peak_line)

            peak_sphere = Marker()
            peak_sphere.header = self.marker_header()
            peak_sphere.ns = 'v8a3_low_strong_wind'
            peak_sphere.id = 6
            peak_sphere.type = Marker.SPHERE
            peak_sphere.action = Marker.ADD
            peak_sphere.pose.position.x = self.peak_position[0]
            peak_sphere.pose.position.y = self.peak_position[1]
            peak_sphere.pose.position.z = self.peak_position[2]
            peak_sphere.pose.orientation.w = 1.0
            peak_sphere.scale.x = 1.8
            peak_sphere.scale.y = 1.8
            peak_sphere.scale.z = 1.8
            peak_sphere.color.r = 1.0
            peak_sphere.color.g = 0.05
            peak_sphere.color.b = 0.05
            peak_sphere.color.a = 1.0
            array.markers.append(peak_sphere)

        net = self.current_net_deviation()
        if self.position is not None:
            state_label = {
                'waiting_for_P1': 'WAITING FOR P1',
                'armed_P1_to_P2': 'P1-P2 STRONG WIND ARMED',
                'armed_waiting_for_safety_cloud': 'WAITING FOR SAFETY CLOUD',
                'active': 'STRONG WIND ACTIVE',
                'recovery': 'WIND OFF | RECOVERING',
                'recovered_waiting_for_P4': 'RECOVERED | CONTINUING MISSION',
                'failed_waiting_for_P4': 'STRONG-WIND TEST STOPPED',
            }.get(self.state, self.state.upper())

            text = (
                f'{state_label}\n'
                f'NET DEVIATION {net:.2f} / '
                f'{self.target_net_deviation_m:.1f} m\n'
                f'PEAK NET {self.peak_net_deviation_m:.2f} m'
            )
            if self.state == 'active' and self.active_start_s is not None:
                text += (
                    f'\nFORCE '
                    f'{self.force_magnitude(self.now_s()-self.active_start_s):.1f} N'
                )
            if self.minimum_obstacle_clearance_m is not None:
                text += (
                    f'\nMIN CLEARANCE '
                    f'{self.minimum_obstacle_clearance_m:.2f} m'
                )
            array.markers.append(
                self.text_marker(
                    7,
                    text,
                    (
                        self.position[0],
                        self.position[1],
                        self.position[2] + 7.0,
                    ),
                    1.5,
                    (1.0, 1.0, 1.0, 1.0),
                )
            )

        if self.peak_position is not None:
            array.markers.append(
                self.text_marker(
                    8,
                    f'PEAK NET DEVIATION '
                    f'{self.peak_net_deviation_m:.2f} m',
                    (
                        self.peak_position[0],
                        self.peak_position[1],
                        self.peak_position[2] + 3.0,
                    ),
                    1.6,
                    (1.0, 0.20, 0.10, 1.0),
                )
            )

        self.marker_pub.publish(array)

    def publish_status(self) -> None:
        payload = {
            'state': self.state,
            'active': self.state == 'active',
            'triggered_once': self.triggered_once,
            'scheduled': self.segment_armed and not self.triggered_once,
            'visual_phase': self.state,
            'gate_center': list(self.gate),
            'gate_radius_xy_m': self.gate_radius_xy,
            'gate_tolerance_z_m': self.gate_tolerance_z,
            'inside_gate': self.inside_gate(),
            'map_ready': self.map_ready,
            'trigger_position':
                list(self.trigger_position)
                if self.trigger_position is not None else None,
            'baseline_cross_track_m':
                self.baseline_cross_track_m,
            'current_cross_track_m':
                self.latest_cross_track_m,
            'current_net_deviation_m':
                self.current_net_deviation(),
            'peak_cross_track_m':
                self.peak_cross_track_m,
            'peak_net_deviation_m':
                self.peak_net_deviation_m,
            'target_net_deviation_m':
                self.target_net_deviation_m,
            'hard_stop_deviation_m':
                self.hard_stop_deviation_m,
            'target_reached':
                self.target_reached,
            'maximum_tilt_deg':
                self.maximum_observed_tilt_deg,
            'minimum_obstacle_clearance_m':
                self.minimum_obstacle_clearance_m,
            'recovery_time_s':
                self.recovery_time_s,
            'stop_reason':
                self.stop_reason,
            'safety_abort_reason':
                self.safety_abort_reason,
            'output_dir':
                str(self.output_dir)
                if self.output_dir is not None else None,
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)

    def save_results_if_ready(self) -> None:
        if self.result_saved:
            return
        mission_completed = (
            self.mission.get('state') == 'mission_completed'
            and int(self.mission.get('reached_index', -1)) == 4
        )
        if not mission_completed:
            return

        saturation_fraction = (
            self.rpm_saturated_samples / self.rpm_samples
            if self.rpm_samples else 0.0
        )
        criteria = {
            'disturbance_applied':
                self.applied_impulse_n_s > 0.5,
            'target_8m_reached':
                self.peak_net_deviation_m
                >= self.target_net_deviation_m,
            'overshoot_within_10_5m':
                self.peak_net_deviation_m
                <= self.hard_stop_deviation_m,
            'recovered':
                self.recovery_time_s is not None,
            'tilt_pass':
                self.maximum_observed_tilt_deg
                <= self.maximum_tilt_deg,
            'rpm_saturation_pass':
                saturation_fraction
                <= self.maximum_saturation_fraction,
            'clearance_pass':
                self.minimum_obstacle_clearance_m is not None
                and self.minimum_obstacle_clearance_m
                >= self.minimum_clearance_m,
            'no_safety_abort':
                self.safety_abort_reason is None,
            'mission_continued_to_P4':
                mission_completed,
        }

        output_dir = (
            self.results_root
            / datetime.now().strftime(
                'low_strong_wind_%Y%m%d_%H%M%S'
            )
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        self.output_dir = output_dir

        summary = {
            'experiment':
                'cq01_v8a3_low_city_adaptive_strong_wind',
            'final_state':
                self.state,
            'pass':
                all(criteria.values()),
            'failed_criteria': [
                key for key, value in criteria.items()
                if value is not True
            ],
            'criteria':
                criteria,
            'gate_center_m':
                list(self.gate),
            'trigger_position_m':
                list(self.trigger_position)
                if self.trigger_position is not None else None,
            'wind_direction_map':
                self.wind_direction.tolist(),
            'initial_force_n':
                self.initial_force_n,
            'force_ramp_n_per_s':
                self.force_ramp_n_per_s,
            'maximum_force_n':
                self.maximum_force_n,
            'maximum_active_time_s':
                self.maximum_active_time_s,
            'baseline_cross_track_m':
                self.baseline_cross_track_m,
            'peak_cross_track_error_m':
                self.peak_cross_track_m,
            'peak_net_wind_deviation_m':
                self.peak_net_deviation_m,
            'target_net_wind_deviation_m':
                self.target_net_deviation_m,
            'preferred_display_range_m': [
                self.target_net_deviation_m,
                self.preferred_max_deviation_m,
            ],
            'hard_stop_deviation_m':
                self.hard_stop_deviation_m,
            'recovery_time_s':
                self.recovery_time_s,
            'maximum_tilt_deg':
                self.maximum_observed_tilt_deg,
            'minimum_obstacle_clearance_m':
                self.minimum_obstacle_clearance_m,
            'rpm_saturation_fraction':
                saturation_fraction,
            'applied_impulse_n_s':
                self.applied_impulse_n_s,
            'target_reached':
                self.target_reached,
            'stop_reason':
                self.stop_reason,
            'safety_abort_reason':
                self.safety_abort_reason,
            'mission_completed_after_trigger':
                mission_completed,
            'sample_count':
                len(self.samples),
            'visualization': {
                'frozen_reference_path_topic':
                    '/cq01_v8a3/low/frozen_reference_path',
                'marker_topic':
                    '/cq01_v8a3/low/markers',
                'actual_local_trail_points':
                    len(self.local_trail),
                'gate_marker_alpha':
                    0.08,
            },
        }

        (output_dir / 'summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )

        if self.samples:
            with (output_dir / 'samples.csv').open(
                'w',
                newline='',
                encoding='utf-8',
            ) as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=list(self.samples[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.samples)

        self.result_saved = True
        self.state = 'results_saved'
        self.publish_status()
        self.get_logger().warning(
            'V8-A3 LOW STRONG-WIND RESULT: '
            f'pass={summary["pass"]}; '
            f'peak_net={self.peak_net_deviation_m:.3f}m; '
            f'failed={summary["failed_criteria"]}; '
            f'output={output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A3LowCityStrongWind()
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
