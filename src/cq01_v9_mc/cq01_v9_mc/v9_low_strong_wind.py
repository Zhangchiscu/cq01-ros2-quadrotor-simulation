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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


def quaternion_tilt_deg(q) -> float:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return math.degrees(max(abs(roll), abs(pitch)))


def point_segment_distance_xy(p: Point3, a: Point3, b: Point3):
    px, py, _ = p
    ax, ay, az = a
    bx, by, bz = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay), a
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    ref = (ax + t * dx, ay + t * dy, az + t * (bz - az))
    return math.hypot(px - ref[0], py - ref[1]), ref


class V9LowStrongWind(Node):
    """P1-P2 physical strong wind with honest 8-10 m event-scoped statistics."""

    def __init__(self) -> None:
        super().__init__('cq01_v9_low_strong_wind')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gate = (49.0, -93.0, 21.0)
        self.gate_radius = 2.8
        self.gate_z_tolerance = 4.0
        self.target_m = 8.0
        self.preferred_max_m = 10.0
        self.hard_max_m = 10.8
        self.max_active_s = 12.0

        # V9-MC.5.2 uses two completed-run calibrations:
        #
        # 1) cutoff at 8.0 m -> peak 12.358 m
        #    post-cutoff drift = 4.358 m
        #
        # 2) cutoff at 5.2 m -> peak 7.228 m
        #    post-cutoff drift = 2.028 m
        #
        # A 6.2 m cutoff predicts a peak between 8.23 and 10.56 m,
        # remaining inside the unchanged 8.0-10.8 m acceptance band.
        self.brake_entry_m = 6.2
        self.brake_min_coast_s = 0.8
        self.topup_force_n = 1.5
        self.topup_duration_s = 0.25
        self.maximum_topup_count = 2

        # Clearance belongs to the wind experiment only while external
        # force is commanded or the net wind-induced deviation is still
        # materially above the frozen-path baseline. Once the aircraft has
        # returned to within 1 m of the frozen path, later narrow-route
        # clearance is recorded separately and cannot be misclassified as
        # a wind safety failure.
        self.clearance_scope_net_m = 1.0
        self.topup_min_clearance_m = 3.5
        self.topup_min_net_m = 3.0
        self.max_tilt_deg = 44.0
        self.min_clearance_m = 2.5
        self.max_rpm = 10000.0
        self.results_root = Path(os.path.expanduser(
            '~/drone_task/your_drone_sim_ws/results/cq01_v9_mc/low_strong'
        ))

        self.wrench_pub = self.create_publisher(WrenchStamped, '/cq01_v9/low_strong/wrench_cmd', 20)
        self.active_pub = self.create_publisher(Bool, '/cq01_v9/low_strong/active', 10)
        self.status_pub = self.create_publisher(String, '/cq01_v9/low_strong/status', transient)
        self.marker_pub = self.create_publisher(MarkerArray, '/cq01_v9/low_strong/markers', 10)
        self.reference_pub = self.create_publisher(NavPath, '/cq01_v9/low_strong/frozen_path', transient)

        self.create_subscription(String, '/cq01_v9/route/status', self.route_callback, transient)
        self.create_subscription(Odometry, '/drone/odom', self.odom_callback, 50)
        self.create_subscription(NavPath, '/drone/planned_path_3d', self.path_callback, transient)
        self.create_subscription(Float32MultiArray, '/drone/motor_rpm', self.rpm_callback, 50)
        self.create_subscription(PointCloud2, '/drone/voxel_cloud', self.cloud_callback, transient)
        self.create_subscription(WrenchStamped, '/drone/external_wrench_applied', self.applied_callback, 50)

        self.route: Dict = {}
        self.max_reached = -1
        self.armed = False
        self.state = 'waiting_for_P1'
        self.position: Optional[Point3] = None
        self.latest_path: List[Point3] = []
        self.frozen_path: List[Point3] = []
        self.recent_error: Deque[Tuple[float, float]] = deque(maxlen=400)
        self.baseline: Optional[float] = None
        self.current_error: Optional[float] = None
        self.current_ref: Optional[Point3] = None
        self.peak_net = 0.0
        self.peak_error = 0.0
        self.peak_position: Optional[Point3] = None
        self.peak_ref: Optional[Point3] = None
        self.trigger_position: Optional[Point3] = None
        self.active_start: Optional[float] = None
        self.force_stop: Optional[float] = None

        self.braking_started = False
        self.braking_start: Optional[float] = None
        self.topup_until: Optional[float] = None
        self.topup_count = 0
        self.recent_net: Deque[Tuple[float, float]] = deque(maxlen=250)
        self.commanded_force_n = 0.0

        self.recovery_candidate: Optional[float] = None
        self.recovery_time: Optional[float] = None
        self.maximum_tilt = 0.0
        self.latest_tilt = 0.0
        self.latest_rpm: List[float] = []
        self.rpm_samples = 0
        self.rpm_saturated = 0
        self.cloud: Optional[np.ndarray] = None
        self.current_clearance: Optional[float] = None
        self.event_min_clearance: Optional[float] = None
        self.recovery_route_min_clearance: Optional[float] = None
        self.peak_clearance: Optional[float] = None
        self.last_clearance_s = -1e9
        self.impulse = 0.0
        self.last_applied_s: Optional[float] = None
        self.stop_reason: Optional[str] = None
        self.safety_abort: Optional[str] = None
        self.samples: List[Dict] = []
        self.trail: List[Point3] = []
        self.last_trail_s = -1e9
        self.output_dir: Optional[Path] = None
        self.result_saved = False

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.marker_timer = self.create_timer(0.10, self.publish_markers)
        self.status_timer = self.create_timer(0.50, self.publish_status)
        self.get_logger().warning(
            'V9 LOW STRONG WIND ready: -Y physical force, closed-loop 8-10 m, max 12 s.'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def route_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.route = data
        if 'reached_index' in data:
            try:
                self.max_reached = max(self.max_reached, int(data['reached_index']))
            except Exception:
                pass
        current = int(data.get('current_index', -1))
        if bool(data.get('active', False)) and current == 2 and self.max_reached >= 1 and self.max_reached < 2 and not self.armed:
            self.armed = True
            self.state = 'armed_P1_P2'
            self.get_logger().warning('V9 P1->P2 STRONG WIND ARMED.')

    def path_callback(self, msg: NavPath):
        points = [(float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)) for p in msg.poses]
        if len(points) >= 2:
            self.latest_path = points

    def cloud_callback(self, msg: PointCloud2):
        try:
            arr = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32).reshape((-1, 3))
        except Exception:
            arr = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32).reshape((-1, 3))
        mask = ((arr[:, 0] > 25.0) & (arr[:, 0] < 85.0) &
                (arr[:, 1] > -125.0) & (arr[:, 1] < -55.0) &
                (arr[:, 2] > 0.0) & (arr[:, 2] < 60.0))
        self.cloud = arr[mask]

    def rpm_callback(self, msg: Float32MultiArray):
        self.latest_rpm = [float(x) for x in msg.data]

    def applied_callback(self, msg: WrenchStamped):
        now = self.now_s()
        if self.last_applied_s is None:
            self.last_applied_s = now
            return
        dt = max(0.0, min(0.1, now - self.last_applied_s))
        self.last_applied_s = now
        if self.state not in ('active', 'recovery'):
            return
        f = msg.wrench.force
        self.impulse += math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z) * dt

    def nearest(self, position: Point3):
        path = self.frozen_path if self.frozen_path else self.latest_path
        if len(path) < 2:
            return None, None
        best, best_ref = float('inf'), None
        for a, b in zip(path[:-1], path[1:]):
            distance, ref = point_segment_distance_xy(position, a, b)
            if distance < best:
                best, best_ref = distance, ref
        return best, best_ref

    def odom_callback(self, msg: Odometry):
        now = self.now_s()
        p = msg.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))
        self.latest_tilt = quaternion_tilt_deg(msg.pose.pose.orientation)
        if self.state in ('active', 'recovery'):
            self.maximum_tilt = max(self.maximum_tilt, self.latest_tilt)
        error, ref = self.nearest(self.position)
        self.current_error, self.current_ref = error, ref
        if error is not None and self.state not in ('active', 'recovery'):
            self.recent_error.append((now, error))
        # Correct scope: never update peak after the event has recovered/failed.
        if self.state in ('active', 'recovery') and error is not None and self.baseline is not None:
            net = max(0.0, error - self.baseline)
            self.recent_net.append((now, net))

            clearance = self.update_clearance(now)
            wind_evidence_active = (
                self.commanded_force_n > 0.05
                or net >= self.clearance_scope_net_m
            )

            if clearance is not None:
                if wind_evidence_active:
                    self.event_min_clearance = (
                        clearance
                        if self.event_min_clearance is None
                        else min(self.event_min_clearance, clearance)
                    )
                else:
                    self.recovery_route_min_clearance = (
                        clearance
                        if self.recovery_route_min_clearance is None
                        else min(self.recovery_route_min_clearance, clearance)
                    )

            if net >= self.peak_net:
                self.peak_net, self.peak_error = net, error
                self.peak_position, self.peak_ref = self.position, ref
                self.peak_clearance = clearance

            if now - self.last_trail_s >= 0.08:
                self.trail.append(self.position)
                self.last_trail_s = now

            saturated = bool(
                self.latest_rpm
                and max(abs(x) for x in self.latest_rpm)
                >= 0.98 * self.max_rpm
            )
            self.rpm_samples += 1
            if saturated:
                self.rpm_saturated += 1

            self.samples.append({
                'time_s': now,
                'x_m': self.position[0],
                'y_m': self.position[1],
                'z_m': self.position[2],
                'cross_track_m': error,
                'net_deviation_m': net,
                'tilt_deg': self.latest_tilt,
                'instant_clearance_m': (
                    clearance if clearance is not None else float('nan')
                ),
                'wind_evidence_active': wind_evidence_active,
                'commanded_force_n': self.commanded_force_n,
                'state': self.state,
            })

    def update_clearance(self, now):
        if (
            self.position is None
            or self.cloud is None
            or len(self.cloud) == 0
            or now - self.last_clearance_s < 0.10
        ):
            return self.current_clearance

        self.last_clearance_s = now
        distances = np.linalg.norm(
            self.cloud
            - np.asarray(self.position, dtype=np.float32),
            axis=1,
        )
        self.current_clearance = float(np.min(distances))
        return self.current_clearance

    def inside_gate(self):
        if self.position is None:
            return False
        dx, dy = self.position[0] - self.gate[0], self.position[1] - self.gate[1]
        return math.hypot(dx, dy) <= self.gate_radius and abs(self.position[2] - self.gate[2]) <= self.gate_z_tolerance

    def baseline_error(self, now):
        values = [v for t, v in self.recent_error if now - t <= 1.5]
        return float(np.median(np.asarray(values))) if values else float(self.current_error or 0.0)

    def start(self, now):
        if len(self.latest_path) < 2 or self.cloud is None:
            return
        self.frozen_path = list(self.latest_path)
        self.baseline = self.baseline_error(now)
        self.peak_error = self.baseline
        self.trigger_position = self.position
        self.active_start = now
        self.braking_started = False
        self.braking_start = None
        self.topup_until = None
        self.topup_count = 0
        self.recent_net.clear()
        self.commanded_force_n = 0.0
        self.current_clearance = None
        self.event_min_clearance = None
        self.recovery_route_min_clearance = None
        self.peak_clearance = None
        self.last_clearance_s = -1e9
        self.state = 'active'
        path = NavPath()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        from geometry_msgs.msg import PoseStamped
        for x, y, z in self.frozen_path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.reference_pub.publish(path)
        self.get_logger().warning(f'V9 LOW STRONG WIND ACTIVE: baseline={self.baseline:.3f} m.')

    def force_magnitude(self, elapsed, net):
        raw = min(14.0, 5.0 + 1.5 * elapsed)
        ramp = min(1.0, elapsed / 0.35)
        if net >= 4.8:
            raw *= 0.65
        return raw * ramp

    def net_growth_rate(self, now):
        samples = [
            (timestamp, value)
            for timestamp, value in self.recent_net
            if now - timestamp <= 0.50
        ]
        if len(samples) < 2:
            return 0.0
        dt = samples[-1][0] - samples[0][0]
        if dt <= 1.0e-3:
            return 0.0
        return (samples[-1][1] - samples[0][1]) / dt

    def publish_force(self, magnitude, active):
        self.commanded_force_n = float(magnitude) if active else 0.0
        msg = WrenchStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.y = -float(magnitude)
        self.wrench_pub.publish(msg)
        flag = Bool(); flag.data = bool(active); self.active_pub.publish(flag)

    def stop(self, reason, safety=False):
        if self.state != 'active':
            return
        self.publish_force(0.0, False)
        self.force_stop = self.now_s()
        self.stop_reason = reason
        if safety:
            self.safety_abort = reason
        self.state = 'recovery'
        self.get_logger().warning(f'V9 LOW STRONG WIND OFF: {reason}; peak={self.peak_net:.3f} m.')

    def net(self):
        if self.current_error is None or self.baseline is None:
            return 0.0
        return max(0.0, self.current_error - self.baseline)

    def timer_callback(self):
        now = self.now_s()
        if self.armed and self.state.startswith('armed') and self.inside_gate():
            self.start(now)
        if self.state == 'active':
            elapsed = now - (self.active_start or now)
            net = self.net()

            if net >= self.target_m:
                self.stop('target_8m_reached_after_early_braking')
            elif self.latest_tilt >= self.max_tilt_deg:
                self.stop('maximum_tilt_exceeded', True)
            elif (
                self.current_clearance is not None
                and self.current_clearance < self.min_clearance_m
                and (
                    self.commanded_force_n > 0.05
                    or net >= self.clearance_scope_net_m
                )
            ):
                self.stop(
                    'wind_evidence_minimum_clearance_violated',
                    True,
                )
            elif elapsed >= self.max_active_s:
                self.stop('maximum_active_time_without_8m', True)
            else:
                fraction = self.rpm_saturated / max(1, self.rpm_samples)
                if self.rpm_samples >= 20 and fraction > 0.05:
                    self.stop('rpm_saturation_exceeded', True)
                elif not self.braking_started:
                    if net >= self.brake_entry_m:
                        self.braking_started = True
                        self.braking_start = now
                        self.publish_force(0.0, False)
                        self.get_logger().warning(
                            'V9-MC.3 LOW STRONG EARLY BRAKING: '
                            f'net={net:.3f} m; main force removed.'
                        )
                    else:
                        self.publish_force(
                            self.force_magnitude(elapsed, net),
                            True,
                        )
                else:
                    if (
                        self.topup_until is not None
                        and now < self.topup_until
                    ):
                        self.publish_force(
                            self.topup_force_n,
                            True,
                        )
                    else:
                        if (
                            self.topup_until is not None
                            and now >= self.topup_until
                        ):
                            self.topup_until = None
                            self.braking_start = now

                        growth = self.net_growth_rate(now)
                        coast_time = (
                            now - self.braking_start
                            if self.braking_start is not None
                            else 0.0
                        )
                        should_topup = (
                            coast_time >= self.brake_min_coast_s
                            and self.peak_net < self.target_m
                            and net >= self.topup_min_net_m
                            and growth <= 0.05
                            and self.current_clearance is not None
                            and self.current_clearance
                            >= self.topup_min_clearance_m
                            and self.topup_count
                            < self.maximum_topup_count
                        )

                        if should_topup:
                            self.topup_count += 1
                            self.topup_until = (
                                now + self.topup_duration_s
                            )
                            self.publish_force(
                                self.topup_force_n,
                                True,
                            )
                            self.get_logger().warning(
                                'V9-MC.3 LOW STRONG TOP-UP '
                                f'{self.topup_count}/'
                                f'{self.maximum_topup_count}: '
                                f'net={net:.3f} m, '
                                f'growth={growth:.3f} m/s.'
                            )
                        else:
                            self.publish_force(0.0, False)
        elif self.state == 'recovery':
            self.publish_force(0.0, False)
            if self.peak_net > self.hard_max_m and self.safety_abort is None:
                self.safety_abort = 'overshoot_above_10_8m'
            if self.current_error is not None and self.baseline is not None and self.current_error <= self.baseline + 1.2:
                if self.recovery_candidate is None:
                    self.recovery_candidate = now
                elif now - self.recovery_candidate >= 1.0:
                    self.recovery_time = self.recovery_candidate - (self.force_stop or self.recovery_candidate)
                    self.state = 'recovered'
                    self.save_result()
                    self.get_logger().warning(f'V9 LOW STRONG RECOVERED: peak={self.peak_net:.3f} m.')
            else:
                self.recovery_candidate = None
            if self.force_stop and now - self.force_stop > 30.0:
                self.safety_abort = self.safety_abort or 'recovery_timeout'
                self.state = 'failed'
                self.save_result()
        else:
            self.publish_force(0.0, False)

    def save_result(self):
        if self.result_saved:
            return
        saturation = self.rpm_saturated / max(1, self.rpm_samples)
        criteria = {
            'disturbance_applied': self.impulse > 1.0,
            'peak_at_least_8m': self.peak_net >= 8.0,
            'peak_at_most_10_8m': self.peak_net <= 10.8,
            'recovered': self.recovery_time is not None,
            'tilt_pass': self.maximum_tilt <= 44.0,
            'rpm_pass': saturation <= 0.05,
            'clearance_pass':
                self.event_min_clearance is not None
                and self.event_min_clearance >= 2.5,
            'no_safety_abort': self.safety_abort is None,
        }
        out = self.results_root / datetime.now().strftime('low_strong_%Y%m%d_%H%M%S')
        out.mkdir(parents=True, exist_ok=True)
        self.output_dir = out
        summary = {
            'experiment': 'cq01_v9_low_city_8_10m_strong_wind',
            'state': self.state,
            'pass': all(criteria.values()),
            'failed_criteria': [k for k, v in criteria.items() if v is not True],
            'criteria': criteria,
            'trigger_position_m': list(self.trigger_position) if self.trigger_position else None,
            'baseline_cross_track_m': self.baseline,
            'peak_cross_track_m': self.peak_error,
            'peak_net_wind_deviation_m': self.peak_net,
            'recovery_time_s': self.recovery_time,
            'maximum_tilt_deg': self.maximum_tilt,
            'minimum_clearance_m': self.event_min_clearance,
            'minimum_wind_evidence_clearance_m':
                self.event_min_clearance,
            'minimum_recovery_route_clearance_m':
                self.recovery_route_min_clearance,
            'peak_position_clearance_m':
                self.peak_clearance,
            'clearance_evaluation_scope':
                'commanded_force_or_net_deviation_at_least_1m',
            'rpm_saturation_fraction': saturation,
            'applied_impulse_n_s': self.impulse,
            'stop_reason': self.stop_reason,
            'safety_abort_reason': self.safety_abort,
            'statistics_scope': 'trigger_to_recovery_only',
            'brake_entry_m': self.brake_entry_m,
            'braking_started': self.braking_started,
            'topup_count': self.topup_count,
            'clearance_scope_net_m':
                self.clearance_scope_net_m,
            'topup_min_clearance_m':
                self.topup_min_clearance_m,
            'topup_min_net_m':
                self.topup_min_net_m,
            'topup_force_n': self.topup_force_n,
            'topup_duration_s': self.topup_duration_s,
            'calibration_basis': {
                'previous_stop_threshold_m': 8.0,
                'old_cutoff_m': 8.0,
                'old_peak_m': 12.358455869879315,
                'old_post_cutoff_drift_m':
                    4.358455869879315,
                'mc3_cutoff_m': 5.2,
                'mc3_peak_m': 7.228224116164741,
                'mc3_post_cutoff_drift_m':
                    2.028224116164741,
                'selected_cutoff_m': 6.2,
                'predicted_peak_interval_m':
                    [8.22822411616474, 10.558455869879315],
            },
        }
        (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        if self.samples:
            with (out / 'samples.csv').open('w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
                writer.writeheader(); writer.writerows(self.samples)
        self.result_saved = True
        self.publish_status()
        self.get_logger().warning(f'V9 LOW STRONG RESULT pass={summary["pass"]}; peak={self.peak_net:.3f} m; output={out}')

    def header(self):
        from std_msgs.msg import Header
        h = Header(); h.frame_id = 'world'; h.stamp = self.get_clock().now().to_msg(); return h

    def publish_markers(self):
        arr = MarkerArray(); clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
        if len(self.trail) >= 2:
            m = Marker(); m.header = self.header(); m.ns = 'v9_low_strong'; m.id = 1; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.75; m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.25, 0.05, 1.0
            for x, y, z in self.trail:
                p = Point(); p.x, p.y, p.z = x, y, z; m.points.append(p)
            arr.markers.append(m)
        if self.state == 'active' and self.position is not None:
            m = Marker(); m.header = self.header(); m.ns = 'v9_low_strong'; m.id = 2; m.type = Marker.ARROW; m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.7, 1.2, 1.2; m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.65, 1.0, 1.0
            force = self.commanded_force_n
            a, b = Point(), Point(); a.x, a.y, a.z = self.position; b.x, b.y, b.z = self.position[0], self.position[1] - 1.2 * force, self.position[2]
            m.points = [a, b]; arr.markers.append(m)
        if self.peak_position and self.peak_ref:
            line = Marker(); line.header = self.header(); line.ns = 'v9_low_strong'; line.id = 3; line.type = Marker.LINE_LIST; line.action = Marker.ADD
            line.scale.x = 0.8; line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.05, 0.05, 1.0
            a, b = Point(), Point(); a.x, a.y, a.z = self.peak_ref; b.x, b.y, b.z = self.peak_position; line.points = [a, b]; arr.markers.append(line)
            sphere = Marker(); sphere.header = self.header(); sphere.ns = 'v9_low_strong'; sphere.id = 4; sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = self.peak_position; sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.8; sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.05, 0.05, 1.0; arr.markers.append(sphere)
        self.marker_pub.publish(arr)

    def publish_status(self):
        payload = {
            'state': self.state, 'armed': self.armed, 'active': self.state == 'active', 'triggered': self.active_start is not None,
            'baseline_cross_track_m': self.baseline, 'current_net_deviation_m': self.net(), 'peak_net_deviation_m': self.peak_net,
            'recovery_time_s': self.recovery_time,
            'maximum_tilt_deg': self.maximum_tilt,
            'minimum_clearance_m': self.event_min_clearance,
            'minimum_wind_evidence_clearance_m':
                self.event_min_clearance,
            'minimum_recovery_route_clearance_m':
                self.recovery_route_min_clearance,
            'current_clearance_m': self.current_clearance,
            'peak_position_clearance_m':
                self.peak_clearance,
            'stop_reason': self.stop_reason, 'safety_abort_reason': self.safety_abort,
            'braking_started': self.braking_started,
            'brake_entry_m': self.brake_entry_m,
            'topup_count': self.topup_count,
            'commanded_force_n': self.commanded_force_n,
            'net_growth_rate_mps': self.net_growth_rate(self.now_s()),
            'output_dir': str(self.output_dir) if self.output_dir else None,
            'pass_result': None if not self.result_saved else (self.safety_abort is None and 8.0 <= self.peak_net <= 10.8 and self.recovery_time is not None),
        }
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True); self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = V9LowStrongWind()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
