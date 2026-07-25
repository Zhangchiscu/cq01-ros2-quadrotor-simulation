#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


Point3 = Tuple[float, float, float]


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class HighAltitudeWindManager(Node):
    """Strong wind while climbing from P12 to P13.

    The aircraft remains in normal flight. At a spatial gate near 160 m
    altitude, a stronger crosswind is applied perpendicular to the active A*
    path. The same P13 mission target is then republished so the planner
    generates a new path from the disturbed odometry position.
    """

    def __init__(self) -> None:
        super().__init__('cq01_high_altitude_wind_manager')

        self.declare_parameter(
            'mission_status_topic',
            '/cq01_v34h/mission_status',
        )
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('raw_goal_topic', '/drone/goal_raw')
        # V8-B interface split: subscribe to the authoritative raw-goal
        # output, but publish the H14 replan request to an isolated input
        # of the single raw-goal mux. Physics and H14 criteria are unchanged.
        self.declare_parameter(
            'replan_goal_topic',
            '/drone/goal_raw',
        )
        self.declare_parameter(
            'path_topic',
            '/drone/planned_path_3d',
        )
        self.declare_parameter(
            'planner_status_topic',
            '/drone/planner_3d/status',
        )
        self.declare_parameter('rpm_topic', '/drone/motor_rpm')
        self.declare_parameter(
            'applied_wrench_topic',
            '/drone/external_wrench_applied',
        )
        self.declare_parameter(
            'command_wrench_topic',
            '/drone/external_wrench_high_altitude',
        )
        self.declare_parameter(
            'active_topic',
            '/drone/high_altitude_wind_active',
        )
        self.declare_parameter(
            'status_topic',
            '/cq01_v34h/high_altitude_wind_status',
        )
        self.declare_parameter(
            'marker_topic',
            '/cq01_v34h/high_altitude_wind_markers',
        )

        # P12 -> P13 high-altitude climb gate.
        self.declare_parameter('arm_after_reached_index', 12)
        self.declare_parameter('gate_x_m', 334.0)
        self.declare_parameter('gate_y_m', -31.0)
        self.declare_parameter('gate_z_m', 162.0)
        self.declare_parameter('gate_radius_xy_m', 13.0)
        self.declare_parameter('gate_tolerance_z_m', 22.0)
        self.declare_parameter('countdown_s', 1.2)

        # Stronger than the 2.5 N low-city crosswind, but still bounded.
        # H14 does not increase peak force because H13 already reached
        # 43.63 deg tilt against a 44 deg safety limit. Instead, it uses a
        # longer, tilt-limited gust until a real 5 m lateral offset is reached.
        self.declare_parameter('force_magnitude_n', 2.85)
        self.declare_parameter('duration_s', 4.2)
        self.declare_parameter('minimum_gust_duration_s', 1.2)
        self.declare_parameter('target_deviation_m', 5.0)
        self.declare_parameter('ramp_time_s', 0.30)
        self.declare_parameter('yaw_torque_nm', 0.020)
        self.declare_parameter('soft_tilt_start_deg', 39.0)
        self.declare_parameter('soft_tilt_end_deg', 43.4)
        self.declare_parameter('minimum_force_scale', 0.30)
        self.declare_parameter('gust_frequency_hz', 0.9)
        self.declare_parameter('lateral_sign', -1.0)

        self.declare_parameter('minimum_replan_deviation_m', 1.50)
        self.declare_parameter('replan_timeout_s', 0.80)
        self.declare_parameter('tracking_tolerance_m', 1.10)
        self.declare_parameter('tracking_hold_s', 0.80)
        self.declare_parameter('evidence_hold_s', 5.0)
        self.declare_parameter('maximum_retracking_wait_s', 20.0)

        self.declare_parameter('safety_tilt_deg', 44.0)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter('safety_rpm_ratio', 0.985)
        self.declare_parameter('minimum_visible_branch_m', 1.0)

        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('marker_rate_hz', 10.0)
        self.declare_parameter('visual_force_scale_m_per_n', 4.0)
        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_high_altitude_wind'
            ),
        )

        self.arm_after_reached_index = int(
            self.get_parameter('arm_after_reached_index').value
        )
        self.gate = (
            float(self.get_parameter('gate_x_m').value),
            float(self.get_parameter('gate_y_m').value),
            float(self.get_parameter('gate_z_m').value),
        )
        self.gate_radius_xy_m = max(
            2.0,
            float(self.get_parameter('gate_radius_xy_m').value),
        )
        self.gate_tolerance_z_m = max(
            2.0,
            float(
                self.get_parameter('gate_tolerance_z_m').value
            ),
        )
        self.countdown_s = max(
            0.0,
            float(self.get_parameter('countdown_s').value),
        )

        self.force_magnitude_n = max(
            0.1,
            float(self.get_parameter('force_magnitude_n').value),
        )
        self.duration_s = max(
            0.2,
            float(self.get_parameter('duration_s').value),
        )
        self.minimum_gust_duration_s = max(
            0.0,
            min(
                self.duration_s,
                float(
                    self.get_parameter(
                        'minimum_gust_duration_s'
                    ).value
                ),
            ),
        )
        self.target_deviation_m = max(
            1.0,
            float(
                self.get_parameter('target_deviation_m').value
            ),
        )
        self.ramp_time_s = max(
            0.0,
            min(
                float(self.get_parameter('ramp_time_s').value),
                self.duration_s / 2.0,
            ),
        )
        self.yaw_torque_nm = float(
            self.get_parameter('yaw_torque_nm').value
        )
        self.soft_tilt_start_deg = float(
            self.get_parameter('soft_tilt_start_deg').value
        )
        self.soft_tilt_end_deg = max(
            self.soft_tilt_start_deg + 0.5,
            float(
                self.get_parameter('soft_tilt_end_deg').value
            ),
        )
        self.minimum_force_scale = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'minimum_force_scale'
                    ).value
                ),
            ),
        )
        self.gust_frequency_hz = max(
            0.0,
            float(self.get_parameter('gust_frequency_hz').value),
        )
        self.lateral_sign = (
            1.0
            if float(self.get_parameter('lateral_sign').value) >= 0.0
            else -1.0
        )

        self.minimum_replan_deviation_m = max(
            0.2,
            float(
                self.get_parameter(
                    'minimum_replan_deviation_m'
                ).value
            ),
        )
        self.replan_timeout_s = max(
            0.2,
            float(self.get_parameter('replan_timeout_s').value),
        )
        self.tracking_tolerance_m = max(
            0.2,
            float(
                self.get_parameter('tracking_tolerance_m').value
            ),
        )
        self.tracking_hold_s = max(
            0.1,
            float(self.get_parameter('tracking_hold_s').value),
        )
        self.evidence_hold_s = max(
            2.0,
            float(self.get_parameter('evidence_hold_s').value),
        )
        self.maximum_retracking_wait_s = max(
            5.0,
            float(
                self.get_parameter(
                    'maximum_retracking_wait_s'
                ).value
            ),
        )

        self.safety_tilt_deg = float(
            self.get_parameter('safety_tilt_deg').value
        )
        self.maximum_rpm = float(
            self.get_parameter('maximum_rpm').value
        )
        self.safety_rpm_ratio = float(
            self.get_parameter('safety_rpm_ratio').value
        )
        self.minimum_visible_branch_m = max(
            0.2,
            float(
                self.get_parameter(
                    'minimum_visible_branch_m'
                ).value
            ),
        )
        self.visual_force_scale = max(
            1.0,
            float(
                self.get_parameter(
                    'visual_force_scale_m_per_n'
                ).value
            ),
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )

        publish_rate_hz = max(
            20.0,
            float(self.get_parameter('publish_rate_hz').value),
        )
        marker_rate_hz = max(
            1.0,
            float(self.get_parameter('marker_rate_hz').value),
        )
        self.period_s = 1.0 / publish_rate_hz
        self.marker_period_s = 1.0 / marker_rate_hz

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
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self.replan_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('replan_goal_topic').value),
            10,
        )

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
            PoseStamped,
            str(self.get_parameter('raw_goal_topic').value),
            self.raw_goal_callback,
            20,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('path_topic').value),
            self.path_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('planner_status_topic').value),
            self.planner_status_callback,
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

        self.state = 'waiting_for_high_altitude'
        self.armed = False
        self.triggered = False
        self.latest_position: Optional[Point3] = None
        self.latest_raw_goal: Optional[PoseStamped] = None
        self.current_path: List[Point3] = []
        self.old_path: List[Point3] = []
        self.new_path: List[Point3] = []
        self.actual_trail: List[Point3] = []
        self.path_message_count = 0
        self.path_count_at_replan = 0

        self.crosswind_unit: Point3 = (0.0, 1.0, 0.0)
        self.latest_rpy_deg = (0.0, 0.0, 0.0)
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.latest_applied_force = [0.0, 0.0, 0.0]

        self.scheduled_time: Optional[float] = None
        self.gust_start_time: Optional[float] = None
        self.gust_end_time: Optional[float] = None
        self.replan_request_time: Optional[float] = None
        self.new_path_time: Optional[float] = None
        self.replan_latency_s: Optional[float] = None
        self.retracking_candidate_time: Optional[float] = None
        self.retracking_time_s: Optional[float] = None
        self.evidence_end_time: Optional[float] = None
        self.retracking_deadline: Optional[float] = None
        self.target_deviation_reached = False
        self.gust_stop_reason = ''
        self.actual_gust_duration_s: Optional[float] = None
        self.tracking_timeout = False

        self.current_old_path_deviation_m = 0.0
        self.peak_old_path_deviation_m = 0.0
        self.current_new_path_deviation_m: Optional[float] = None
        self.new_path_branch_m = 0.0
        self.replan_requested = False
        self.new_path_received = False
        self.maximum_tilt_observed_deg = 0.0
        self.maximum_rpm_observed = 0.0
        self.peak_applied_force_n = 0.0
        self.safety_abort = False
        self.safety_abort_reason = ''

        self.last_marker_time = -1.0e9
        self.last_status_time = -1.0e9
        self.last_status_state: Optional[str] = None
        self.last_trail_time = -1.0e9
        self.samples: List[Dict] = []
        self.saved_output_dir: Optional[Path] = None

        self.timer = self.create_timer(
            self.period_s,
            self.timer_callback,
        )
        self.publish_status(
            'waiting_for_high_altitude',
            force=True,
        )
        self.get_logger().info(
            'High-altitude in-flight wind manager ready: '
            f'gate={self.gate}, '
            f'force={self.force_magnitude_n:.2f} N x '
            f'{self.duration_s:.2f} s.'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def copy_pose(self, source: PoseStamped) -> PoseStamped:
        target = PoseStamped()
        target.header = source.header
        target.pose = source.pose
        return target

    def publish_status(
        self,
        state: str,
        force: bool = False,
        **extra,
    ) -> None:
        now = self.now_s()
        if (
            not force
            and state == self.last_status_state
            and now - self.last_status_time < 1.0
        ):
            return

        payload = {
            'state': state,
            'armed': self.armed,
            'triggered': self.triggered,
            'altitude_m':
                self.latest_position[2]
                if self.latest_position is not None else None,
            'current_old_path_deviation_m':
                self.current_old_path_deviation_m,
            'peak_old_path_deviation_m':
                self.peak_old_path_deviation_m,
            'current_new_path_deviation_m':
                self.current_new_path_deviation_m,
            'new_path_branch_m': self.new_path_branch_m,
            'target_deviation_m': self.target_deviation_m,
            'target_deviation_reached':
                self.target_deviation_reached,
            'gust_stop_reason': self.gust_stop_reason,
            'actual_gust_duration_s':
                self.actual_gust_duration_s,
            'tracking_timeout': self.tracking_timeout,
            'replan_requested': self.replan_requested,
            'new_path_received': self.new_path_received,
            'replan_latency_s': self.replan_latency_s,
            'maximum_tilt_deg':
                self.maximum_tilt_observed_deg,
            'maximum_rpm': self.maximum_rpm_observed,
            'safety_abort': self.safety_abort,
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
        self.last_status_time = now

    def mission_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(payload.get('state', ''))
        reached_index = int(payload.get('reached_index', -1))
        current_index = int(payload.get('current_index', -1))

        if (
            not self.triggered
            and not self.armed
            and (
                reached_index >= self.arm_after_reached_index
                or current_index >= self.arm_after_reached_index + 1
            )
        ):
            self.armed = True
            self.state = 'high_altitude_armed'
            self.publish_status(
                'high_altitude_armed',
                force=True,
                gate=list(self.gate),
            )
            self.get_logger().warning(
                'P12 reached: high-altitude strong-wind stage ARMED.'
            )

        if (
            self.triggered
            and state in (
                'planning_failed',
                'mission_failed',
                'stalled',
            )
            and self.saved_output_dir is None
        ):
            self.save_results(state)

    def raw_goal_callback(self, message: PoseStamped) -> None:
        self.latest_raw_goal = self.copy_pose(message)

    def path_callback(self, message: NavPath) -> None:
        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]
        if len(points) < 2:
            return

        self.current_path = points
        self.path_message_count += 1

        if (
            self.replan_requested
            and not self.new_path_received
            and self.path_message_count > self.path_count_at_replan
        ):
            self.new_path = list(points)
            self.new_path_received = True
            self.new_path_time = self.now_s()
            if self.replan_request_time is not None:
                self.replan_latency_s = (
                    self.new_path_time - self.replan_request_time
                )
            self.new_path_branch_m = self.calculate_path_branch()
            self.state = 'high_altitude_new_path_ready'
            self.retracking_deadline = (
                self.now_s() + self.maximum_retracking_wait_s
            )
            self.evidence_end_time = None
            self.publish_status(
                'high_altitude_new_path_ready',
                force=True,
                replan_latency_s=self.replan_latency_s,
                branch_m=self.new_path_branch_m,
            )
            self.get_logger().warning(
                'HIGH-ALTITUDE A* PATH READY: '
                f'latency={1000.0 * (self.replan_latency_s or 0.0):.1f} ms, '
                f'branch={self.new_path_branch_m:.2f} m'
            )

    def planner_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            self.replan_requested
            and str(payload.get('state', '')) == 'planning_failed'
        ):
            self.state = 'high_altitude_replan_failed'
            self.publish_status(
                'high_altitude_replan_failed',
                force=True,
                planner_status=payload,
            )

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
        force_norm = math.sqrt(
            sum(value * value for value in self.latest_applied_force)
        )
        self.peak_applied_force_n = max(
            self.peak_applied_force_n,
            force_norm,
        )

    def inside_gate(self) -> bool:
        if self.latest_position is None:
            return False
        dx = self.latest_position[0] - self.gate[0]
        dy = self.latest_position[1] - self.gate[1]
        dz = abs(self.latest_position[2] - self.gate[2])
        return (
            math.hypot(dx, dy) <= self.gate_radius_xy_m
            and dz <= self.gate_tolerance_z_m
        )

    def nearest_path_distance(
        self,
        position: Point3,
        path: List[Point3],
    ) -> Tuple[Optional[float], Optional[Point3]]:
        if not path:
            return None, None
        nearest = min(
            path,
            key=lambda point: math.dist(position, point),
        )
        return math.dist(position, nearest), nearest

    def nearest_segment_direction(
        self,
        position: Point3,
        path: List[Point3],
    ) -> Point3:
        if len(path) < 2:
            return (1.0, 0.0, 0.0)

        best_distance = float('inf')
        best_direction = (1.0, 0.0, 0.0)
        for first, second in zip(path[:-1], path[1:]):
            vx = second[0] - first[0]
            vy = second[1] - first[1]
            vz = second[2] - first[2]
            length_sq = vx * vx + vy * vy + vz * vz
            if length_sq <= 1.0e-9:
                continue

            wx = position[0] - first[0]
            wy = position[1] - first[1]
            wz = position[2] - first[2]
            t = max(
                0.0,
                min(
                    1.0,
                    (wx * vx + wy * vy + wz * vz) / length_sq,
                ),
            )
            projection = (
                first[0] + t * vx,
                first[1] + t * vy,
                first[2] + t * vz,
            )
            distance = math.dist(position, projection)
            if distance < best_distance:
                best_distance = distance
                horizontal = math.hypot(vx, vy)
                if horizontal > 1.0e-6:
                    best_direction = (
                        vx / horizontal,
                        vy / horizontal,
                        0.0,
                    )
        return best_direction

    def calculate_crosswind_unit(self) -> Point3:
        if self.latest_position is None:
            return (0.0, self.lateral_sign, 0.0)
        tangent = self.nearest_segment_direction(
            self.latest_position,
            self.current_path,
        )
        return (
            -self.lateral_sign * tangent[1],
            self.lateral_sign * tangent[0],
            0.0,
        )

    def calculate_path_branch(self) -> float:
        if not self.old_path or not self.new_path:
            return 0.0
        sample = self.new_path[: min(16, len(self.new_path))]
        values: List[float] = []
        for point in sample:
            distance, _ = self.nearest_path_distance(
                point,
                self.old_path,
            )
            if distance is not None:
                values.append(distance)
        return max(values, default=0.0)

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.latest_position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        q = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        self.latest_rpy_deg = (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )

        self.maximum_tilt_observed_deg = max(
            self.maximum_tilt_observed_deg,
            abs(self.latest_rpy_deg[0]),
            abs(self.latest_rpy_deg[1]),
        )
        self.maximum_rpm_observed = max(
            self.maximum_rpm_observed,
            max(self.latest_rpm),
        )

        if self.old_path:
            distance, _ = self.nearest_path_distance(
                self.latest_position,
                self.old_path,
            )
            self.current_old_path_deviation_m = (
                distance or 0.0
            )
            self.peak_old_path_deviation_m = max(
                self.peak_old_path_deviation_m,
                self.current_old_path_deviation_m,
            )

        if self.new_path:
            distance, _ = self.nearest_path_distance(
                self.latest_position,
                self.new_path,
            )
            self.current_new_path_deviation_m = distance

        now = self.now_s()
        if (
            self.triggered
            and now - self.last_trail_time >= 0.08
        ):
            self.actual_trail.append(self.latest_position)
            self.last_trail_time = now

        if self.gust_start_time is not None:
            force_norm = math.sqrt(
                sum(value * value for value in self.latest_applied_force)
            )
            self.samples.append({
                'time_s': now - self.gust_start_time,
                'state': self.state,
                'x_m': self.latest_position[0],
                'y_m': self.latest_position[1],
                'z_m': self.latest_position[2],
                'old_path_deviation_m':
                    self.current_old_path_deviation_m,
                'new_path_deviation_m':
                    self.current_new_path_deviation_m
                    if self.current_new_path_deviation_m is not None
                    else float('nan'),
                'roll_deg': self.latest_rpy_deg[0],
                'pitch_deg': self.latest_rpy_deg[1],
                'yaw_deg': self.latest_rpy_deg[2],
                'force_norm_n': force_norm,
                'rpm_1': self.latest_rpm[0],
                'rpm_2': self.latest_rpm[1],
                'rpm_3': self.latest_rpm[2],
                'rpm_4': self.latest_rpm[3],
            })

        if (
            self.new_path_received
            and self.retracking_time_s is None
            and self.current_new_path_deviation_m is not None
        ):
            if (
                self.current_new_path_deviation_m
                <= self.tracking_tolerance_m
            ):
                if self.retracking_candidate_time is None:
                    self.retracking_candidate_time = now
                elif (
                    now - self.retracking_candidate_time
                    >= self.tracking_hold_s
                ):
                    self.retracking_time_s = (
                        self.retracking_candidate_time
                        - (self.new_path_time or now)
                    )
                    self.state = 'high_altitude_tracking_new_path'
                    self.evidence_end_time = (
                        now + self.evidence_hold_s
                    )
                    self.publish_status(
                        'high_altitude_tracking_new_path',
                        force=True,
                        retracking_time_s=self.retracking_time_s,
                    )
            else:
                self.retracking_candidate_time = None

    def prepare_experiment(self) -> None:
        assert self.latest_position is not None
        self.old_path = list(self.current_path)
        self.new_path = []
        self.actual_trail = [self.latest_position]
        self.crosswind_unit = self.calculate_crosswind_unit()
        self.triggered = True
        self.armed = False
        self.state = 'high_altitude_countdown'
        self.scheduled_time = self.now_s() + self.countdown_s
        self.path_count_at_replan = self.path_message_count
        self.current_old_path_deviation_m = 0.0
        self.peak_old_path_deviation_m = 0.0
        self.current_new_path_deviation_m = None
        self.new_path_branch_m = 0.0
        self.replan_requested = False
        self.new_path_received = False
        self.retracking_deadline = None
        self.evidence_end_time = None
        self.target_deviation_reached = False
        self.gust_stop_reason = ''
        self.actual_gust_duration_s = None
        self.tracking_timeout = False
        self.last_trail_time = self.now_s()
        self.publish_status(
            'high_altitude_countdown',
            force=True,
            countdown_s=self.countdown_s,
            crosswind_unit=self.crosswind_unit,
        )
        self.get_logger().warning(
            'HIGH-ALTITUDE WIND GATE ENTERED: '
            f'z={self.latest_position[2]:.1f} m, '
            f'gust starts in {self.countdown_s:.1f} s.'
        )

    def envelope(self, elapsed: float) -> float:
        if elapsed < 0.0 or elapsed > self.duration_s:
            return 0.0

        if self.ramp_time_s > 0.0:
            if elapsed < self.ramp_time_s:
                base = elapsed / self.ramp_time_s
            elif self.duration_s - elapsed < self.ramp_time_s:
                base = (
                    self.duration_s - elapsed
                ) / self.ramp_time_s
            else:
                base = 1.0
        else:
            base = 1.0

        modulation = (
            0.88
            + 0.12
            * math.sin(
                2.0
                * math.pi
                * self.gust_frequency_hz
                * elapsed
            )
        )
        return max(0.0, min(1.0, base * modulation))

    def start_gust(self, now: float) -> None:
        self.scheduled_time = None
        self.gust_start_time = now
        self.gust_end_time = now + self.duration_s
        self.state = 'high_altitude_gust_active'
        self.publish_status(
            'high_altitude_gust_active',
            force=True,
            force_magnitude_n=self.force_magnitude_n,
            duration_s=self.duration_s,
            yaw_torque_nm=self.yaw_torque_nm,
        )
        self.get_logger().warning(
            'HIGH-ALTITUDE GUST ACTIVE DURING CLIMB: '
            f'{self.force_magnitude_n:.2f} N x '
            f'{self.duration_s:.2f} s.'
        )

    def publish_wrench(
        self,
        magnitude: float,
        active: bool,
    ) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x = (
            magnitude * self.crosswind_unit[0]
        )
        message.wrench.force.y = (
            magnitude * self.crosswind_unit[1]
        )
        if active and self.force_magnitude_n > 0.0:
            gain = magnitude / self.force_magnitude_n
            message.wrench.torque.z = gain * self.yaw_torque_nm
        self.wrench_pub.publish(message)

        active_message = Bool()
        active_message.data = active
        self.active_pub.publish(active_message)

    def current_tilt_deg(self) -> float:
        return max(
            abs(self.latest_rpy_deg[0]),
            abs(self.latest_rpy_deg[1]),
        )

    def tilt_force_scale(self) -> float:
        tilt = self.current_tilt_deg()
        if tilt <= self.soft_tilt_start_deg:
            return 1.0
        if tilt >= self.soft_tilt_end_deg:
            return self.minimum_force_scale
        fraction = (
            (tilt - self.soft_tilt_start_deg)
            / (
                self.soft_tilt_end_deg
                - self.soft_tilt_start_deg
            )
        )
        return (
            1.0
            - fraction * (1.0 - self.minimum_force_scale)
        )

    def current_force_magnitude(self, now: float) -> float:
        if (
            self.gust_start_time is None
            or self.gust_end_time is None
            or not (
                self.gust_start_time <= now <= self.gust_end_time
            )
        ):
            return 0.0
        return (
            self.force_magnitude_n
            * self.envelope(now - self.gust_start_time)
            * self.tilt_force_scale()
        )

    def safety_reason(self) -> Optional[str]:
        tilt = max(
            abs(self.latest_rpy_deg[0]),
            abs(self.latest_rpy_deg[1]),
        )
        if tilt >= self.safety_tilt_deg:
            return (
                f'tilt {tilt:.2f} deg >= '
                f'{self.safety_tilt_deg:.2f} deg'
            )
        rpm_limit = self.maximum_rpm * self.safety_rpm_ratio
        if max(self.latest_rpm) >= rpm_limit:
            return (
                f'RPM {max(self.latest_rpm):.1f} >= '
                f'{rpm_limit:.1f}'
            )
        return None

    def stop_gust_and_replan(
        self,
        now: float,
        reason: str,
    ) -> None:
        if self.gust_start_time is not None:
            self.actual_gust_duration_s = max(
                0.0,
                now - self.gust_start_time,
            )
        self.gust_end_time = now
        self.gust_stop_reason = reason
        self.publish_wrench(0.0, False)
        self.publish_status(
            'high_altitude_gust_complete',
            force=True,
            stop_reason=reason,
            peak_deviation_m=
                self.peak_old_path_deviation_m,
            actual_gust_duration_s=
                self.actual_gust_duration_s,
        )
        self.request_replan()

    def request_replan(self) -> None:
        if self.replan_requested or self.latest_raw_goal is None:
            return
        message = self.copy_pose(self.latest_raw_goal)
        message.header.stamp = self.get_clock().now().to_msg()
        self.path_count_at_replan = self.path_message_count
        self.replan_goal_pub.publish(message)
        self.replan_requested = True
        self.replan_request_time = self.now_s()
        self.state = 'high_altitude_replan_requested'
        self.publish_status(
            'high_altitude_replan_requested',
            force=True,
            target=[
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ],
        )
        self.get_logger().warning(
            'HIGH-ALTITUDE DEVIATION -> A* REPLANNING '
            'TO THE SAME P13 TARGET.'
        )

    def marker_due(self, now: float) -> bool:
        if now - self.last_marker_time < self.marker_period_s:
            return False
        self.last_marker_time = now
        return True

    def make_header(self):
        header = WrenchStamped().header
        header.frame_id = 'world'
        header.stamp = self.get_clock().now().to_msg()
        return header

    def add_path(
        self,
        array: MarkerArray,
        marker_id: int,
        path: List[Point3],
        width: float,
        color: Tuple[float, float, float, float],
    ) -> None:
        if len(path) < 2:
            return
        marker = Marker()
        marker.header = self.make_header()
        marker.ns = 'cq01_high_altitude_wind'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [
            Point(x=x, y=y, z=z)
            for x, y, z in path
        ]
        array.markers.append(marker)

    def add_text(
        self,
        array: MarkerArray,
        marker_id: int,
        text: str,
        position: Point3,
        color: Tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header = self.make_header()
        marker.ns = 'cq01_high_altitude_wind'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.15
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        array.markers.append(marker)

    def publish_markers(self, force_magnitude: float) -> None:
        if self.latest_position is None:
            return

        array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        # Fixed physical ruler at the gust start point.
        if self.actual_trail:
            reference = self.actual_trail[0]
            for ring_index, radius in enumerate(
                (2.0, 4.0, 6.0, 8.0),
                start=30,
            ):
                ring = Marker()
                ring.header = self.make_header()
                ring.ns = 'cq01_high_altitude_wind'
                ring.id = ring_index
                ring.type = Marker.LINE_STRIP
                ring.action = Marker.ADD
                ring.scale.x = 0.12
                ring.color.r = 1.0
                ring.color.g = 0.80
                ring.color.b = 0.05
                ring.color.a = 0.85
                for step in range(49):
                    angle = 2.0 * math.pi * step / 48.0
                    ring.points.append(
                        Point(
                            x=reference[0]
                            + radius * math.cos(angle),
                            y=reference[1]
                            + radius * math.sin(angle),
                            z=reference[2],
                        )
                    )
                array.markers.append(ring)

            beacon = Marker()
            beacon.header = self.make_header()
            beacon.ns = 'cq01_high_altitude_wind'
            beacon.id = 40
            beacon.type = Marker.SPHERE
            beacon.action = Marker.ADD
            beacon.pose.position.x = self.latest_position[0]
            beacon.pose.position.y = self.latest_position[1]
            beacon.pose.position.z = self.latest_position[2]
            beacon.pose.orientation.w = 1.0
            beacon.scale.x = 2.4
            beacon.scale.y = 2.4
            beacon.scale.z = 2.4
            beacon.color.r = 1.0
            beacon.color.g = 1.0
            beacon.color.b = 1.0
            beacon.color.a = 0.42
            array.markers.append(beacon)

        self.add_path(
            array,
            1,
            self.old_path,
            0.34,
            (1.0, 0.55, 0.05, 0.76),
        )
        self.add_path(
            array,
            2,
            self.new_path,
            0.54,
            (0.10, 1.0, 0.20, 1.0),
        )
        self.add_path(
            array,
            3,
            self.actual_trail,
            0.28,
            (1.0, 0.05, 0.85, 0.95),
        )

        _, nearest = self.nearest_path_distance(
            self.latest_position,
            self.old_path,
        )
        if nearest is not None:
            deviation = Marker()
            deviation.header = self.make_header()
            deviation.ns = 'cq01_high_altitude_wind'
            deviation.id = 4
            deviation.type = Marker.ARROW
            deviation.action = Marker.ADD
            deviation.points = [
                Point(x=nearest[0], y=nearest[1], z=nearest[2]),
                Point(
                    x=self.latest_position[0],
                    y=self.latest_position[1],
                    z=self.latest_position[2],
                ),
            ]
            deviation.scale.x = 0.23
            deviation.scale.y = 0.52
            deviation.scale.z = 0.68
            deviation.color.r = 0.05
            deviation.color.g = 1.0
            deviation.color.b = 1.0
            deviation.color.a = 1.0
            array.markers.append(deviation)

        if force_magnitude > 0.01:
            force_arrow = Marker()
            force_arrow.header = self.make_header()
            force_arrow.ns = 'cq01_high_altitude_wind'
            force_arrow.id = 5
            force_arrow.type = Marker.ARROW
            force_arrow.action = Marker.ADD
            force_arrow.points = [
                Point(
                    x=self.latest_position[0],
                    y=self.latest_position[1],
                    z=self.latest_position[2] + 0.6,
                ),
                Point(
                    x=self.latest_position[0]
                    + self.visual_force_scale
                    * force_magnitude
                    * self.crosswind_unit[0],
                    y=self.latest_position[1]
                    + self.visual_force_scale
                    * force_magnitude
                    * self.crosswind_unit[1],
                    z=self.latest_position[2] + 0.6,
                ),
            ]
            force_arrow.scale.x = 0.62
            force_arrow.scale.y = 1.30
            force_arrow.scale.z = 1.60
            force_arrow.color.r = 1.0
            force_arrow.color.g = 0.05
            force_arrow.color.b = 0.0
            force_arrow.color.a = 1.0
            array.markers.append(force_arrow)

        altitude = self.latest_position[2]
        anchor = (
            self.actual_trail[0]
            if self.actual_trail else self.latest_position
        )
        if self.state == 'high_altitude_armed':
            label = (
                f'HIGH-ALTITUDE WIND ARMED | '
                f'CLIMBING TO P13 | {altitude:.0f}m'
            )
            color = (0.85, 0.85, 0.85, 1.0)
        elif self.state == 'high_altitude_countdown':
            remaining = max(
                0.0,
                (self.scheduled_time or self.now_s())
                - self.now_s(),
            )
            label = (
                f'RIDGE GUST IN {remaining:.1f}s | '
                f'ALTITUDE {altitude:.0f}m'
            )
            color = (1.0, 0.90, 0.10, 1.0)
        elif self.state == 'high_altitude_gust_active':
            label = (
                f'TARGET OFFSET {self.target_deviation_m:.1f}m | '
                f'NOW {self.current_old_path_deviation_m:.2f}m | '
                f'FORCE {force_magnitude:.2f}N'
            )
            color = (1.0, 0.45, 0.05, 1.0)
        elif self.state == 'high_altitude_replan_requested':
            label = (
                f'DEVIATED '
                f'{self.peak_old_path_deviation_m:.2f}m | '
                'A* REPLANNING TO P13'
            )
            color = (0.05, 1.0, 1.0, 1.0)
        elif self.state in (
            'high_altitude_new_path_ready',
            'high_altitude_tracking_new_path',
        ):
            latency_ms = 1000.0 * (self.replan_latency_s or 0.0)
            label = (
                f'PHYSICAL OFFSET '
                f'{self.peak_old_path_deviation_m:.2f}m | '
                f'NEW PATH BRANCH {self.new_path_branch_m:.2f}m | '
                f'{latency_ms:.0f}ms'
            )
            color = (0.15, 1.0, 0.25, 1.0)
        elif self.state == 'high_altitude_safety_abort':
            label = (
                f'WIND SAFETY CUT-OFF | '
                f'{self.safety_abort_reason}'
            )
            color = (1.0, 0.05, 0.05, 1.0)
        else:
            label = 'HIGH-ALTITUDE WIND STAGE'
            color = (0.85, 0.85, 0.85, 1.0)

        self.add_text(
            array,
            20,
            label,
            (anchor[0], anchor[1], anchor[2] + 9.0),
            color,
        )
        self.marker_pub.publish(array)

    def timer_callback(self) -> None:
        now = self.now_s()
        force_magnitude = 0.0

        if (
            self.armed
            and not self.triggered
            and self.inside_gate()
            and self.latest_raw_goal is not None
            and len(self.current_path) >= 2
        ):
            self.prepare_experiment()

        if (
            self.state == 'high_altitude_countdown'
            and self.scheduled_time is not None
            and now >= self.scheduled_time
        ):
            self.start_gust(now)

        if self.state == 'high_altitude_gust_active':
            reason = self.safety_reason()
            elapsed = (
                now - self.gust_start_time
                if self.gust_start_time is not None
                else 0.0
            )
            target_reached = (
                elapsed >= self.minimum_gust_duration_s
                and self.current_old_path_deviation_m
                >= self.target_deviation_m
            )
            duration_reached = elapsed >= self.duration_s

            if reason is not None:
                self.safety_abort = True
                self.safety_abort_reason = reason
                self.state = 'high_altitude_safety_abort'
                self.stop_gust_and_replan(
                    now,
                    f'safety_cutoff: {reason}',
                )
                self.publish_status(
                    'high_altitude_safety_abort',
                    force=True,
                    reason=reason,
                )
            elif target_reached:
                self.target_deviation_reached = True
                force_magnitude = 0.0
                self.stop_gust_and_replan(
                    now,
                    'target_deviation_reached',
                )
            elif duration_reached:
                force_magnitude = 0.0
                self.stop_gust_and_replan(
                    now,
                    'maximum_duration_reached',
                )
            else:
                force_magnitude = self.current_force_magnitude(now)
                self.publish_wrench(force_magnitude, True)
                self.publish_status(
                    'high_altitude_gust_active',
                    force_command_n=force_magnitude,
                    tilt_force_scale=self.tilt_force_scale(),
                    target_deviation_m=self.target_deviation_m,
                )
        else:
            self.publish_wrench(0.0, False)

        if (
            self.new_path_received
            and self.retracking_time_s is None
            and self.retracking_deadline is not None
            and now >= self.retracking_deadline
        ):
            self.tracking_timeout = True
            self.retracking_deadline = None
            self.evidence_end_time = now + 2.0
            self.publish_status(
                'high_altitude_tracking_timeout',
                force=True,
                current_new_path_deviation_m=
                    self.current_new_path_deviation_m,
            )

        if (
            self.replan_requested
            and self.evidence_end_time is not None
            and now >= self.evidence_end_time
            and self.saved_output_dir is None
        ):
            final_state = (
                'high_altitude_safety_abort_complete'
                if self.safety_abort
                else 'high_altitude_evidence_complete'
            )
            self.state = final_state
            self.publish_status(
                final_state,
                force=True,
            )
            self.save_results(final_state)

        if self.marker_due(now) and self.triggered:
            self.publish_markers(force_magnitude)

    def compute_summary(self, final_state: str) -> Dict:
        criteria = {
            'gust_applied':
                self.peak_applied_force_n >= 2.4,
            'target_deviation_reached':
                self.target_deviation_reached,
            'visible_deviation':
                self.peak_old_path_deviation_m
                >= 0.90 * self.target_deviation_m,
            'replan_requested': self.replan_requested,
            'new_path_received': self.new_path_received,
            'visible_new_path_branch':
                self.new_path_branch_m
                >= self.minimum_visible_branch_m,
            'tracking_recovered':
                self.retracking_time_s is not None,
            'tilt_pass':
                self.maximum_tilt_observed_deg
                <= self.safety_tilt_deg,
            'rpm_pass':
                self.maximum_rpm_observed
                <= self.maximum_rpm * self.safety_rpm_ratio,
            'no_safety_abort': not self.safety_abort,
        }
        return {
            'experiment':
                'cq01_high_altitude_climb_strong_wind_replan',
            'final_state': final_state,
            'gate': self.gate,
            'force_magnitude_n': self.force_magnitude_n,
            'maximum_duration_s': self.duration_s,
            'minimum_gust_duration_s':
                self.minimum_gust_duration_s,
            'target_deviation_m': self.target_deviation_m,
            'target_deviation_reached':
                self.target_deviation_reached,
            'actual_gust_duration_s':
                self.actual_gust_duration_s,
            'gust_stop_reason': self.gust_stop_reason,
            'yaw_torque_nm': self.yaw_torque_nm,
            'crosswind_unit': self.crosswind_unit,
            'peak_applied_force_n': self.peak_applied_force_n,
            'peak_old_path_deviation_m':
                self.peak_old_path_deviation_m,
            'new_path_branch_m': self.new_path_branch_m,
            'replan_latency_s': self.replan_latency_s,
            'retracking_time_s': self.retracking_time_s,
            'maximum_tilt_deg':
                self.maximum_tilt_observed_deg,
            'maximum_rpm': self.maximum_rpm_observed,
            'safety_abort': self.safety_abort,
            'safety_abort_reason': self.safety_abort_reason,
            'tracking_timeout': self.tracking_timeout,
            'criteria': criteria,
            'pass': all(criteria.values()),
            'sample_count': len(self.samples),
        }

    def save_results(self, final_state: str) -> None:
        if self.saved_output_dir is not None:
            return

        summary = self.compute_summary(final_state)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = (
            self.results_root
            / f'high_altitude_wind_{timestamp}'
        )
        output_dir.mkdir(parents=True, exist_ok=True)

        (output_dir / 'summary.json').write_text(
            json.dumps(summary, indent=2, ensure_ascii=False),
            encoding='utf-8',
        )

        if self.samples:
            with (output_dir / 'samples.csv').open(
                'w',
                newline='',
                encoding='utf-8',
            ) as stream:
                writer = csv.DictWriter(
                    stream,
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
                        sample['old_path_deviation_m']
                        for sample in self.samples
                    ],
                    label='Deviation from old path',
                )
                plt.plot(
                    times,
                    [
                        sample['new_path_deviation_m']
                        for sample in self.samples
                    ],
                    label='Deviation from replanned path',
                )
                plt.xlabel('Time after high-altitude gust start (s)')
                plt.ylabel('Path deviation (m)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir
                    / 'high_altitude_deviation_and_retracking.png',
                    dpi=180,
                )
                plt.close()

                plt.figure()
                plt.plot(
                    times,
                    [
                        sample['force_norm_n']
                        for sample in self.samples
                    ],
                    label='Applied force',
                )
                plt.xlabel('Time after gust start (s)')
                plt.ylabel('Force (N)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'high_altitude_wind_profile.png',
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
                plt.plot(
                    times,
                    [sample['yaw_deg'] for sample in self.samples],
                    label='Yaw',
                )
                plt.xlabel('Time after gust start (s)')
                plt.ylabel('Attitude (deg)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'high_altitude_attitude.png',
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
                plt.xlabel('Time after gust start (s)')
                plt.ylabel('RPM')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'high_altitude_motor_rpm.png',
                    dpi=180,
                )
                plt.close()
        except Exception as exception:
            self.get_logger().warning(
                f'High-altitude plot generation failed: {exception}'
            )

        self.saved_output_dir = output_dir
        self.publish_status(
            'high_altitude_result_saved',
            force=True,
            output_dir=str(output_dir),
            pass_result=summary['pass'],
        )
        self.get_logger().info(
            'High-altitude wind result: '
            f'pass={summary["pass"]}, '
            f'peak_deviation='
            f'{summary["peak_old_path_deviation_m"]:.2f}m, '
            f'branch={summary["new_path_branch_m"]:.2f}m, '
            f'replan_latency={summary["replan_latency_s"]}'
        )
        self.get_logger().info(
            f'High-altitude results saved to: {output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HighAltitudeWindManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.triggered and node.saved_output_dir is None:
            node.save_results('keyboard_interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
