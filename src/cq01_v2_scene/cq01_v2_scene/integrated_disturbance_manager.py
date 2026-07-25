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
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
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


class CQ01DynamicReplanDisturbanceManager(Node):
    """Crosswind-induced deviation and explicit A* replanning in CQ-01.

    The aircraft never pauses for a hover test. During normal CQ-01 flight:
      1. snapshot the currently followed A* path and current mission target;
      2. calculate a crosswind direction perpendicular to the local path;
      3. apply a real external force while the aircraft keeps flying;
      4. republish the same mission target to force a fresh A* plan from the
         disturbed odometry position;
      5. display the old path, new path, actual trail and deviation together;
      6. verify that the mission continues to the next waypoint and P15.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v34h_integrated_disturbance_manager')

        self.declare_parameter('trigger_mode', 'manual')
        self.declare_parameter('trigger_after_reached_index', 1)
        self.declare_parameter('trigger_delay_s', 1.0)
        self.declare_parameter('manual_countdown_s', 2.0)

        self.declare_parameter('force_magnitude_n', 2.5)
        self.declare_parameter('duration_s', 1.0)
        self.declare_parameter('ramp_time_s', 0.15)
        self.declare_parameter('lateral_sign', 1.0)
        self.declare_parameter('torque_z_nm', 0.0)

        # Deliberate two-route decision point around the restored V12
        # planning obstacle on P1 -> P2.
        self.declare_parameter('fork_center_x_m', 60.70)
        self.declare_parameter('fork_center_y_m', -85.88)
        self.declare_parameter('fork_tangent_x', 28.0)
        self.declare_parameter('fork_tangent_y', 27.0)
        self.declare_parameter('minimum_route_branch_m', 3.0)

        # Stable spatial trigger gate on the P1 -> P2 path.
        # The RViz camera is fixed on the fork from launch time; the service
        # only arms the experiment and never moves the camera.
        self.declare_parameter('trigger_gate_x_m', 52.0)
        self.declare_parameter('trigger_gate_y_m', -90.0)
        self.declare_parameter('trigger_gate_z_m', 25.0)
        self.declare_parameter('trigger_gate_radius_xy_m', 7.0)
        self.declare_parameter('trigger_gate_tolerance_z_m', 8.0)
        self.declare_parameter('gate_countdown_s', 0.8)

        # Repeat triggering inside the same fixed RViz field of view.
        self.declare_parameter('repeat_countdown_s', 1.0)
        self.declare_parameter('repeat_cooldown_s', 2.0)
        self.declare_parameter('repeat_zone_radius_xy_m', 34.0)
        self.declare_parameter('repeat_zone_tolerance_z_m', 18.0)
        self.declare_parameter('maximum_trigger_count', 3)
        self.declare_parameter('maximum_queued_repeats', 2)
        self.declare_parameter('auto_queue_repeats', 2)
        self.declare_parameter('publish_rate_hz', 100.0)

        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('raw_goal_topic', '/drone/goal_raw')
        self.declare_parameter('path_topic', '/drone/planned_path_3d')
        self.declare_parameter(
            'planner_status_topic',
            '/drone/planner_3d/status',
        )
        self.declare_parameter(
            'mission_status_topic',
            '/cq01_v34h/mission_status',
        )
        self.declare_parameter(
            'disturbance_status_topic',
            '/cq01_v34h/disturbance_status',
        )
        self.declare_parameter('rpm_topic', '/drone/motor_rpm')
        self.declare_parameter(
            'command_wrench_topic',
            '/drone/external_wrench',
        )
        self.declare_parameter(
            'applied_wrench_topic',
            '/drone/external_wrench_applied',
        )
        self.declare_parameter(
            'active_topic',
            '/drone/disturbance_active',
        )
        self.declare_parameter(
            'marker_topic',
            '/cq01_v34h/disturbance_markers',
        )

        self.declare_parameter('replan_delay_after_force_s', 0.15)
        self.declare_parameter('minimum_replan_deviation_m', 0.80)
        self.declare_parameter('replan_force_timeout_s', 1.20)
        self.declare_parameter('tracking_tolerance_m', 0.90)
        self.declare_parameter('tracking_hold_s', 0.70)
        self.declare_parameter('marker_publish_rate_hz', 10.0)
        self.declare_parameter('trail_sample_period_s', 0.08)
        self.declare_parameter('visual_force_scale_m_per_n', 4.0)
        self.declare_parameter('evidence_hold_s', 15.0)
        self.declare_parameter('maximum_tilt_deg', 45.0)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter('saturation_ratio_threshold', 0.98)
        self.declare_parameter('maximum_saturation_fraction', 0.05)
        self.declare_parameter('allow_retrigger', True)
        self.declare_parameter(
            'results_root',
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_dynamic_replan_disturbance'
            ),
        )

        self.trigger_mode = str(
            self.get_parameter('trigger_mode').value
        ).strip().lower()
        if self.trigger_mode not in ('manual', 'waypoint'):
            raise ValueError("trigger_mode must be manual or waypoint")

        self.trigger_after_reached_index = int(
            self.get_parameter('trigger_after_reached_index').value
        )
        self.trigger_delay_s = max(
            0.0,
            float(self.get_parameter('trigger_delay_s').value),
        )
        self.manual_countdown_s = max(
            0.0,
            float(self.get_parameter('manual_countdown_s').value),
        )
        self.force_magnitude_n = max(
            0.1,
            float(self.get_parameter('force_magnitude_n').value),
        )
        self.duration_s = max(
            0.1,
            float(self.get_parameter('duration_s').value),
        )
        self.ramp_time_s = max(
            0.0,
            min(
                float(self.get_parameter('ramp_time_s').value),
                self.duration_s / 2.0,
            ),
        )
        self.lateral_sign = (
            1.0
            if float(self.get_parameter('lateral_sign').value) >= 0.0
            else -1.0
        )
        self.torque_z_nm = float(
            self.get_parameter('torque_z_nm').value
        )
        self.fork_center = (
            float(self.get_parameter('fork_center_x_m').value),
            float(self.get_parameter('fork_center_y_m').value),
        )
        tangent_x = float(
            self.get_parameter('fork_tangent_x').value
        )
        tangent_y = float(
            self.get_parameter('fork_tangent_y').value
        )
        tangent_norm = max(
            1.0e-9,
            math.hypot(tangent_x, tangent_y),
        )
        self.fork_tangent = (
            tangent_x / tangent_norm,
            tangent_y / tangent_norm,
        )
        self.fork_normal = (
            -self.fork_tangent[1],
            self.fork_tangent[0],
        )
        self.minimum_route_branch_m = max(
            1.0,
            float(
                self.get_parameter(
                    'minimum_route_branch_m'
                ).value
            ),
        )
        self.trigger_gate = (
            float(self.get_parameter('trigger_gate_x_m').value),
            float(self.get_parameter('trigger_gate_y_m').value),
            float(self.get_parameter('trigger_gate_z_m').value),
        )
        self.trigger_gate_radius_xy_m = max(
            1.0,
            float(
                self.get_parameter(
                    'trigger_gate_radius_xy_m'
                ).value
            ),
        )
        self.trigger_gate_tolerance_z_m = max(
            1.0,
            float(
                self.get_parameter(
                    'trigger_gate_tolerance_z_m'
                ).value
            ),
        )
        self.gate_countdown_s = max(
            0.0,
            float(
                self.get_parameter('gate_countdown_s').value
            ),
        )
        self.repeat_countdown_s = max(
            0.0,
            float(
                self.get_parameter('repeat_countdown_s').value
            ),
        )
        self.repeat_cooldown_s = max(
            0.0,
            float(
                self.get_parameter('repeat_cooldown_s').value
            ),
        )
        self.repeat_zone_radius_xy_m = max(
            self.trigger_gate_radius_xy_m,
            float(
                self.get_parameter(
                    'repeat_zone_radius_xy_m'
                ).value
            ),
        )
        self.repeat_zone_tolerance_z_m = max(
            self.trigger_gate_tolerance_z_m,
            float(
                self.get_parameter(
                    'repeat_zone_tolerance_z_m'
                ).value
            ),
        )
        self.maximum_trigger_count = max(
            1,
            int(
                self.get_parameter(
                    'maximum_trigger_count'
                ).value
            ),
        )
        self.maximum_queued_repeats = max(
            0,
            int(
                self.get_parameter(
                    'maximum_queued_repeats'
                ).value
            ),
        )
        self.auto_queue_repeats = max(
            0,
            min(
                self.maximum_queued_repeats,
                int(
                    self.get_parameter(
                        'auto_queue_repeats'
                    ).value
                ),
            ),
        )
        self.replan_delay_after_force_s = max(
            0.0,
            float(
                self.get_parameter(
                    'replan_delay_after_force_s'
                ).value
            ),
        )
        self.minimum_replan_deviation_m = max(
            0.1,
            float(
                self.get_parameter(
                    'minimum_replan_deviation_m'
                ).value
            ),
        )
        self.replan_force_timeout_s = max(
            0.2,
            float(
                self.get_parameter(
                    'replan_force_timeout_s'
                ).value
            ),
        )
        self.tracking_tolerance_m = max(
            0.1,
            float(self.get_parameter('tracking_tolerance_m').value),
        )
        self.tracking_hold_s = max(
            0.1,
            float(self.get_parameter('tracking_hold_s').value),
        )
        self.trail_sample_period_s = max(
            0.02,
            float(
                self.get_parameter('trail_sample_period_s').value
            ),
        )
        marker_rate = max(
            1.0,
            float(
                self.get_parameter(
                    'marker_publish_rate_hz'
                ).value
            ),
        )
        self.marker_period_s = 1.0 / marker_rate
        self.visual_force_scale = max(
            1.0,
            float(
                self.get_parameter(
                    'visual_force_scale_m_per_n'
                ).value
            ),
        )
        self.evidence_hold_s = max(
            5.0,
            float(self.get_parameter('evidence_hold_s').value),
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
        self.allow_retrigger = bool(
            self.get_parameter('allow_retrigger').value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter('results_root').value)
            )
        )

        rate_hz = max(
            20.0,
            float(self.get_parameter('publish_rate_hz').value),
        )
        self.timer_period_s = 1.0 / rate_hz

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
        # A second publisher on /drone/goal_raw is deliberate. Re-publishing
        # the current mission goal forces A* to plan again from current odom.
        self.replan_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('raw_goal_topic').value),
            10,
        )
        self.focus_tf_broadcaster = TransformBroadcaster(self)

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
            String,
            str(self.get_parameter('mission_status_topic').value),
            self.mission_status_callback,
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

        self.create_service(
            Trigger,
            '~/trigger',
            self.trigger_callback,
        )
        self.create_service(
            Trigger,
            '~/reset',
            self.reset_callback,
        )
        self.create_service(
            Trigger,
            '~/finalize',
            self.finalize_callback,
        )

        self.latest_position: Optional[Point3] = None
        self.latest_rpy_deg = (0.0, 0.0, 0.0)
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.latest_applied_force = [0.0, 0.0, 0.0]
        self.latest_raw_goal: Optional[PoseStamped] = None
        self.latest_mission_payload: Dict = {}

        self.current_path: List[Point3] = []
        self.old_path: List[Point3] = []
        self.new_path: List[Point3] = []
        self.actual_trail: List[Point3] = []
        self.path_message_count = 0
        self.path_count_at_trigger = 0

        self.state = 'idle'
        self.armed = False
        self.gate_entered = False
        self.trigger_count = 0
        self.pending_repeat_count = 0
        self.last_cycle_end_time: Optional[float] = None
        self.scheduled_time: Optional[float] = None
        self.disturbance_start_time: Optional[float] = None
        self.disturbance_end_time: Optional[float] = None
        self.replan_request_time: Optional[float] = None
        self.new_path_time: Optional[float] = None
        self.replan_latency_s: Optional[float] = None
        self.retracking_time_s: Optional[float] = None
        self.retracking_candidate_time: Optional[float] = None
        self.evidence_end_time: Optional[float] = None

        self.trigger_position: Optional[Point3] = None
        self.focus_position: Optional[Point3] = (
            self.fork_center[0],
            self.fork_center[1],
            self.trigger_gate[2],
        )
        self.crosswind_unit = (0.0, 1.0, 0.0)
        self.peak_old_path_deviation_m = 0.0
        self.current_old_path_deviation_m = 0.0
        self.current_new_path_deviation_m: Optional[float] = None
        self.new_path_branch_m = 0.0
        self.old_route_side_value = 0.0
        self.new_route_side_value = 0.0
        self.old_route_side_name = 'UNKNOWN'
        self.new_route_side_name = 'UNKNOWN'
        self.route_switch_confirmed = False
        self.replan_requested = False
        self.new_path_received = False
        self.mission_completed_after_replan = False
        self.mission_final_state: Optional[str] = None

        self.last_trail_sample_time = -1.0e9
        self.last_marker_publish_time = -1.0e9
        self.last_status_state: Optional[str] = None
        self.last_status_publish_time = -1.0e9
        self.samples: List[Dict[str, float]] = []
        self.saved_output_dir: Optional[Path] = None

        self.timer = self.create_timer(
            self.timer_period_s,
            self.timer_callback,
        )
        self.publish_status('ready', force=True)
        self.get_logger().info(
            'CQ-01 dynamic-replan disturbance manager ready: '
            f'crosswind={self.force_magnitude_n:.2f}N, '
            f'duration={self.duration_s:.2f}s. '
            'The aircraft remains in normal flight.'
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
            and now - self.last_status_publish_time < 1.0
        ):
            return
        payload = {
            'state': state,
            'active': self.is_disturbance_active(now),
            'armed': self.armed,
            'gate_entered': self.gate_entered,
            'trigger_count': self.trigger_count,
            'pending_repeat_count':
                self.pending_repeat_count,
            'maximum_trigger_count':
                self.maximum_trigger_count,
            'auto_queue_repeats':
                self.auto_queue_repeats,
            'current_old_path_deviation_m':
                self.current_old_path_deviation_m,
            'peak_old_path_deviation_m':
                self.peak_old_path_deviation_m,
            'current_new_path_deviation_m':
                self.current_new_path_deviation_m,
            'replan_requested': self.replan_requested,
            'new_path_received': self.new_path_received,
            'old_route_side': self.old_route_side_name,
            'new_route_side': self.new_route_side_name,
            'route_switch_confirmed':
                self.route_switch_confirmed,
            'replan_latency_s': self.replan_latency_s,
            'mission_completed_after_replan':
                self.mission_completed_after_replan,
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
            and self.path_message_count > self.path_count_at_trigger
        ):
            self.new_path = list(points)
            self.new_path_received = True
            self.new_path_time = self.now_s()
            self.last_cycle_end_time = self.new_path_time
            if self.replan_request_time is not None:
                self.replan_latency_s = (
                    self.new_path_time - self.replan_request_time
                )
            self.new_path_branch_m = self.calculate_path_branch()
            self.new_route_side_value = self.fork_side_value(
                self.new_path
            )
            self.new_route_side_name = self.route_side_name(
                self.new_route_side_value
            )
            self.route_switch_confirmed = (
                self.old_route_side_value
                * self.new_route_side_value < 0.0
                and self.new_path_branch_m
                >= self.minimum_route_branch_m
            )
            self.state = (
                'route_switch_ready'
                if self.route_switch_confirmed
                else 'new_path_same_branch'
            )
            self.evidence_end_time = (
                self.now_s() + self.evidence_hold_s
            )
            self.publish_status(
                'new_path_ready',
                force=True,
                replan_latency_s=self.replan_latency_s,
                branch_m=self.new_path_branch_m,
                old_route_side=self.old_route_side_name,
                new_route_side=self.new_route_side_name,
                route_switch_confirmed=
                    self.route_switch_confirmed,
            )
            self.get_logger().warning(
                'NEW A* PATH READY: '
                f'latency={1000.0 * (self.replan_latency_s or 0.0):.1f}ms, '
                f'visible_branch={self.new_path_branch_m:.2f}m, '
                f'route={self.old_route_side_name}'
                f'->{self.new_route_side_name}, '
                f'switched={self.route_switch_confirmed}'
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
            self.state = 'replan_failed'
            self.publish_status(
                'replan_failed',
                force=True,
                planner_status=payload,
            )

    def mission_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.latest_mission_payload = payload

        if (
            self.trigger_mode == 'waypoint'
            and self.state == 'idle'
            and str(payload.get('state', '')) == 'advancing'
            and int(payload.get('reached_index', -999))
            == self.trigger_after_reached_index
        ):
            self.schedule_trigger(self.trigger_delay_s)

        state = str(payload.get('state', ''))
        if state in (
            'mission_completed',
            'planning_failed',
            'mission_failed',
            'stalled',
        ):
            self.mission_final_state = state
            if self.replan_requested:
                self.mission_completed_after_replan = (
                    state == 'mission_completed'
                )
                self.save_results(final_state=state)
                self.publish_status(
                    'results_saved',
                    force=True,
                    output_dir=str(self.saved_output_dir)
                    if self.saved_output_dir else None,
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

    def fork_side_value(
        self,
        path: List[Point3],
    ) -> float:
        if not path:
            return 0.0

        center_x, center_y = self.fork_center
        nearest = min(
            path,
            key=lambda point: math.hypot(
                point[0] - center_x,
                point[1] - center_y,
            ),
        )
        return (
            (nearest[0] - center_x) * self.fork_normal[0]
            + (nearest[1] - center_y) * self.fork_normal[1]
        )

    def route_side_name(self, side_value: float) -> str:
        if side_value > 0.25:
            return 'LEFT'
        if side_value < -0.25:
            return 'RIGHT'
        return 'CENTER'

    def calculate_crosswind_unit(self) -> Point3:
        # The old A* route is classified relative to the V12 fork island.
        # Crosswind is intentionally directed to the opposite side so the
        # disturbed start state is more likely to select the other corridor.
        self.old_route_side_value = self.fork_side_value(
            self.current_path
        )
        self.old_route_side_name = self.route_side_name(
            self.old_route_side_value
        )

        if self.old_route_side_value >= 0.0:
            sign = -1.0
        else:
            sign = 1.0

        return (
            sign * self.fork_normal[0],
            sign * self.fork_normal[1],
            0.0,
        )

    def calculate_path_branch(self) -> float:
        if not self.old_path or not self.new_path:
            return 0.0
        sample = self.new_path[: min(12, len(self.new_path))]
        values = []
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

        if self.old_path:
            old_distance, _ = self.nearest_path_distance(
                self.latest_position,
                self.old_path,
            )
            self.current_old_path_deviation_m = (
                old_distance or 0.0
            )
            self.peak_old_path_deviation_m = max(
                self.peak_old_path_deviation_m,
                self.current_old_path_deviation_m,
            )

        if self.new_path:
            new_distance, _ = self.nearest_path_distance(
                self.latest_position,
                self.new_path,
            )
            self.current_new_path_deviation_m = new_distance
        else:
            self.current_new_path_deviation_m = None

        now = self.now_s()
        if (
            self.state != 'idle'
            and now - self.last_trail_sample_time
            >= self.trail_sample_period_s
        ):
            self.actual_trail.append(self.latest_position)
            self.last_trail_sample_time = now

        if self.state != 'idle' and self.disturbance_start_time is not None:
            force_norm = math.sqrt(
                sum(value * value for value in self.latest_applied_force)
            )
            tilt = max(
                abs(self.latest_rpy_deg[0]),
                abs(self.latest_rpy_deg[1]),
            )
            self.samples.append({
                'time_s': now - self.disturbance_start_time,
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
                'tilt_deg': tilt,
                'force_x_n': self.latest_applied_force[0],
                'force_y_n': self.latest_applied_force[1],
                'force_z_n': self.latest_applied_force[2],
                'force_norm_n': force_norm,
                'rpm_1': self.latest_rpm[0],
                'rpm_2': self.latest_rpm[1],
                'rpm_3': self.latest_rpm[2],
                'rpm_4': self.latest_rpm[3],
                'rpm_max': max(self.latest_rpm),
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
                    self.state = 'tracking_replanned_path'
                    self.evidence_end_time = now + self.evidence_hold_s
                    self.publish_status(
                        'tracking_replanned_path',
                        force=True,
                        retracking_time_s=self.retracking_time_s,
                    )
            else:
                self.retracking_candidate_time = None

    def inside_repeat_zone(self) -> bool:
        if self.latest_position is None:
            return False
        dx = self.latest_position[0] - self.fork_center[0]
        dy = self.latest_position[1] - self.fork_center[1]
        dz = abs(
            self.latest_position[2] - self.trigger_gate[2]
        )
        return (
            math.hypot(dx, dy)
            <= self.repeat_zone_radius_xy_m
            and dz <= self.repeat_zone_tolerance_z_m
        )

    def repeat_ready_state(self) -> bool:
        return self.state in (
            'route_switch_ready',
            'new_path_same_branch',
            'tracking_replanned_path',
        )

    def schedule_repeat_now(self) -> bool:
        if self.latest_position is None:
            return False
        if self.latest_raw_goal is None or len(self.current_path) < 2:
            return False
        if not self.inside_repeat_zone():
            return False
        if self.trigger_count >= self.maximum_trigger_count:
            return False

        self.prepare_experiment()
        self.scheduled_time = (
            self.now_s() + self.repeat_countdown_s
        )
        self.state = 'repeat_countdown'
        self.publish_status(
            'repeat_countdown',
            force=True,
            countdown_s=self.repeat_countdown_s,
            repeat_index=self.trigger_count + 1,
        )
        self.get_logger().warning(
            'REPEAT ROUTE-SWITCH CYCLE SCHEDULED: '
            f'#{self.trigger_count + 1}, '
            f'countdown={self.repeat_countdown_s:.1f}s'
        )
        return True

    def queue_repeat(self) -> bool:
        remaining_capacity = (
            self.maximum_trigger_count
            - self.trigger_count
            - self.pending_repeat_count
        )
        if remaining_capacity <= 0:
            return False
        if (
            self.pending_repeat_count
            >= self.maximum_queued_repeats
        ):
            return False

        self.pending_repeat_count += 1
        self.publish_status(
            'repeat_queued',
            force=True,
            pending_repeat_count=
                self.pending_repeat_count,
        )
        self.get_logger().info(
            'Repeat disturbance queued: '
            f'pending={self.pending_repeat_count}'
        )
        return True

    def inside_trigger_gate(self) -> bool:
        if self.latest_position is None:
            return False
        dx = self.latest_position[0] - self.trigger_gate[0]
        dy = self.latest_position[1] - self.trigger_gate[1]
        dz = abs(self.latest_position[2] - self.trigger_gate[2])
        return (
            math.hypot(dx, dy) <= self.trigger_gate_radius_xy_m
            and dz <= self.trigger_gate_tolerance_z_m
        )

    def arm_experiment(self) -> None:
        self.armed = True
        self.gate_entered = False
        self.state = 'armed_waiting_for_gate'
        self.publish_status(
            'armed_waiting_for_gate',
            force=True,
            trigger_gate=list(self.trigger_gate),
            radius_xy_m=self.trigger_gate_radius_xy_m,
        )
        self.get_logger().info(
            'Route-switch disturbance ARMED. Waiting for aircraft '
            f'near gate={self.trigger_gate}, '
            f'radius={self.trigger_gate_radius_xy_m:.1f}m'
        )

    def enter_trigger_gate(self) -> None:
        if self.latest_position is None:
            return
        if self.latest_raw_goal is None or len(self.current_path) < 2:
            return

        self.gate_entered = True
        self.armed = False
        self.prepare_experiment()
        self.pending_repeat_count = min(
            self.auto_queue_repeats,
            self.maximum_trigger_count - 1,
        )
        self.scheduled_time = self.now_s() + self.gate_countdown_s
        self.state = 'gate_countdown'
        self.publish_status(
            'gate_countdown',
            force=True,
            countdown_s=self.gate_countdown_s,
            position=list(self.latest_position),
            old_route_side=self.old_route_side_name,
            auto_queued_repeats=self.pending_repeat_count,
        )
        self.get_logger().warning(
            'AIRCRAFT ENTERED FIXED CAMERA GATE -> '
            f'crosswind starts in {self.gate_countdown_s:.1f}s'
        )

    def trigger_callback(self, _request, response):
        now = self.now_s()

        if self.trigger_count >= self.maximum_trigger_count:
            response.success = False
            response.message = (
                'Maximum disturbance count reached: '
                f'{self.maximum_trigger_count}'
            )
            return response

        if (
            self.armed
            and self.state == 'armed_waiting_for_gate'
        ):
            response.success = False
            response.message = (
                'Route-switch test is already armed. '
                'Wait for the aircraft to enter the yellow gate.'
            )
            return response

        if (
            self.scheduled_time is not None
            or self.is_disturbance_active(now)
            or self.state in (
                'crosswind_active',
                'replan_requested',
            )
        ):
            if self.queue_repeat():
                response.success = True
                response.message = (
                    'Current cycle is active. One repeat has been '
                    f'queued; pending={self.pending_repeat_count}.'
                )
            else:
                response.success = False
                response.message = (
                    'Current cycle is active and no additional '
                    'repeat slot is available.'
                )
            return response

        if self.repeat_ready_state():
            if not self.inside_repeat_zone():
                response.success = False
                response.message = (
                    'Previous cycle finished, but the aircraft has '
                    'left the fixed-camera repeat zone. Restart the '
                    'mission to repeat the route-switch demonstration.'
                )
                return response
            if self.schedule_repeat_now():
                response.success = True
                response.message = (
                    'Repeat route-switch cycle accepted. '
                    f'Crosswind starts in '
                    f'{self.repeat_countdown_s:.1f}s.'
                )
            else:
                response.success = False
                response.message = (
                    'Repeat could not be scheduled because odometry, '
                    'goal, or active A* path is unavailable.'
                )
            return response

        if self.state not in ('idle', 'reset'):
            response.success = False
            response.message = (
                f'Cannot trigger from state={self.state}'
            )
            return response

        self.arm_experiment()
        response.success = True
        response.message = (
            'Route-switch sequence armed. Camera remains fixed. '
            'When the aircraft enters the P1->P2 gate, the system '
            f'will execute up to {self.maximum_trigger_count} real '
            'crosswind/replan cycles automatically.'
        )
        return response

    def reset_callback(self, _request, response):
        self.clear_state(delete_markers=True)
        response.success = True
        response.message = 'Dynamic-replan disturbance state reset'
        return response

    def finalize_callback(self, _request, response):
        if self.state == 'idle':
            response.success = False
            response.message = 'No experiment is active'
            return response
        self.save_results(final_state='manual_finalize')
        response.success = True
        response.message = f'Results saved to {self.saved_output_dir}'
        return response

    def prepare_experiment(self) -> None:
        assert self.latest_position is not None
        self.old_path = list(self.current_path)
        self.new_path = []
        self.actual_trail = [self.latest_position]
        self.path_count_at_trigger = self.path_message_count
        self.trigger_position = self.latest_position
        self.focus_position = (
            self.fork_center[0],
            self.fork_center[1],
            self.trigger_gate[2],
        )
        self.crosswind_unit = self.calculate_crosswind_unit()
        self.peak_old_path_deviation_m = 0.0
        self.current_old_path_deviation_m = 0.0
        self.current_new_path_deviation_m = None
        self.new_path_branch_m = 0.0
        self.new_route_side_value = 0.0
        self.new_route_side_name = 'UNKNOWN'
        self.route_switch_confirmed = False
        self.replan_requested = False
        self.new_path_received = False
        self.mission_completed_after_replan = False
        self.mission_final_state = None
        self.disturbance_start_time = None
        self.disturbance_end_time = None
        self.replan_request_time = None
        self.new_path_time = None
        self.replan_latency_s = None
        self.retracking_time_s = None
        self.retracking_candidate_time = None
        self.evidence_end_time = None
        self.last_trail_sample_time = self.now_s()
        self.samples = []
        self.saved_output_dir = None

    def schedule_trigger(self, delay_s: float) -> None:
        if not self.old_path and self.latest_position is not None:
            self.prepare_experiment()
        self.scheduled_time = self.now_s() + max(0.0, delay_s)
        self.state = 'scheduled'
        self.publish_status(
            'scheduled',
            force=True,
            countdown_s=delay_s,
            crosswind_unit=self.crosswind_unit,
        )

    def clear_state(self, delete_markers: bool = False) -> None:
        self.publish_zero_wrench()
        self.state = 'idle'
        self.armed = False
        self.gate_entered = False
        self.trigger_count = 0
        self.pending_repeat_count = 0
        self.last_cycle_end_time = None
        self.scheduled_time = None
        self.disturbance_start_time = None
        self.disturbance_end_time = None
        self.replan_request_time = None
        self.new_path_time = None
        self.replan_latency_s = None
        self.retracking_time_s = None
        self.retracking_candidate_time = None
        self.evidence_end_time = None
        self.old_path = []
        self.new_path = []
        self.actual_trail = []
        self.trigger_position = None
        self.focus_position = (
            self.fork_center[0],
            self.fork_center[1],
            self.trigger_gate[2],
        )
        self.replan_requested = False
        self.new_path_received = False
        self.samples = []
        self.last_status_state = None
        self.last_status_publish_time = -1.0e9
        if delete_markers:
            array = MarkerArray()
            marker = Marker()
            marker.action = Marker.DELETEALL
            array.markers.append(marker)
            self.marker_pub.publish(array)
        self.publish_status('reset', force=True)

    def start_disturbance(self, now: float) -> None:
        self.scheduled_time = None
        self.trigger_count += 1
        self.disturbance_start_time = now
        self.disturbance_end_time = now + self.duration_s
        self.state = 'crosswind_active'
        self.publish_status(
            'crosswind_active',
            force=True,
            force_magnitude_n=self.force_magnitude_n,
            crosswind_unit=self.crosswind_unit,
            trigger_index=self.trigger_count,
        )
        self.get_logger().warning(
            'CROSSWIND ACTIVE DURING FLIGHT: '
            f'{self.force_magnitude_n:.2f}N, '
            f'direction={self.crosswind_unit}'
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
        force: Point3,
        torque_z_nm: float,
        active: bool,
    ) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.wrench.force.x = force[0]
        message.wrench.force.y = force[1]
        message.wrench.force.z = force[2]
        message.wrench.torque.z = torque_z_nm
        self.wrench_pub.publish(message)

        active_message = Bool()
        active_message.data = active
        self.active_pub.publish(active_message)

    def publish_zero_wrench(self) -> None:
        self.publish_wrench((0.0, 0.0, 0.0), 0.0, False)

    def request_replan(self) -> None:
        if self.replan_requested or self.latest_raw_goal is None:
            return
        message = self.copy_pose(self.latest_raw_goal)
        message.header.stamp = self.get_clock().now().to_msg()
        self.replan_goal_pub.publish(message)
        self.replan_request_time = self.now_s()
        self.replan_requested = True
        self.state = 'replan_requested'
        self.publish_status(
            'replan_requested',
            force=True,
            target=[
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ],
        )
        self.get_logger().warning(
            'DEVIATION DETECTED -> REPUBLISHED CURRENT TARGET -> '
            'A* REPLANNING FROM DISTURBED ODOM'
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

    def make_header(self):
        header = WrenchStamped().header
        header.frame_id = 'world'
        header.stamp = self.get_clock().now().to_msg()
        return header

    def add_path_marker(
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
        marker.ns = 'cq01_dynamic_replan'
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
        size: float,
        color: Tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header = self.make_header()
        marker.ns = 'cq01_dynamic_replan'
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
        marker.text = text
        array.markers.append(marker)

    def publish_markers(self, force: Point3) -> None:
        if self.latest_position is None:
            return
        array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        self.add_path_marker(
            array,
            1,
            self.old_path,
            0.32,
            (1.0, 0.55, 0.05, 0.72),
        )
        self.add_path_marker(
            array,
            2,
            self.new_path,
            0.52,
            (0.10, 1.0, 0.20, 1.0),
        )
        self.add_path_marker(
            array,
            3,
            self.actual_trail,
            0.28,
            (1.0, 0.05, 0.85, 0.95),
        )

        old_distance, old_nearest = self.nearest_path_distance(
            self.latest_position,
            self.old_path,
        )
        if old_nearest is not None:
            deviation = Marker()
            deviation.header = self.make_header()
            deviation.ns = 'cq01_dynamic_replan'
            deviation.id = 4
            deviation.type = Marker.ARROW
            deviation.action = Marker.ADD
            deviation.points = [
                Point(
                    x=old_nearest[0],
                    y=old_nearest[1],
                    z=old_nearest[2],
                ),
                Point(
                    x=self.latest_position[0],
                    y=self.latest_position[1],
                    z=self.latest_position[2],
                ),
            ]
            deviation.scale.x = 0.22
            deviation.scale.y = 0.48
            deviation.scale.z = 0.62
            deviation.color.r = 0.05
            deviation.color.g = 1.0
            deviation.color.b = 1.0
            deviation.color.a = 1.0
            array.markers.append(deviation)

        if any(abs(value) > 1.0e-6 for value in force):
            arrow = Marker()
            arrow.header = self.make_header()
            arrow.ns = 'cq01_dynamic_replan'
            arrow.id = 5
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(
                    x=self.latest_position[0],
                    y=self.latest_position[1],
                    z=self.latest_position[2] + 0.5,
                ),
                Point(
                    x=self.latest_position[0]
                    + self.visual_force_scale * force[0],
                    y=self.latest_position[1]
                    + self.visual_force_scale * force[1],
                    z=self.latest_position[2] + 0.5,
                ),
            ]
            arrow.scale.x = 0.55
            arrow.scale.y = 1.15
            arrow.scale.z = 1.45
            arrow.color.r = 1.0
            arrow.color.g = 0.05
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            array.markers.append(arrow)

        # Fixed spatial trigger gate. The aircraft enters this circle while
        # already visible in the fixed world-frame camera.
        gate = Marker()
        gate.header = self.make_header()
        gate.ns = 'cq01_dynamic_replan'
        gate.id = 29
        gate.type = Marker.CYLINDER
        gate.action = Marker.ADD
        gate.pose.position.x = self.trigger_gate[0]
        gate.pose.position.y = self.trigger_gate[1]
        gate.pose.position.z = self.trigger_gate[2] - 0.2
        gate.pose.orientation.w = 1.0
        gate.scale.x = 2.0 * self.trigger_gate_radius_xy_m
        gate.scale.y = 2.0 * self.trigger_gate_radius_xy_m
        gate.scale.z = 0.18
        gate.color.r = 0.95
        gate.color.g = 0.85
        gate.color.b = 0.05
        gate.color.a = 0.18
        array.markers.append(gate)

        self.add_text(
            array,
            28,
            'AUTO TRIGGER GATE',
            (
                self.trigger_gate[0],
                self.trigger_gate[1],
                self.trigger_gate[2] + 2.0,
            ),
            0.85,
            (1.0, 0.90, 0.10, 1.0),
        )

        # Highlight the existing V12 planning obstacle as the route fork.
        fork = Marker()
        fork.header = self.make_header()
        fork.ns = 'cq01_dynamic_replan'
        fork.id = 30
        fork.type = Marker.CYLINDER
        fork.action = Marker.ADD
        fork.pose.position.x = self.fork_center[0]
        fork.pose.position.y = self.fork_center[1]
        fork.pose.position.z = (
            self.focus_position[2]
            if self.focus_position is not None
            else self.latest_position[2]
        )
        fork.pose.orientation.w = 1.0
        fork.scale.x = 5.0
        fork.scale.y = 5.0
        fork.scale.z = 0.35
        fork.color.r = 1.0
        fork.color.g = 0.15
        fork.color.b = 0.05
        fork.color.a = 0.55
        array.markers.append(fork)

        branch_offset = 7.0
        left_position = (
            self.fork_center[0]
            + branch_offset * self.fork_normal[0],
            self.fork_center[1]
            + branch_offset * self.fork_normal[1],
            (self.focus_position[2]
             if self.focus_position is not None
             else self.latest_position[2]) + 2.0,
        )
        right_position = (
            self.fork_center[0]
            - branch_offset * self.fork_normal[0],
            self.fork_center[1]
            - branch_offset * self.fork_normal[1],
            (self.focus_position[2]
             if self.focus_position is not None
             else self.latest_position[2]) + 2.0,
        )
        self.add_text(
            array,
            31,
            'LEFT ROUTE',
            left_position,
            0.95,
            (0.95, 0.95, 0.95, 1.0),
        )
        self.add_text(
            array,
            32,
            'RIGHT ROUTE',
            right_position,
            0.95,
            (0.95, 0.95, 0.95, 1.0),
        )

        if self.latest_raw_goal is not None:
            goal = Marker()
            goal.header = self.make_header()
            goal.ns = 'cq01_dynamic_replan'
            goal.id = 6
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position = self.latest_raw_goal.pose.position
            goal.pose.orientation.w = 1.0
            goal.scale.x = 2.0
            goal.scale.y = 2.0
            goal.scale.z = 2.0
            goal.color.r = 0.10
            goal.color.g = 1.0
            goal.color.b = 0.20
            goal.color.a = 0.65
            array.markers.append(goal)

        anchor = self.focus_position or self.latest_position
        ax, ay, az = anchor
        if self.state == 'armed_waiting_for_gate':
            label = (
                'ARMED | WAITING FOR AIRCRAFT AT GATE | '
                f'AUTO SEQUENCE {self.trigger_count + 1}/'
                f'{self.maximum_trigger_count}'
            )
            color = (0.85, 0.85, 0.85, 1.0)
        elif (
            self.state in (
                'scheduled',
                'gate_countdown',
                'repeat_countdown',
            )
            and self.scheduled_time is not None
        ):
            remaining = max(0.0, self.scheduled_time - self.now_s())
            label = (
                f'OLD ROUTE {self.old_route_side_name} | '
                f'WIND TO OPPOSITE SIDE IN {remaining:.1f}s'
            )
            color = (1.0, 0.85, 0.10, 1.0)
        elif self.state == 'crosswind_active':
            label = (
                f'CROSSWIND {self.force_magnitude_n:.2f}N | '
                f'PUSHING OFF {self.old_route_side_name} ROUTE | '
                f'{self.current_old_path_deviation_m:.2f}m'
            )
            color = (1.0, 0.45, 0.05, 1.0)
        elif self.state == 'replan_requested':
            label = (
                f'DEVIATED {self.peak_old_path_deviation_m:.2f}m | '
                'A* REPLANNING TO SAME TARGET'
            )
            color = (0.10, 1.0, 1.0, 1.0)
        elif self.state in ('route_switch_ready', 'new_path_same_branch', 'tracking_replanned_path'):
            latency_ms = 1000.0 * (self.replan_latency_s or 0.0)
            if self.route_switch_confirmed:
                label = (
                    f'ROUTE SWITCH '
                    f'{self.old_route_side_name}'
                    f'->{self.new_route_side_name} | '
                    f'RUN {self.trigger_count}/'
                    f'{self.maximum_trigger_count} | '
                    f'NEXT {self.pending_repeat_count}'
                )
                color = (0.15, 1.0, 0.25, 1.0)
            else:
                label = (
                    f'REPLAN SAME BRANCH | '
                    f'{self.old_route_side_name}'
                    f'->{self.new_route_side_name} | '
                    f'{self.new_path_branch_m:.1f}m'
                )
                color = (1.0, 0.55, 0.05, 1.0)
        elif self.state == 'replan_failed':
            label = 'A* REPLAN FAILED'
            color = (1.0, 0.05, 0.05, 1.0)
        else:
            label = 'DYNAMIC CROSSWIND REPLAN READY'
            color = (0.85, 0.85, 0.85, 1.0)

        self.add_text(
            array,
            20,
            label,
            (ax, ay, az + 8.0),
            1.15,
            color,
        )
        self.marker_pub.publish(array)

    def timer_callback(self) -> None:
        now = self.now_s()
        self.publish_focus_transform()
        markers_due = self.marker_update_due(now)
        applied_force = (0.0, 0.0, 0.0)

        if (
            self.armed
            and self.state == 'armed_waiting_for_gate'
            and self.inside_trigger_gate()
        ):
            self.enter_trigger_gate()

        if (
            self.pending_repeat_count > 0
            and self.repeat_ready_state()
            and self.last_cycle_end_time is not None
            and now - self.last_cycle_end_time
            >= self.repeat_cooldown_s
        ):
            if self.inside_repeat_zone():
                self.pending_repeat_count -= 1
                if not self.schedule_repeat_now():
                    self.pending_repeat_count += 1
            else:
                self.publish_status(
                    'queued_repeat_waiting_for_visible_zone',
                    force=True,
                    pending_repeat_count=
                        self.pending_repeat_count,
                )

        if (
            self.scheduled_time is not None
            and now >= self.scheduled_time
        ):
            self.start_disturbance(now)

        if self.is_disturbance_active(now):
            elapsed = now - float(self.disturbance_start_time)
            gain = max(0.0, min(1.0, self.envelope(elapsed)))
            applied_force = (
                gain * self.force_magnitude_n * self.crosswind_unit[0],
                gain * self.force_magnitude_n * self.crosswind_unit[1],
                0.0,
            )
            self.publish_wrench(
                applied_force,
                gain * self.torque_z_nm,
                True,
            )
            self.state = 'crosswind_active'
            self.publish_status(
                'crosswind_active',
                applied_force_n=applied_force,
            )
        else:
            self.publish_zero_wrench()

        if (
            self.disturbance_end_time is not None
            and now > self.disturbance_end_time
            and not self.replan_requested
        ):
            enough_deviation = (
                self.peak_old_path_deviation_m
                >= self.minimum_replan_deviation_m
            )
            timeout = (
                now - self.disturbance_end_time
                >= self.replan_force_timeout_s
            )
            delay_elapsed = (
                now - self.disturbance_end_time
                >= self.replan_delay_after_force_s
            )
            if delay_elapsed and (enough_deviation or timeout):
                self.request_replan()

        if markers_due and self.state != 'idle':
            self.publish_markers(applied_force)

    def compute_metrics(self, final_state: str) -> Dict:
        applied_force_samples = [
            sample['force_norm_n']
            for sample in self.samples
            if sample['force_norm_n'] > 0.05
        ]
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
        criteria = {
            'disturbance_applied':
                max(applied_force_samples, default=0.0) > 0.5,
            'visible_route_deviation':
                self.peak_old_path_deviation_m
                >= self.minimum_replan_deviation_m,
            'replan_requested': self.replan_requested,
            'new_path_received': self.new_path_received,
            'route_switch_confirmed':
                self.route_switch_confirmed,
            'visible_route_branch':
                self.new_path_branch_m
                >= self.minimum_route_branch_m,
            'tilt_pass': max_tilt <= self.maximum_tilt_deg,
            'rpm_saturation_pass':
                saturation_fraction
                <= self.maximum_saturation_fraction,
            'mission_continued_to_completion':
                self.mission_completed_after_replan,
        }
        return {
            'experiment':
                'cq01_crosswind_dynamic_astar_replan',
            'final_state': final_state,
            'force_magnitude_n': self.force_magnitude_n,
            'trigger_count': self.trigger_count,
            'pending_repeat_count':
                self.pending_repeat_count,
            'maximum_trigger_count':
                self.maximum_trigger_count,
            'force_direction_unit': self.crosswind_unit,
            'duration_s': self.duration_s,
            'peak_old_path_deviation_m':
                self.peak_old_path_deviation_m,
            'new_path_branch_m': self.new_path_branch_m,
            'old_route_side': self.old_route_side_name,
            'new_route_side': self.new_route_side_name,
            'route_switch_confirmed':
                self.route_switch_confirmed,
            'minimum_route_branch_m':
                self.minimum_route_branch_m,
            'replan_latency_s': self.replan_latency_s,
            'retracking_time_s': self.retracking_time_s,
            'maximum_tilt_deg': max_tilt,
            'rpm_saturation_fraction': saturation_fraction,
            'mission_completed_after_replan':
                self.mission_completed_after_replan,
            'criteria': criteria,
            'pass': all(criteria.values()),
            'sample_count': len(self.samples),
        }

    def save_results(self, final_state: str) -> None:
        if self.saved_output_dir is not None:
            return
        summary = self.compute_metrics(final_state)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = (
            self.results_root
            / f'cq01_dynamic_replan_{timestamp}'
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
                if self.replan_request_time is not None:
                    replan_relative = (
                        self.replan_request_time
                        - (self.disturbance_start_time or 0.0)
                    )
                    plt.axvline(
                        replan_relative,
                        linestyle='--',
                        label='Replan requested',
                    )
                plt.xlabel('Time after crosswind start (s)')
                plt.ylabel('Path deviation (m)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'deviation_and_retracking.png',
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
                plt.xlabel('Time after crosswind start (s)')
                plt.ylabel('Force (N)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'crosswind_profile.png',
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
                plt.xlabel('Time after crosswind start (s)')
                plt.ylabel('Attitude (deg)')
                plt.legend()
                plt.tight_layout()
                plt.savefig(
                    output_dir / 'attitude_response.png',
                    dpi=180,
                )
                plt.close()
        except Exception as exception:
            self.get_logger().warning(
                f'Plot generation failed: {exception}'
            )

        self.saved_output_dir = output_dir
        self.get_logger().info(
            'CQ-01 dynamic replan result: '
            f'pass={summary["pass"]}, '
            f'peak_deviation='
            f'{summary["peak_old_path_deviation_m"]:.2f}m, '
            f'branch={summary["new_path_branch_m"]:.2f}m, '
            f'replan_latency={summary["replan_latency_s"]}'
        )
        self.get_logger().info(
            f'Results saved to: {output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01DynamicReplanDisturbanceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.state != 'idle':
            node.save_results(final_state='keyboard_interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
