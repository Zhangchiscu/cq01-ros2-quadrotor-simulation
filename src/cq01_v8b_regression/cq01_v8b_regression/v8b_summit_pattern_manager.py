#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


def smoothstep5(value: float) -> float:
    u = max(0.0, min(1.0, value))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def interpolate(
    first: Point3,
    second: Point3,
    ratio: float,
) -> Point3:
    return tuple(
        first[index]
        + ratio * (second[index] - first[index])
        for index in range(3)
    )


class SummitPatternTrajectoryManager(Node):
    """Fast summit ascent followed by compact circle and figure-eight."""

    def __init__(self) -> None:
        super().__init__(
            'cq01_summit_pattern_trajectory_manager'
        )

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'goal_topic',
            '/drone/goal_pattern',
        )
        self.declare_parameter(
            'status_topic',
            '/cq01_pattern/status',
        )
        self.declare_parameter(
            'reference_path_topic',
            '/cq01_pattern/reference_path',
        )
        self.declare_parameter(
            'marker_topic',
            '/cq01_pattern/reference_markers',
        )

        # P15 is the terminal ridge waypoint in the proven V3.4G route.
        self.declare_parameter('summit_x_m', 410.0)
        self.declare_parameter('summit_y_m', 45.0)
        self.declare_parameter('summit_z_m', 219.878)

        # The ascent endpoint is also the common start of both patterns.
        # Center=(400,45,250), radius=10 -> start=(410,45,250).
        self.declare_parameter('center_x_m', 400.0)
        self.declare_parameter('center_y_m', 45.0)
        self.declare_parameter('center_z_m', 250.0)
        self.declare_parameter('circle_radius_m', 10.0)
        self.declare_parameter('circle_period_s', 16.0)
        self.declare_parameter('figure8_x_radius_m', 10.0)
        self.declare_parameter('figure8_y_radius_m', 6.5)
        self.declare_parameter('figure8_period_s', 22.0)

        self.declare_parameter('entry_tolerance_m', 1.5)
        self.declare_parameter('summit_hold_s', 0.5)
        self.declare_parameter('rapid_ascent_s', 8.0)
        self.declare_parameter('top_hold_s', 0.8)
        self.declare_parameter('between_pattern_hold_s', 0.6)
        self.declare_parameter('final_hold_s', 3.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.summit_base = (
            float(self.get_parameter('summit_x_m').value),
            float(self.get_parameter('summit_y_m').value),
            float(self.get_parameter('summit_z_m').value),
        )
        self.center = (
            float(self.get_parameter('center_x_m').value),
            float(self.get_parameter('center_y_m').value),
            float(self.get_parameter('center_z_m').value),
        )
        self.circle_radius = float(
            self.get_parameter('circle_radius_m').value
        )
        self.circle_period = max(
            8.0,
            float(
                self.get_parameter(
                    'circle_period_s'
                ).value
            ),
        )
        self.figure8_x_radius = float(
            self.get_parameter(
                'figure8_x_radius_m'
            ).value
        )
        self.figure8_y_radius = float(
            self.get_parameter(
                'figure8_y_radius_m'
            ).value
        )
        self.figure8_period = max(
            10.0,
            float(
                self.get_parameter(
                    'figure8_period_s'
                ).value
            ),
        )

        self.entry_tolerance = max(
            0.2,
            float(
                self.get_parameter(
                    'entry_tolerance_m'
                ).value
            ),
        )
        self.summit_hold_s = max(
            0.0,
            float(
                self.get_parameter('summit_hold_s').value
            ),
        )
        self.rapid_ascent_s = max(
            3.0,
            float(
                self.get_parameter('rapid_ascent_s').value
            ),
        )
        self.top_hold_s = max(
            0.0,
            float(self.get_parameter('top_hold_s').value),
        )
        self.between_hold_s = max(
            0.0,
            float(
                self.get_parameter(
                    'between_pattern_hold_s'
                ).value
            ),
        )
        self.final_hold_s = max(
            1.0,
            float(
                self.get_parameter('final_hold_s').value
            ),
        )

        self.pattern_start = (
            self.center[0] + self.circle_radius,
            self.center[1],
            self.center[2],
        )

        if math.dist(
            self.pattern_start[:2],
            self.summit_base[:2],
        ) > 1.0e-6:
            raise RuntimeError(
                'Pattern start XY must be directly above P15.'
            )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            20,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient_qos,
        )
        self.reference_path_pub = self.create_publisher(
            Path,
            str(
                self.get_parameter(
                    'reference_path_topic'
                ).value
            ),
            transient_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            transient_qos,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            100,
        )
        self.create_service(
            Trigger,
            '~/start',
            self.start_callback,
        )

        rate_hz = max(
            10.0,
            float(
                self.get_parameter(
                    'publish_rate_hz'
                ).value
            ),
        )
        self.timer = self.create_timer(
            1.0 / rate_hz,
            self.timer_callback,
        )

        self.latest_odom: Optional[Odometry] = None
        self.active = False
        self.completed = False
        self.phase = 'idle'
        self.phase_start_s: Optional[float] = None
        self.sequence_start_s: Optional[float] = None
        self.reference_goal = self.summit_base

        self.publish_reference_path()
        self.publish_markers()
        self.publish_status()

        self.get_logger().info(
            'Summit pattern manager ready: '
            f'P15={self.summit_base}, '
            f'rapid ascent={self.pattern_start[2] - self.summit_base[2]:.3f}m/'
            f'{self.rapid_ascent_s:.1f}s, '
            f'circle={self.circle_period:.1f}s, '
            f'figure8={self.figure8_period:.1f}s.'
        )

    def now_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds / 1.0e9
        )

    @staticmethod
    def position_from_odom(message: Odometry) -> Point3:
        point = message.pose.pose.position
        return (
            float(point.x),
            float(point.y),
            float(point.z),
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def start_callback(self, _request, response):
        if self.latest_odom is None:
            response.success = False
            response.message = 'No odometry available'
            return response

        entry_error = math.dist(
            self.position_from_odom(self.latest_odom),
            self.summit_base,
        )
        if entry_error > self.entry_tolerance:
            response.success = False
            response.message = (
                f'P15 entry error {entry_error:.3f}m '
                f'exceeds {self.entry_tolerance:.3f}m'
            )
            return response

        self.active = True
        self.completed = False
        self.phase = 'summit_hold'
        self.phase_start_s = self.now_s()
        self.sequence_start_s = self.phase_start_s
        self.reference_goal = self.summit_base
        self.publish_status()

        response.success = True
        response.message = (
            'Summit rapid-ascent pattern sequence started'
        )
        self.get_logger().info(
            f'P15 accepted at error={entry_error:.3f}m.'
        )
        return response

    def phase_elapsed(self) -> float:
        if self.phase_start_s is None:
            return 0.0
        return max(0.0, self.now_s() - self.phase_start_s)

    def begin_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_start_s = self.now_s()
        self.get_logger().info(
            f'Summit demonstration phase -> {phase}'
        )

    def circle_reference(self, progress: float) -> Point3:
        theta = 2.0 * math.pi * smoothstep5(progress)
        return (
            self.center[0]
            + self.circle_radius * math.cos(theta),
            self.center[1]
            + self.circle_radius * math.sin(theta),
            self.center[2],
        )

    def figure8_reference(self, progress: float) -> Point3:
        theta = 2.0 * math.pi * smoothstep5(progress)
        return (
            self.center[0]
            + self.figure8_x_radius * math.cos(theta),
            self.center[1]
            + self.figure8_y_radius * math.sin(2.0 * theta),
            self.center[2],
        )

    def timer_callback(self) -> None:
        if not self.active:
            self.publish_status()
            return

        elapsed = self.phase_elapsed()

        if self.phase == 'summit_hold':
            self.reference_goal = self.summit_base
            if elapsed >= self.summit_hold_s:
                self.begin_phase('rapid_ascent')

        elif self.phase == 'rapid_ascent':
            progress = max(
                0.0,
                min(1.0, elapsed / self.rapid_ascent_s),
            )
            self.reference_goal = interpolate(
                self.summit_base,
                self.pattern_start,
                smoothstep5(progress),
            )
            if progress >= 1.0:
                self.reference_goal = self.pattern_start
                self.begin_phase('top_hold')

        elif self.phase == 'top_hold':
            self.reference_goal = self.pattern_start
            if elapsed >= self.top_hold_s:
                self.begin_phase('circle')

        elif self.phase == 'circle':
            progress = max(
                0.0,
                min(1.0, elapsed / self.circle_period),
            )
            self.reference_goal = self.circle_reference(
                progress
            )
            if progress >= 1.0:
                self.reference_goal = self.pattern_start
                self.begin_phase('circle_hold')

        elif self.phase == 'circle_hold':
            self.reference_goal = self.pattern_start
            if elapsed >= self.between_hold_s:
                self.begin_phase('figure8')

        elif self.phase == 'figure8':
            progress = max(
                0.0,
                min(1.0, elapsed / self.figure8_period),
            )
            self.reference_goal = self.figure8_reference(
                progress
            )
            if progress >= 1.0:
                self.reference_goal = self.pattern_start
                self.begin_phase('final_hold')

        elif self.phase == 'final_hold':
            self.reference_goal = self.pattern_start
            if elapsed >= self.final_hold_s:
                self.active = False
                self.completed = True
                self.phase = 'pattern_completed'
                self.phase_start_s = self.now_s()
                self.get_logger().info(
                    'Summit rapid-ascent, circle and '
                    'figure-eight demonstration completed.'
                )

        if self.active:
            self.publish_goal()
        self.publish_status()
        self.publish_markers()

    def phase_progress(self) -> float:
        elapsed = self.phase_elapsed()
        duration = {
            'rapid_ascent': self.rapid_ascent_s,
            'circle': self.circle_period,
            'figure8': self.figure8_period,
        }.get(self.phase)
        if duration is None:
            return 0.0
        return max(0.0, min(1.0, elapsed / duration))

    def publish_goal(self) -> None:
        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.pose.position.x = self.reference_goal[0]
        message.pose.position.y = self.reference_goal[1]
        message.pose.position.z = self.reference_goal[2]
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def publish_status(self) -> None:
        payload = {
            'state': (
                'pattern_completed'
                if self.completed
                else (
                    'running'
                    if self.active
                    else self.phase
                )
            ),
            'phase': self.phase,
            'phase_progress': self.phase_progress(),
            'elapsed_s': (
                0.0
                if self.sequence_start_s is None
                else self.now_s() - self.sequence_start_s
            ),
            'reference': list(self.reference_goal),
            'summit_base': list(self.summit_base),
            'center': list(self.center),
            'pattern_start': list(self.pattern_start),
            'rapid_ascent_height_m':
                self.pattern_start[2] - self.summit_base[2],
            'rapid_ascent_s': self.rapid_ascent_s,
            'circle_radius_m': self.circle_radius,
            'circle_period_s': self.circle_period,
            'figure8_x_radius_m':
                self.figure8_x_radius,
            'figure8_y_radius_m':
                self.figure8_y_radius,
            'figure8_period_s': self.figure8_period,
            'circle_complete':
                self.phase in (
                    'circle_hold',
                    'figure8',
                    'final_hold',
                    'pattern_completed',
                ),
            'figure8_complete':
                self.phase in (
                    'final_hold',
                    'pattern_completed',
                ),
            'final_state': (
                'pattern_completed'
                if self.completed
                else ''
            ),
        }
        message = String()
        message.data = json.dumps(payload)
        self.status_pub.publish(message)

    def publish_reference_path(self) -> None:
        message = Path()
        message.header.frame_id = self.frame_id
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # The vertical ascent is included as part of the single reference path.
        for index in range(31):
            ratio = index / 30.0
            point = interpolate(
                self.summit_base,
                self.pattern_start,
                ratio,
            )
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        for index in range(121):
            theta = 2.0 * math.pi * index / 120.0
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = (
                self.center[0]
                + self.circle_radius * math.cos(theta)
            )
            pose.pose.position.y = (
                self.center[1]
                + self.circle_radius * math.sin(theta)
            )
            pose.pose.position.z = self.center[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        for index in range(161):
            theta = 2.0 * math.pi * index / 160.0
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = (
                self.center[0]
                + self.figure8_x_radius
                * math.cos(theta)
            )
            pose.pose.position.y = (
                self.center[1]
                + self.figure8_y_radius
                * math.sin(2.0 * theta)
            )
            pose.pose.position.z = self.center[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.reference_path_pub.publish(message)

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

    def line_marker(
        self,
        marker_id: int,
        namespace: str,
        points,
        color: ColorRGBA,
        width: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color = color
        marker.points = [
            self.point(point) for point in points
        ]
        return marker

    def publish_markers(self) -> None:
        array = MarkerArray()

        ascent_points = [
            interpolate(
                self.summit_base,
                self.pattern_start,
                index / 30.0,
            )
            for index in range(31)
        ]
        array.markers.append(
            self.line_marker(
                1,
                'rapid_ascent_reference',
                ascent_points,
                self.rgba(1.0, 0.22, 0.08, 1.0),
                0.85,
            )
        )

        circle_points = [
            (
                self.center[0]
                + self.circle_radius
                * math.cos(
                    2.0 * math.pi * index / 120.0
                ),
                self.center[1]
                + self.circle_radius
                * math.sin(
                    2.0 * math.pi * index / 120.0
                ),
                self.center[2],
            )
            for index in range(121)
        ]
        array.markers.append(
            self.line_marker(
                2,
                'summit_circle_reference',
                circle_points,
                self.rgba(0.15, 1.0, 0.30, 0.98),
                0.70,
            )
        )

        figure8_points = [
            (
                self.center[0]
                + self.figure8_x_radius
                * math.cos(
                    2.0 * math.pi * index / 160.0
                ),
                self.center[1]
                + self.figure8_y_radius
                * math.sin(
                    4.0 * math.pi * index / 160.0
                ),
                self.center[2],
            )
            for index in range(161)
        ]
        array.markers.append(
            self.line_marker(
                3,
                'summit_figure8_reference',
                figure8_points,
                self.rgba(0.10, 0.80, 1.0, 0.98),
                0.70,
            )
        )

        moving = Marker()
        moving.header.frame_id = self.frame_id
        moving.header.stamp = (
            self.get_clock().now().to_msg()
        )
        moving.ns = 'summit_moving_reference'
        moving.id = 4
        moving.type = Marker.SPHERE
        moving.action = Marker.ADD
        moving.pose.position = self.point(
            self.reference_goal
        )
        moving.pose.orientation.w = 1.0
        moving.scale.x = 2.6
        moving.scale.y = 2.6
        moving.scale.z = 2.6
        moving.color = self.rgba(
            1.0, 0.18, 0.10, 1.0
        )
        array.markers.append(moving)

        text = Marker()
        text.header = moving.header
        text.ns = 'summit_demonstration_status'
        text.id = 5
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.center[0]
        text.pose.position.y = self.center[1]
        text.pose.position.z = self.center[2] + 12.0
        text.pose.orientation.w = 1.0
        text.scale.z = 3.8
        text.color = self.rgba(
            1.0, 1.0, 1.0, 1.0
        )
        text.text = (
            'SUMMIT FINALE\n'
            f'PHASE: {self.phase.upper()}'
        )
        array.markers.append(text)

        self.marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SummitPatternTrajectoryManager()
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
