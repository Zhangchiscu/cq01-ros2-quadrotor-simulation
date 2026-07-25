#!/usr/bin/env python3
"""Publish smooth circle and figure-eight reference trajectories."""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

Point3 = Tuple[float, float, float]


def quintic_progress(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def yaw_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PatternExecutor(Node):
    def __init__(self) -> None:
        super().__init__("pattern_trajectory_executor")

        self.declare_parameter("pattern", "circle")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("altitude_m", 1.5)
        self.declare_parameter("radius_x_m", 2.0)
        self.declare_parameter("radius_y_m", 1.4)
        self.declare_parameter("period_s", 24.0)
        self.declare_parameter("laps", 1.0)
        self.declare_parameter("entry_duration_s", 6.0)
        self.declare_parameter("pre_pattern_hold_s", 1.5)
        self.declare_parameter("post_pattern_hold_s", 2.0)
        self.declare_parameter("start_delay_s", 0.8)
        self.declare_parameter("tangent_yaw", False)
        self.declare_parameter("reference_samples", 500)

        self.pattern = str(self.get_parameter("pattern").value)
        if self.pattern not in {"circle", "figure_eight"}:
            raise ValueError("pattern must be 'circle' or 'figure_eight'")

        self.control_rate = float(self.get_parameter("control_rate_hz").value)
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.altitude = float(self.get_parameter("altitude_m").value)
        self.radius_x = float(self.get_parameter("radius_x_m").value)
        self.radius_y = float(self.get_parameter("radius_y_m").value)
        self.period = float(self.get_parameter("period_s").value)
        self.laps = float(self.get_parameter("laps").value)
        self.entry_duration = float(self.get_parameter("entry_duration_s").value)
        self.pre_hold = float(self.get_parameter("pre_pattern_hold_s").value)
        self.post_hold = float(self.get_parameter("post_pattern_hold_s").value)
        self.start_delay = float(self.get_parameter("start_delay_s").value)
        self.tangent_yaw = bool(self.get_parameter("tangent_yaw").value)
        self.reference_samples = int(self.get_parameter("reference_samples").value)

        if self.control_rate <= 0.0 or self.period <= 0.0 or self.laps <= 0.0:
            raise ValueError("control rate, period and laps must be positive")
        if self.radius_x <= 0.0 or self.radius_y <= 0.0:
            raise ValueError("trajectory radii must be positive")

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(PoseStamped, "/drone/goal", 10)
        self.reference_publisher = self.create_publisher(
            Path, "/drone/pattern_reference", transient_qos
        )
        self.compat_reference_publisher = self.create_publisher(
            Path, "/drone/trajectory_reference", transient_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/drone/pattern/status", transient_qos
        )
        self.create_subscription(Odometry, "/drone/odom", self.odom_callback, 50)

        self.latest_odom: Optional[Odometry] = None
        self.start_position: Optional[Point3] = None
        self.started_at: Optional[float] = None
        self.phase = "waiting_for_odom"
        self.last_phase = ""
        self.complete_published = False
        self.reference_path_published = False

        self.timer = self.create_timer(1.0 / self.control_rate, self.step)
        self.get_logger().info(
            f"Pattern executor started: pattern={self.pattern}, "
            f"period={self.period:.1f}s, laps={self.laps:.2f}"
        )

    @property
    def pattern_duration(self) -> float:
        return self.period * self.laps

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        if self.start_position is None:
            position = message.pose.pose.position
            self.start_position = (float(position.x), float(position.y), float(position.z))
            self.started_at = time.monotonic() + self.start_delay
            self.publish_reference_path()

    def pattern_point(self, theta: float) -> Tuple[Point3, float]:
        if self.pattern == "circle":
            x = self.center_x + self.radius_x * math.cos(theta)
            y = self.center_y + self.radius_y * math.sin(theta)
            dx = -self.radius_x * math.sin(theta)
            dy = self.radius_y * math.cos(theta)
        else:
            x = self.center_x + self.radius_x * math.sin(theta)
            y = self.center_y + self.radius_y * math.sin(theta) * math.cos(theta)
            dx = self.radius_x * math.cos(theta)
            dy = self.radius_y * math.cos(2.0 * theta)

        yaw = math.atan2(dy, dx) if self.tangent_yaw else 0.0
        return (x, y, self.altitude), yaw

    def first_pattern_point(self) -> Tuple[Point3, float]:
        return self.pattern_point(0.0)

    def publish_reference_path(self) -> None:
        if self.start_position is None or self.reference_path_published:
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        first_point, first_yaw = self.first_pattern_point()
        entry_samples = max(30, int(self.entry_duration * self.control_rate))

        for index in range(entry_samples):
            ratio = index / max(1, entry_samples - 1)
            progress = quintic_progress(ratio)
            point = tuple(
                self.start_position[axis]
                + progress * (first_point[axis] - self.start_position[axis])
                for axis in range(3)
            )
            path.poses.append(self.make_pose(point, first_yaw))

        pattern_samples = max(100, self.reference_samples)
        for index in range(pattern_samples + 1):
            theta = 2.0 * math.pi * self.laps * index / pattern_samples
            point, yaw = self.pattern_point(theta)
            path.poses.append(self.make_pose(point, yaw))

        self.reference_publisher.publish(path)
        self.compat_reference_publisher.publish(path)
        self.reference_path_published = True

    def make_pose(self, point: Point3, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        qx, qy, qz, qw = yaw_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_goal(self, point: Point3, yaw: float) -> None:
        self.goal_publisher.publish(self.make_pose(point, yaw))

    def publish_status(self, state: str, elapsed: float, progress: float) -> None:
        if state != self.last_phase:
            self.get_logger().info(f"Pattern state: {state}")
            self.last_phase = state

        message = String()
        message.data = json.dumps({
            "state": state,
            "pattern": self.pattern,
            "elapsed_s": elapsed,
            "progress": max(0.0, min(1.0, progress)),
            "period_s": self.period,
            "laps": self.laps,
            "success": state == "pattern_complete",
        })
        self.status_publisher.publish(message)

    def step(self) -> None:
        if self.latest_odom is None or self.start_position is None or self.started_at is None:
            self.publish_status("waiting_for_odom", 0.0, 0.0)
            return

        now = time.monotonic()
        if now < self.started_at:
            point, yaw = self.first_pattern_point()
            self.publish_goal(self.start_position, yaw)
            self.publish_status("start_delay", 0.0, 0.0)
            return

        elapsed = now - self.started_at
        first_point, first_yaw = self.first_pattern_point()

        if elapsed < self.entry_duration:
            ratio = elapsed / max(self.entry_duration, 1.0e-6)
            progress = quintic_progress(ratio)
            target = tuple(
                self.start_position[axis]
                + progress * (first_point[axis] - self.start_position[axis])
                for axis in range(3)
            )
            self.publish_goal(target, first_yaw)
            self.publish_status("entry", elapsed, 0.0)
            return

        elapsed_after_entry = elapsed - self.entry_duration
        if elapsed_after_entry < self.pre_hold:
            self.publish_goal(first_point, first_yaw)
            self.publish_status("pre_pattern_hold", elapsed, 0.0)
            return

        pattern_elapsed = elapsed_after_entry - self.pre_hold
        if pattern_elapsed < self.pattern_duration:
            theta = 2.0 * math.pi * pattern_elapsed / self.period
            target, yaw = self.pattern_point(theta)
            self.publish_goal(target, yaw)
            self.publish_status(
                "pattern_running",
                elapsed,
                pattern_elapsed / self.pattern_duration,
            )
            return

        final_theta = 2.0 * math.pi * self.laps
        final_point, final_yaw = self.pattern_point(final_theta)
        self.publish_goal(final_point, final_yaw)

        post_elapsed = pattern_elapsed - self.pattern_duration
        if post_elapsed < self.post_hold:
            self.publish_status("post_pattern_hold", elapsed, 1.0)
            return

        self.publish_status("pattern_complete", elapsed, 1.0)
        self.complete_published = True


def main() -> None:
    rclpy.init()
    node = PatternExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
