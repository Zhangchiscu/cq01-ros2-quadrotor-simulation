#!/usr/bin/env python3

import math
import random
from typing import List

import rclpy
from geometry_msgs.msg import Point, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class DisturbanceInjector(Node):
    """Publish configurable external force/torque profiles."""

    def __init__(self) -> None:
        super().__init__("disturbance_injector")

        self.declare_parameter("mode", "pulse")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("start_delay_s", 8.0)
        self.declare_parameter("duration_s", 1.0)
        self.declare_parameter("ramp_time_s", 0.15)

        self.declare_parameter("force_x_n", 4.0)
        self.declare_parameter("force_y_n", 0.0)
        self.declare_parameter("force_z_n", 0.0)
        self.declare_parameter("torque_x_nm", 0.0)
        self.declare_parameter("torque_y_nm", 0.0)
        self.declare_parameter("torque_z_nm", 0.0)

        self.declare_parameter("sine_frequency_hz", 0.5)
        self.declare_parameter("random_seed", 2026)
        self.declare_parameter("random_force_std_x_n", 1.5)
        self.declare_parameter("random_force_std_y_n", 1.5)
        self.declare_parameter("random_force_std_z_n", 0.3)
        self.declare_parameter("random_torque_std_x_nm", 0.02)
        self.declare_parameter("random_torque_std_y_nm", 0.02)
        self.declare_parameter("random_torque_std_z_nm", 0.01)
        self.declare_parameter("gust_correlation_time_s", 0.5)
        self.declare_parameter("marker_scale_m_per_n", 0.25)

        self.mode = str(self.get_parameter("mode").value).lower()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.start_delay_s = float(
            self.get_parameter("start_delay_s").value
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.ramp_time_s = float(self.get_parameter("ramp_time_s").value)

        self.force = [
            float(self.get_parameter("force_x_n").value),
            float(self.get_parameter("force_y_n").value),
            float(self.get_parameter("force_z_n").value),
        ]
        self.torque = [
            float(self.get_parameter("torque_x_nm").value),
            float(self.get_parameter("torque_y_nm").value),
            float(self.get_parameter("torque_z_nm").value),
        ]

        self.sine_frequency_hz = float(
            self.get_parameter("sine_frequency_hz").value
        )
        self.seed = int(self.get_parameter("random_seed").value)
        self.random_force_std = [
            float(self.get_parameter("random_force_std_x_n").value),
            float(self.get_parameter("random_force_std_y_n").value),
            float(self.get_parameter("random_force_std_z_n").value),
        ]
        self.random_torque_std = [
            float(self.get_parameter("random_torque_std_x_nm").value),
            float(self.get_parameter("random_torque_std_y_nm").value),
            float(self.get_parameter("random_torque_std_z_nm").value),
        ]
        self.gust_tau = float(
            self.get_parameter("gust_correlation_time_s").value
        )
        self.marker_scale = float(
            self.get_parameter("marker_scale_m_per_n").value
        )

        valid_modes = {"off", "pulse", "constant", "sine", "random_gust"}
        if self.mode not in valid_modes:
            raise RuntimeError(
                f"Unsupported disturbance mode '{self.mode}'. "
                f"Valid modes: {sorted(valid_modes)}"
            )
        if self.publish_rate_hz <= 0.0:
            raise RuntimeError("publish_rate_hz must be positive")
        if self.start_delay_s < 0.0 or self.duration_s < 0.0:
            raise RuntimeError("start_delay_s and duration_s must be non-negative")
        if self.ramp_time_s < 0.0:
            raise RuntimeError("ramp_time_s must be non-negative")
        if self.gust_tau <= 0.0:
            raise RuntimeError("gust_correlation_time_s must be positive")

        self.random = random.Random(self.seed)
        self.filtered_force = [0.0, 0.0, 0.0]
        self.filtered_torque = [0.0, 0.0, 0.0]
        self.latest_position = [0.0, 0.0, 0.0]
        self.start_ns = self.get_clock().now().nanoseconds
        self.was_active = False

        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            "drone/external_wrench",
            20,
        )
        self.active_pub = self.create_publisher(
            Bool,
            "drone/disturbance_active",
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            "drone/disturbance_marker",
            10,
        )
        self.create_subscription(
            Odometry,
            "drone/odom",
            self.odom_callback,
            20,
        )

        self.period_s = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(self.period_s, self.timer_callback)

        self.get_logger().info(
            "Disturbance injector started: "
            f"mode={self.mode}, frame={self.frame_id}, "
            f"start={self.start_delay_s:.2f}s, "
            f"duration={self.duration_s:.2f}s, force={self.force} N"
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_position = [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ]

    def elapsed_s(self) -> float:
        return (
            self.get_clock().now().nanoseconds - self.start_ns
        ) / 1.0e9

    def smooth_envelope(self, profile_time: float) -> float:
        if profile_time < 0.0 or profile_time > self.duration_s:
            return 0.0
        if self.duration_s <= 0.0:
            return 0.0

        ramp = min(self.ramp_time_s, 0.5 * self.duration_s)
        if ramp <= 1.0e-9:
            return 1.0
        if profile_time < ramp:
            return 0.5 - 0.5 * math.cos(math.pi * profile_time / ramp)
        if profile_time > self.duration_s - ramp:
            remaining = self.duration_s - profile_time
            return 0.5 - 0.5 * math.cos(math.pi * remaining / ramp)
        return 1.0

    def random_gust_step(self, std_values: List[float], state: List[float]) -> List[float]:
        alpha = math.exp(-self.period_s / self.gust_tau)
        noise_scale = math.sqrt(max(0.0, 1.0 - alpha * alpha))
        output = []
        for axis in range(3):
            sample = self.random.gauss(0.0, std_values[axis])
            value = alpha * state[axis] + noise_scale * sample
            output.append(value)
        return output

    def compute_profile(self, elapsed: float) -> tuple[List[float], List[float], bool]:
        profile_time = elapsed - self.start_delay_s
        active = 0.0 <= profile_time <= self.duration_s and self.mode != "off"
        if not active:
            self.filtered_force = [0.0, 0.0, 0.0]
            self.filtered_torque = [0.0, 0.0, 0.0]
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], False

        envelope = self.smooth_envelope(profile_time)

        if self.mode in {"pulse", "constant"}:
            force = [envelope * value for value in self.force]
            torque = [envelope * value for value in self.torque]
        elif self.mode == "sine":
            phase = 2.0 * math.pi * self.sine_frequency_hz * profile_time
            scale = envelope * math.sin(phase)
            force = [scale * value for value in self.force]
            torque = [scale * value for value in self.torque]
        elif self.mode == "random_gust":
            self.filtered_force = self.random_gust_step(
                self.random_force_std,
                self.filtered_force,
            )
            self.filtered_torque = self.random_gust_step(
                self.random_torque_std,
                self.filtered_torque,
            )
            force = [envelope * value for value in self.filtered_force]
            torque = [envelope * value for value in self.filtered_torque]
        else:
            force = [0.0, 0.0, 0.0]
            torque = [0.0, 0.0, 0.0]
            active = False

        return force, torque, active

    def publish_marker(self, force: List[float], active: bool) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "external_disturbance"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 0.90
        marker.color.g = 0.15
        marker.color.b = 0.05
        marker.color.a = 0.95 if active else 0.0

        start = Point()
        if self.frame_id == "map":
            start.x, start.y, start.z = self.latest_position
        else:
            start.x, start.y, start.z = 0.0, 0.0, 0.0
        end = Point()
        end.x = start.x + self.marker_scale * force[0]
        end.y = start.y + self.marker_scale * force[1]
        end.z = start.z + self.marker_scale * force[2]
        marker.points = [start, end]
        self.marker_pub.publish(marker)

    def timer_callback(self) -> None:
        elapsed = self.elapsed_s()
        force, torque, active = self.compute_profile(elapsed)

        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
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
        self.publish_marker(force, active)

        if active and not self.was_active:
            self.get_logger().info("External disturbance became active")
        if self.was_active and not active:
            self.get_logger().info("External disturbance ended")
        self.was_active = active


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisturbanceInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
