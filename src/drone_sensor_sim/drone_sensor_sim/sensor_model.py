#!/usr/bin/env python3

from __future__ import annotations

import copy
import math
import random
from typing import Iterable, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_quaternion(
    w: float, x: float, y: float, z: float
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / norm, x / norm, y / norm, z / norm


def quaternion_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return normalize_quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def diagonal_covariance(
    linear_variances: Iterable[float],
    angular_variances: Iterable[float],
) -> list[float]:
    covariance = [0.0] * 36
    values = list(linear_variances) + list(angular_variances)
    for index, value in enumerate(values):
        covariance[index * 6 + index] = float(value)
    return covariance


class SensorModelNode(Node):
    """Create deterministic noisy odometry and IMU measurements.

    The dynamics node publishes ideal truth messages on /drone/odom and
    /drone/imu. This node applies a simplified sensor model and publishes
    /drone/odom_sensor and /drone/imu_sensor. The model supports white noise,
    initial bias, bias random walk, scale error, clipping and deterministic
    packet dropouts. Defaults are deliberately conservative and reproducible.
    """

    def __init__(self) -> None:
        super().__init__("sensor_model")
        self._declare_parameters()
        self._load_parameters()

        self.random = random.Random(self.seed)
        self.last_odom_stamp: float | None = None
        self.last_imu_stamp: float | None = None

        self.position_bias = list(self.position_bias_initial)
        self.velocity_bias = list(self.velocity_bias_initial)
        self.orientation_bias = list(self.orientation_bias_initial)
        self.angular_velocity_bias = list(self.angular_velocity_bias_initial)
        self.accelerometer_bias = list(self.accelerometer_bias_initial)
        self.gyroscope_bias = list(self.gyroscope_bias_initial)

        self.odom_publisher = self.create_publisher(
            Odometry, "/drone/odom_sensor", 20
        )
        self.imu_publisher = self.create_publisher(
            Imu, "/drone/imu_sensor", 20
        )

        self.odom_subscription = self.create_subscription(
            Odometry, "/drone/odom", self._odom_callback, 20
        )
        self.imu_subscription = self.create_subscription(
            Imu, "/drone/imu", self._imu_callback, 20
        )

        self.odom_received = 0
        self.odom_published = 0
        self.imu_received = 0
        self.imu_published = 0

        self.statistics_timer = self.create_timer(5.0, self._log_statistics)

        self.get_logger().info(
            f"Sensor model started: profile={self.profile} seed={self.seed}"
        )
        self.get_logger().info(
            "Outputs: /drone/odom_sensor and /drone/imu_sensor"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("profile", "nominal")
        self.declare_parameter("seed", 2026)
        self.declare_parameter("dropout_probability", 0.0)

        self.declare_parameter("position_noise_std_m", 0.0)
        self.declare_parameter("velocity_noise_std_mps", 0.0)
        self.declare_parameter("orientation_noise_std_deg", 0.0)
        self.declare_parameter("angular_velocity_noise_std_rps", 0.0)

        self.declare_parameter("position_bias_initial_m", [0.0, 0.0, 0.0])
        self.declare_parameter("velocity_bias_initial_mps", [0.0, 0.0, 0.0])
        self.declare_parameter("orientation_bias_initial_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("angular_velocity_bias_initial_rps", [0.0, 0.0, 0.0])

        self.declare_parameter("position_bias_rw_std_m_sqrt_s", 0.0)
        self.declare_parameter("velocity_bias_rw_std_mps_sqrt_s", 0.0)
        self.declare_parameter("orientation_bias_rw_std_deg_sqrt_s", 0.0)
        self.declare_parameter("angular_velocity_bias_rw_std_rps_sqrt_s", 0.0)

        self.declare_parameter("accelerometer_noise_std_mps2", 0.0)
        self.declare_parameter("gyroscope_noise_std_rps", 0.0)
        self.declare_parameter("imu_orientation_noise_std_deg", 0.0)
        self.declare_parameter("accelerometer_bias_initial_mps2", [0.0, 0.0, 0.0])
        self.declare_parameter("gyroscope_bias_initial_rps", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_orientation_bias_initial_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("accelerometer_bias_rw_std_mps2_sqrt_s", 0.0)
        self.declare_parameter("gyroscope_bias_rw_std_rps_sqrt_s", 0.0)
        self.declare_parameter("accelerometer_scale_error", [0.0, 0.0, 0.0])
        self.declare_parameter("gyroscope_scale_error", [0.0, 0.0, 0.0])
        self.declare_parameter("accelerometer_limit_mps2", 80.0)
        self.declare_parameter("gyroscope_limit_rps", 20.0)

    @staticmethod
    def _vector_parameter(node: Node, name: str) -> Tuple[float, float, float]:
        values = list(node.get_parameter(name).value)
        if len(values) != 3:
            raise RuntimeError(f"Parameter {name} must contain exactly 3 values")
        return float(values[0]), float(values[1]), float(values[2])

    def _load_parameters(self) -> None:
        self.profile = str(self.get_parameter("profile").value)
        self.seed = int(self.get_parameter("seed").value)
        self.dropout_probability = float(
            self.get_parameter("dropout_probability").value
        )

        self.position_noise_std = float(
            self.get_parameter("position_noise_std_m").value
        )
        self.velocity_noise_std = float(
            self.get_parameter("velocity_noise_std_mps").value
        )
        self.orientation_noise_std_rad = math.radians(
            float(self.get_parameter("orientation_noise_std_deg").value)
        )
        self.angular_velocity_noise_std = float(
            self.get_parameter("angular_velocity_noise_std_rps").value
        )

        self.position_bias_initial = self._vector_parameter(
            self, "position_bias_initial_m"
        )
        self.velocity_bias_initial = self._vector_parameter(
            self, "velocity_bias_initial_mps"
        )
        self.orientation_bias_initial = tuple(
            math.radians(value)
            for value in self._vector_parameter(
                self, "orientation_bias_initial_deg"
            )
        )
        self.angular_velocity_bias_initial = self._vector_parameter(
            self, "angular_velocity_bias_initial_rps"
        )

        self.position_bias_rw_std = float(
            self.get_parameter("position_bias_rw_std_m_sqrt_s").value
        )
        self.velocity_bias_rw_std = float(
            self.get_parameter("velocity_bias_rw_std_mps_sqrt_s").value
        )
        self.orientation_bias_rw_std_rad = math.radians(
            float(
                self.get_parameter(
                    "orientation_bias_rw_std_deg_sqrt_s"
                ).value
            )
        )
        self.angular_velocity_bias_rw_std = float(
            self.get_parameter(
                "angular_velocity_bias_rw_std_rps_sqrt_s"
            ).value
        )

        self.accelerometer_noise_std = float(
            self.get_parameter("accelerometer_noise_std_mps2").value
        )
        self.gyroscope_noise_std = float(
            self.get_parameter("gyroscope_noise_std_rps").value
        )
        self.imu_orientation_noise_std_rad = math.radians(
            float(
                self.get_parameter("imu_orientation_noise_std_deg").value
            )
        )
        self.accelerometer_bias_initial = self._vector_parameter(
            self, "accelerometer_bias_initial_mps2"
        )
        self.gyroscope_bias_initial = self._vector_parameter(
            self, "gyroscope_bias_initial_rps"
        )
        self.imu_orientation_bias_initial = tuple(
            math.radians(value)
            for value in self._vector_parameter(
                self, "imu_orientation_bias_initial_deg"
            )
        )
        self.accelerometer_bias_rw_std = float(
            self.get_parameter(
                "accelerometer_bias_rw_std_mps2_sqrt_s"
            ).value
        )
        self.gyroscope_bias_rw_std = float(
            self.get_parameter("gyroscope_bias_rw_std_rps_sqrt_s").value
        )
        self.accelerometer_scale_error = self._vector_parameter(
            self, "accelerometer_scale_error"
        )
        self.gyroscope_scale_error = self._vector_parameter(
            self, "gyroscope_scale_error"
        )
        self.accelerometer_limit = float(
            self.get_parameter("accelerometer_limit_mps2").value
        )
        self.gyroscope_limit = float(
            self.get_parameter("gyroscope_limit_rps").value
        )

        standard_deviations = [
            self.position_noise_std,
            self.velocity_noise_std,
            self.orientation_noise_std_rad,
            self.angular_velocity_noise_std,
            self.position_bias_rw_std,
            self.velocity_bias_rw_std,
            self.orientation_bias_rw_std_rad,
            self.angular_velocity_bias_rw_std,
            self.accelerometer_noise_std,
            self.gyroscope_noise_std,
            self.imu_orientation_noise_std_rad,
            self.accelerometer_bias_rw_std,
            self.gyroscope_bias_rw_std,
        ]
        if any(value < 0.0 for value in standard_deviations):
            raise RuntimeError("Noise and random-walk values must be non-negative")
        if not 0.0 <= self.dropout_probability < 1.0:
            raise RuntimeError("dropout_probability must be in [0, 1)")
        if self.accelerometer_limit <= 0.0 or self.gyroscope_limit <= 0.0:
            raise RuntimeError("Sensor limits must be positive")

    def _gaussian(self, standard_deviation: float) -> float:
        if standard_deviation <= 0.0:
            return 0.0
        return self.random.gauss(0.0, standard_deviation)

    @staticmethod
    def _stamp_seconds(message) -> float:
        return (
            float(message.header.stamp.sec)
            + 1.0e-9 * float(message.header.stamp.nanosec)
        )

    def _compute_dt(self, stamp: float, last_stamp: float | None) -> float:
        if last_stamp is None:
            return 0.005
        return clamp(stamp - last_stamp, 0.0005, 0.10)

    def _random_walk(self, values: list[float], std: float, dt: float) -> None:
        if std <= 0.0:
            return
        increment_std = std * math.sqrt(dt)
        for index in range(3):
            values[index] += self._gaussian(increment_std)

    def _drop_message(self) -> bool:
        return self.random.random() < self.dropout_probability

    def _noisy_quaternion(
        self,
        quaternion,
        bias_rpy: Iterable[float],
        white_noise_std_rad: float,
    ) -> Tuple[float, float, float, float]:
        q_true = normalize_quaternion(
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z,
        )
        bias = list(bias_rpy)
        q_error = quaternion_from_rpy(
            bias[0] + self._gaussian(white_noise_std_rad),
            bias[1] + self._gaussian(white_noise_std_rad),
            bias[2] + self._gaussian(white_noise_std_rad),
        )
        return normalize_quaternion(*quaternion_multiply(q_true, q_error))

    def _odom_callback(self, message: Odometry) -> None:
        self.odom_received += 1
        if self._drop_message():
            return

        stamp = self._stamp_seconds(message)
        dt = self._compute_dt(stamp, self.last_odom_stamp)
        self.last_odom_stamp = stamp

        self._random_walk(
            self.position_bias, self.position_bias_rw_std, dt
        )
        self._random_walk(
            self.velocity_bias, self.velocity_bias_rw_std, dt
        )
        self._random_walk(
            self.orientation_bias, self.orientation_bias_rw_std_rad, dt
        )
        self._random_walk(
            self.angular_velocity_bias,
            self.angular_velocity_bias_rw_std,
            dt,
        )

        noisy = copy.deepcopy(message)

        positions = [
            noisy.pose.pose.position.x,
            noisy.pose.pose.position.y,
            noisy.pose.pose.position.z,
        ]
        velocities = [
            noisy.twist.twist.linear.x,
            noisy.twist.twist.linear.y,
            noisy.twist.twist.linear.z,
        ]
        angular_velocities = [
            noisy.twist.twist.angular.x,
            noisy.twist.twist.angular.y,
            noisy.twist.twist.angular.z,
        ]

        for index in range(3):
            positions[index] += (
                self.position_bias[index]
                + self._gaussian(self.position_noise_std)
            )
            velocities[index] += (
                self.velocity_bias[index]
                + self._gaussian(self.velocity_noise_std)
            )
            angular_velocities[index] += (
                self.angular_velocity_bias[index]
                + self._gaussian(self.angular_velocity_noise_std)
            )

        noisy.pose.pose.position.x = positions[0]
        noisy.pose.pose.position.y = positions[1]
        noisy.pose.pose.position.z = positions[2]
        noisy.twist.twist.linear.x = velocities[0]
        noisy.twist.twist.linear.y = velocities[1]
        noisy.twist.twist.linear.z = velocities[2]
        noisy.twist.twist.angular.x = angular_velocities[0]
        noisy.twist.twist.angular.y = angular_velocities[1]
        noisy.twist.twist.angular.z = angular_velocities[2]

        qw, qx, qy, qz = self._noisy_quaternion(
            noisy.pose.pose.orientation,
            self.orientation_bias,
            self.orientation_noise_std_rad,
        )
        noisy.pose.pose.orientation.w = qw
        noisy.pose.pose.orientation.x = qx
        noisy.pose.pose.orientation.y = qy
        noisy.pose.pose.orientation.z = qz

        position_variance = self.position_noise_std ** 2
        velocity_variance = self.velocity_noise_std ** 2
        orientation_variance = self.orientation_noise_std_rad ** 2
        angular_velocity_variance = self.angular_velocity_noise_std ** 2
        noisy.pose.covariance = diagonal_covariance(
            [position_variance] * 3,
            [orientation_variance] * 3,
        )
        noisy.twist.covariance = diagonal_covariance(
            [velocity_variance] * 3,
            [angular_velocity_variance] * 3,
        )

        self.odom_publisher.publish(noisy)
        self.odom_published += 1

    def _imu_callback(self, message: Imu) -> None:
        self.imu_received += 1
        if self._drop_message():
            return

        stamp = self._stamp_seconds(message)
        dt = self._compute_dt(stamp, self.last_imu_stamp)
        self.last_imu_stamp = stamp

        self._random_walk(
            self.accelerometer_bias,
            self.accelerometer_bias_rw_std,
            dt,
        )
        self._random_walk(
            self.gyroscope_bias,
            self.gyroscope_bias_rw_std,
            dt,
        )

        noisy = copy.deepcopy(message)
        acceleration = [
            noisy.linear_acceleration.x,
            noisy.linear_acceleration.y,
            noisy.linear_acceleration.z,
        ]
        angular_velocity = [
            noisy.angular_velocity.x,
            noisy.angular_velocity.y,
            noisy.angular_velocity.z,
        ]

        for index in range(3):
            acceleration[index] = clamp(
                acceleration[index]
                * (1.0 + self.accelerometer_scale_error[index])
                + self.accelerometer_bias[index]
                + self._gaussian(self.accelerometer_noise_std),
                -self.accelerometer_limit,
                self.accelerometer_limit,
            )
            angular_velocity[index] = clamp(
                angular_velocity[index]
                * (1.0 + self.gyroscope_scale_error[index])
                + self.gyroscope_bias[index]
                + self._gaussian(self.gyroscope_noise_std),
                -self.gyroscope_limit,
                self.gyroscope_limit,
            )

        noisy.linear_acceleration.x = acceleration[0]
        noisy.linear_acceleration.y = acceleration[1]
        noisy.linear_acceleration.z = acceleration[2]
        noisy.angular_velocity.x = angular_velocity[0]
        noisy.angular_velocity.y = angular_velocity[1]
        noisy.angular_velocity.z = angular_velocity[2]

        imu_bias = self.imu_orientation_bias_initial
        qw, qx, qy, qz = self._noisy_quaternion(
            noisy.orientation,
            imu_bias,
            self.imu_orientation_noise_std_rad,
        )
        noisy.orientation.w = qw
        noisy.orientation.x = qx
        noisy.orientation.y = qy
        noisy.orientation.z = qz

        orientation_variance = self.imu_orientation_noise_std_rad ** 2
        gyro_variance = self.gyroscope_noise_std ** 2
        accel_variance = self.accelerometer_noise_std ** 2
        noisy.orientation_covariance = [
            orientation_variance, 0.0, 0.0,
            0.0, orientation_variance, 0.0,
            0.0, 0.0, orientation_variance,
        ]
        noisy.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance,
        ]
        noisy.linear_acceleration_covariance = [
            accel_variance, 0.0, 0.0,
            0.0, accel_variance, 0.0,
            0.0, 0.0, accel_variance,
        ]

        self.imu_publisher.publish(noisy)
        self.imu_published += 1

    def _log_statistics(self) -> None:
        odom_ratio = (
            self.odom_published / self.odom_received
            if self.odom_received else 0.0
        )
        imu_ratio = (
            self.imu_published / self.imu_received
            if self.imu_received else 0.0
        )
        self.get_logger().info(
            f"Sensor delivery ratios: odom={odom_ratio:.4f} "
            f"imu={imu_ratio:.4f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorModelNode()
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
