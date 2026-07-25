#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import random
from typing import Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def quaternion_to_euler(
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


def euler_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


class SensorNoiseBridge(Node):
    def __init__(self) -> None:
        super().__init__("sensor_noise_bridge")

        defaults = {
            "enabled": True,
            "random_seed": 20260723,
            "odom_input_topic": "/drone/odom",
            "odom_output_topic": "/drone/odom_noisy",
            "imu_input_topic": "/drone/imu",
            "imu_output_topic": "/drone/imu_noisy",
            "odom_position_sigma_m": 0.020,
            "odom_velocity_sigma_mps": 0.030,
            "odom_attitude_sigma_deg": 0.30,
            "odom_angular_rate_sigma_rps": 0.010,
            "imu_orientation_sigma_deg": 0.30,
            "imu_angular_velocity_sigma_rps": 0.010,
            "imu_linear_acceleration_sigma_mps2": 0.080,
            "status_publish_period_s": 1.0,
        }

        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.enabled = bool(self.get_parameter("enabled").value)
        seed = int(self.get_parameter("random_seed").value)
        self.rng = random.Random(seed)

        self.odom_position_sigma = float(
            self.get_parameter("odom_position_sigma_m").value
        )
        self.odom_velocity_sigma = float(
            self.get_parameter("odom_velocity_sigma_mps").value
        )
        self.odom_attitude_sigma = math.radians(
            float(self.get_parameter("odom_attitude_sigma_deg").value)
        )
        self.odom_angular_rate_sigma = float(
            self.get_parameter("odom_angular_rate_sigma_rps").value
        )

        self.imu_orientation_sigma = math.radians(
            float(self.get_parameter("imu_orientation_sigma_deg").value)
        )
        self.imu_angular_velocity_sigma = float(
            self.get_parameter("imu_angular_velocity_sigma_rps").value
        )
        self.imu_linear_acceleration_sigma = float(
            self.get_parameter("imu_linear_acceleration_sigma_mps2").value
        )

        odom_in = str(self.get_parameter("odom_input_topic").value)
        odom_out = str(self.get_parameter("odom_output_topic").value)
        imu_in = str(self.get_parameter("imu_input_topic").value)
        imu_out = str(self.get_parameter("imu_output_topic").value)

        self.odom_pub = self.create_publisher(Odometry, odom_out, 20)
        self.imu_pub = self.create_publisher(Imu, imu_out, 20)
        self.status_pub = self.create_publisher(
            String,
            "/cq01_bonus/sensor_noise/status",
            10,
        )

        self.create_subscription(
            Odometry,
            odom_in,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_in,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.odom_count = 0
        self.imu_count = 0
        self.position_sq_sum = 0.0
        self.velocity_sq_sum = 0.0
        self.angular_rate_sq_sum = 0.0
        self.linear_accel_sq_sum = 0.0

        period = max(
            0.2,
            float(
                self.get_parameter("status_publish_period_s").value
            ),
        )
        self.create_timer(period, self.publish_status)

        self.get_logger().warning(
            "Sensor-noise simulation active: "
            f"enabled={self.enabled}, seed={seed}, "
            f"odom_out={odom_out}, imu_out={imu_out}"
        )

    def noise(self, sigma: float) -> float:
        if not self.enabled or sigma <= 0.0:
            return 0.0
        return self.rng.gauss(0.0, sigma)

    def odom_callback(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        nx = self.noise(self.odom_position_sigma)
        ny = self.noise(self.odom_position_sigma)
        nz = self.noise(self.odom_position_sigma)

        out.pose.pose.position.x += nx
        out.pose.pose.position.y += ny
        out.pose.pose.position.z += nz

        q = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(
            q.x,
            q.y,
            q.z,
            q.w,
        )
        roll += self.noise(self.odom_attitude_sigma)
        pitch += self.noise(self.odom_attitude_sigma)
        yaw += self.noise(self.odom_attitude_sigma)

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            yaw,
        )
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        nvx = self.noise(self.odom_velocity_sigma)
        nvy = self.noise(self.odom_velocity_sigma)
        nvz = self.noise(self.odom_velocity_sigma)

        out.twist.twist.linear.x += nvx
        out.twist.twist.linear.y += nvy
        out.twist.twist.linear.z += nvz

        nwx = self.noise(self.odom_angular_rate_sigma)
        nwy = self.noise(self.odom_angular_rate_sigma)
        nwz = self.noise(self.odom_angular_rate_sigma)

        out.twist.twist.angular.x += nwx
        out.twist.twist.angular.y += nwy
        out.twist.twist.angular.z += nwz

        self.odom_pub.publish(out)

        self.odom_count += 1
        self.position_sq_sum += nx * nx + ny * ny + nz * nz
        self.velocity_sq_sum += nvx * nvx + nvy * nvy + nvz * nvz
        self.angular_rate_sq_sum += nwx * nwx + nwy * nwy + nwz * nwz

    def imu_callback(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(
            q.x,
            q.y,
            q.z,
            q.w,
        )
        roll += self.noise(self.imu_orientation_sigma)
        pitch += self.noise(self.imu_orientation_sigma)
        yaw += self.noise(self.imu_orientation_sigma)

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            yaw,
        )
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw

        out.angular_velocity.x += self.noise(
            self.imu_angular_velocity_sigma
        )
        out.angular_velocity.y += self.noise(
            self.imu_angular_velocity_sigma
        )
        out.angular_velocity.z += self.noise(
            self.imu_angular_velocity_sigma
        )

        nax = self.noise(self.imu_linear_acceleration_sigma)
        nay = self.noise(self.imu_linear_acceleration_sigma)
        naz = self.noise(self.imu_linear_acceleration_sigma)

        out.linear_acceleration.x += nax
        out.linear_acceleration.y += nay
        out.linear_acceleration.z += naz

        if self.enabled:
            orientation_var = self.imu_orientation_sigma ** 2
            angular_var = self.imu_angular_velocity_sigma ** 2
            accel_var = self.imu_linear_acceleration_sigma ** 2

            out.orientation_covariance = [
                orientation_var, 0.0, 0.0,
                0.0, orientation_var, 0.0,
                0.0, 0.0, orientation_var,
            ]
            out.angular_velocity_covariance = [
                angular_var, 0.0, 0.0,
                0.0, angular_var, 0.0,
                0.0, 0.0, angular_var,
            ]
            out.linear_acceleration_covariance = [
                accel_var, 0.0, 0.0,
                0.0, accel_var, 0.0,
                0.0, 0.0, accel_var,
            ]

        self.imu_pub.publish(out)
        self.imu_count += 1
        self.linear_accel_sq_sum += nax * nax + nay * nay + naz * naz

    @staticmethod
    def vector_rms(square_sum: float, count: int) -> float:
        if count <= 0:
            return 0.0
        return math.sqrt(square_sum / (3.0 * count))

    def publish_status(self) -> None:
        payload = {
            "enabled": self.enabled,
            "odom_samples": self.odom_count,
            "imu_samples": self.imu_count,
            "configured": {
                "odom_position_sigma_m": self.odom_position_sigma,
                "odom_velocity_sigma_mps": self.odom_velocity_sigma,
                "odom_attitude_sigma_deg": math.degrees(
                    self.odom_attitude_sigma
                ),
                "odom_angular_rate_sigma_rps": self.odom_angular_rate_sigma,
                "imu_orientation_sigma_deg": math.degrees(
                    self.imu_orientation_sigma
                ),
                "imu_angular_velocity_sigma_rps":
                    self.imu_angular_velocity_sigma,
                "imu_linear_acceleration_sigma_mps2":
                    self.imu_linear_acceleration_sigma,
            },
            "observed_noise_rms": {
                "odom_position_m": self.vector_rms(
                    self.position_sq_sum,
                    self.odom_count,
                ),
                "odom_velocity_mps": self.vector_rms(
                    self.velocity_sq_sum,
                    self.odom_count,
                ),
                "odom_angular_rate_rps": self.vector_rms(
                    self.angular_rate_sq_sum,
                    self.odom_count,
                ),
                "imu_linear_acceleration_mps2": self.vector_rms(
                    self.linear_accel_sq_sum,
                    self.imu_count,
                ),
            },
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNoiseBridge()
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
