#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
from collections import OrderedDict
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def quaternion_tilt_deg(quaternion) -> float:
    w = quaternion.w
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-12:
        return 180.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_argument = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_argument)
    return math.degrees(math.sqrt(roll * roll + pitch * pitch))


def quaternion_angle_error_deg(q1, q2) -> float:
    a = (q1.w, q1.x, q1.y, q1.z)
    b = (q2.w, q2.x, q2.y, q2.z)
    na = math.sqrt(sum(value * value for value in a))
    nb = math.sqrt(sum(value * value for value in b))
    if na < 1.0e-12 or nb < 1.0e-12:
        return 180.0
    dot = abs(sum(a[i] * b[i] for i in range(4)) / (na * nb))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


class SensorExperiment(Node):
    def __init__(self) -> None:
        super().__init__("sensor_experiment")
        self._declare_parameters()
        self._load_parameters()

        self.start_time = self.get_clock().now()
        self.truth_odom_cache: OrderedDict[int, Odometry] = OrderedDict()
        self.truth_imu_cache: OrderedDict[int, Imu] = OrderedDict()
        self.latest_truth_odom: Odometry | None = None
        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]

        self.flight_rows = []
        self.odom_error_rows = []
        self.imu_error_rows = []

        self.create_subscription(Odometry, "/drone/odom", self._truth_odom, 50)
        self.create_subscription(
            Odometry, "/drone/odom_sensor", self._sensor_odom, 50
        )
        self.create_subscription(Imu, "/drone/imu", self._truth_imu, 50)
        self.create_subscription(
            Imu, "/drone/imu_sensor", self._sensor_imu, 50
        )
        self.create_subscription(
            Float32MultiArray, "/drone/motor_rpm", self._rpm_callback, 50
        )
        self.create_subscription(
            PoseStamped, "/drone/goal", self._goal_callback, 10
        )

        self.finish_timer = self.create_timer(self.duration_s, self._finish)
        self.finished = False

        self.get_logger().info(
            f"Sensor experiment started: profile={self.profile} "
            f"duration={self.duration_s:.1f} s "
            f"goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}, "
            f"{self.goal[2]:.2f})"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("profile", "nominal")
        self.declare_parameter("duration_s", 18.0)
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("results_root", "")
        self.declare_parameter("final_error_threshold_m", 0.30)
        self.declare_parameter("maximum_tilt_threshold_deg", 35.0)
        self.declare_parameter("maximum_rpm", 10000.0)
        self.declare_parameter("rpm_saturation_ratio", 0.995)
        self.declare_parameter("maximum_rpm_saturation_fraction", 0.02)
        self.declare_parameter("minimum_odom_pairs", 500)
        self.declare_parameter("minimum_imu_pairs", 500)
        self.declare_parameter("position_rmse_threshold_m", 0.20)
        self.declare_parameter("velocity_rmse_threshold_mps", 0.20)
        self.declare_parameter("orientation_rmse_threshold_deg", 3.0)
        self.declare_parameter("gyro_rmse_threshold_rps", 0.05)
        self.declare_parameter("accel_rmse_threshold_mps2", 0.50)

    def _load_parameters(self) -> None:
        self.profile = str(self.get_parameter("profile").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        configured_root = str(self.get_parameter("results_root").value)
        self.results_root = (
            Path(configured_root).expanduser()
            if configured_root
            else Path.home()
            / "drone_task"
            / "your_drone_sim_ws"
            / "results"
            / "sensor_noise"
        )
        self.final_error_threshold = float(
            self.get_parameter("final_error_threshold_m").value
        )
        self.maximum_tilt_threshold = float(
            self.get_parameter("maximum_tilt_threshold_deg").value
        )
        self.maximum_rpm = float(self.get_parameter("maximum_rpm").value)
        self.rpm_saturation_ratio = float(
            self.get_parameter("rpm_saturation_ratio").value
        )
        self.maximum_rpm_saturation_fraction = float(
            self.get_parameter("maximum_rpm_saturation_fraction").value
        )
        self.minimum_odom_pairs = int(
            self.get_parameter("minimum_odom_pairs").value
        )
        self.minimum_imu_pairs = int(
            self.get_parameter("minimum_imu_pairs").value
        )
        self.position_rmse_threshold = float(
            self.get_parameter("position_rmse_threshold_m").value
        )
        self.velocity_rmse_threshold = float(
            self.get_parameter("velocity_rmse_threshold_mps").value
        )
        self.orientation_rmse_threshold = float(
            self.get_parameter("orientation_rmse_threshold_deg").value
        )
        self.gyro_rmse_threshold = float(
            self.get_parameter("gyro_rmse_threshold_rps").value
        )
        self.accel_rmse_threshold = float(
            self.get_parameter("accel_rmse_threshold_mps2").value
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1.0e-9

    @staticmethod
    def _trim_cache(cache: OrderedDict, maximum_size: int = 3000) -> None:
        while len(cache) > maximum_size:
            cache.popitem(last=False)

    def _truth_odom(self, message: Odometry) -> None:
        key = stamp_ns(message)
        self.truth_odom_cache[key] = message
        self._trim_cache(self.truth_odom_cache)
        self.latest_truth_odom = message

        px = message.pose.pose.position.x
        py = message.pose.pose.position.y
        pz = message.pose.pose.position.z
        error = math.sqrt(
            (px - self.goal[0]) ** 2
            + (py - self.goal[1]) ** 2
            + (pz - self.goal[2]) ** 2
        )
        tilt = quaternion_tilt_deg(message.pose.pose.orientation)
        saturated = any(
            rpm >= self.maximum_rpm * self.rpm_saturation_ratio
            for rpm in self.latest_rpm
        )
        self.flight_rows.append(
            [self._elapsed(), px, py, pz, error, tilt, *self.latest_rpm, int(saturated)]
        )

    def _sensor_odom(self, message: Odometry) -> None:
        key = stamp_ns(message)
        truth = self.truth_odom_cache.get(key)
        if truth is None:
            return
        position_error = [
            message.pose.pose.position.x - truth.pose.pose.position.x,
            message.pose.pose.position.y - truth.pose.pose.position.y,
            message.pose.pose.position.z - truth.pose.pose.position.z,
        ]
        velocity_error = [
            message.twist.twist.linear.x - truth.twist.twist.linear.x,
            message.twist.twist.linear.y - truth.twist.twist.linear.y,
            message.twist.twist.linear.z - truth.twist.twist.linear.z,
        ]
        angular_velocity_error = [
            message.twist.twist.angular.x - truth.twist.twist.angular.x,
            message.twist.twist.angular.y - truth.twist.twist.angular.y,
            message.twist.twist.angular.z - truth.twist.twist.angular.z,
        ]
        orientation_error = quaternion_angle_error_deg(
            message.pose.pose.orientation, truth.pose.pose.orientation
        )
        self.odom_error_rows.append(
            [
                self._elapsed(),
                *position_error,
                *velocity_error,
                orientation_error,
                *angular_velocity_error,
            ]
        )

    def _truth_imu(self, message: Imu) -> None:
        key = stamp_ns(message)
        self.truth_imu_cache[key] = message
        self._trim_cache(self.truth_imu_cache)

    def _sensor_imu(self, message: Imu) -> None:
        key = stamp_ns(message)
        truth = self.truth_imu_cache.get(key)
        if truth is None:
            return
        accel_error = [
            message.linear_acceleration.x - truth.linear_acceleration.x,
            message.linear_acceleration.y - truth.linear_acceleration.y,
            message.linear_acceleration.z - truth.linear_acceleration.z,
        ]
        gyro_error = [
            message.angular_velocity.x - truth.angular_velocity.x,
            message.angular_velocity.y - truth.angular_velocity.y,
            message.angular_velocity.z - truth.angular_velocity.z,
        ]
        orientation_error = quaternion_angle_error_deg(
            message.orientation, truth.orientation
        )
        self.imu_error_rows.append(
            [self._elapsed(), *accel_error, *gyro_error, orientation_error]
        )

    def _rpm_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) == 4:
            self.latest_rpm = [float(value) for value in message.data]

    def _goal_callback(self, message: PoseStamped) -> None:
        self.goal = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )

    @staticmethod
    def _rmse(rows, indices) -> float:
        if not rows:
            return float("inf")
        squared_sum = 0.0
        count = 0
        for row in rows:
            for index in indices:
                value = float(row[index])
                squared_sum += value * value
                count += 1
        return math.sqrt(squared_sum / max(count, 1))

    def _finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.finish_timer.cancel()

        if not self.flight_rows:
            self.get_logger().error("No truth odometry samples were collected")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_directory = self.results_root / f"sensor_{self.profile}_{timestamp}"
        output_directory.mkdir(parents=True, exist_ok=True)

        self._write_csv(
            output_directory / "flight.csv",
            [
                "time_s", "x_m", "y_m", "z_m", "goal_error_m", "tilt_deg",
                "rpm_1", "rpm_2", "rpm_3", "rpm_4", "rpm_saturated",
            ],
            self.flight_rows,
        )
        self._write_csv(
            output_directory / "odom_sensor_error.csv",
            [
                "time_s", "px_error_m", "py_error_m", "pz_error_m",
                "vx_error_mps", "vy_error_mps", "vz_error_mps",
                "orientation_error_deg", "wx_error_rps", "wy_error_rps",
                "wz_error_rps",
            ],
            self.odom_error_rows,
        )
        self._write_csv(
            output_directory / "imu_sensor_error.csv",
            [
                "time_s", "ax_error_mps2", "ay_error_mps2", "az_error_mps2",
                "gx_error_rps", "gy_error_rps", "gz_error_rps",
                "orientation_error_deg",
            ],
            self.imu_error_rows,
        )

        final_window = self.flight_rows[-min(len(self.flight_rows), 200):]
        final_error = sum(row[4] for row in final_window) / len(final_window)
        maximum_tilt = max(row[5] for row in self.flight_rows)
        saturation_fraction = sum(row[10] for row in self.flight_rows) / len(
            self.flight_rows
        )

        position_rmse = self._rmse(self.odom_error_rows, [1, 2, 3])
        velocity_rmse = self._rmse(self.odom_error_rows, [4, 5, 6])
        orientation_rmse = self._rmse(self.odom_error_rows, [7])
        angular_velocity_rmse = self._rmse(self.odom_error_rows, [8, 9, 10])
        accel_rmse = self._rmse(self.imu_error_rows, [1, 2, 3])
        gyro_rmse = self._rmse(self.imu_error_rows, [4, 5, 6])
        imu_orientation_rmse = self._rmse(self.imu_error_rows, [7])

        criteria: Dict[str, bool] = {
            "enough_odom_pairs": len(self.odom_error_rows) >= self.minimum_odom_pairs,
            "enough_imu_pairs": len(self.imu_error_rows) >= self.minimum_imu_pairs,
            "final_error_pass": final_error <= self.final_error_threshold,
            "tilt_pass": maximum_tilt <= self.maximum_tilt_threshold,
            "rpm_saturation_pass": saturation_fraction
            <= self.maximum_rpm_saturation_fraction,
            "position_rmse_pass": position_rmse <= self.position_rmse_threshold,
            "velocity_rmse_pass": velocity_rmse <= self.velocity_rmse_threshold,
            "orientation_rmse_pass": orientation_rmse
            <= self.orientation_rmse_threshold,
            "gyro_rmse_pass": gyro_rmse <= self.gyro_rmse_threshold,
            "accel_rmse_pass": accel_rmse <= self.accel_rmse_threshold,
        }
        summary = {
            "experiment": "sensor_noise_and_simplified_imu",
            "profile": self.profile,
            "goal_m": list(self.goal),
            "duration_s": self.duration_s,
            "truth_sample_count": len(self.flight_rows),
            "odom_pair_count": len(self.odom_error_rows),
            "imu_pair_count": len(self.imu_error_rows),
            "final_truth_position_error_m": final_error,
            "maximum_truth_tilt_deg": maximum_tilt,
            "rpm_saturation_fraction": saturation_fraction,
            "odom_position_rmse_m": position_rmse,
            "odom_velocity_rmse_mps": velocity_rmse,
            "odom_orientation_rmse_deg": orientation_rmse,
            "odom_angular_velocity_rmse_rps": angular_velocity_rmse,
            "imu_acceleration_rmse_mps2": accel_rmse,
            "imu_gyroscope_rmse_rps": gyro_rmse,
            "imu_orientation_rmse_deg": imu_orientation_rmse,
            "criteria": criteria,
            "pass": all(criteria.values()),
        }
        (output_directory / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        self._make_plots(output_directory)

        self.get_logger().info(
            f"Sensor experiment result: {output_directory}"
        )
        self.get_logger().info(
            f"profile={self.profile} pass={summary['pass']} "
            f"final_error={final_error:.4f} m "
            f"pos_rmse={position_rmse:.4f} m "
            f"imu_accel_rmse={accel_rmse:.4f} m/s^2"
        )

    @staticmethod
    def _write_csv(path: Path, header, rows) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(rows)

    def _make_plots(self, output_directory: Path) -> None:
        time = [row[0] for row in self.flight_rows]
        plt.figure(figsize=(10, 5))
        plt.plot(time, [row[4] for row in self.flight_rows])
        plt.axhline(self.final_error_threshold, linestyle="--")
        plt.xlabel("Time [s]")
        plt.ylabel("Truth position error [m]")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_directory / "flight_position_error.png", dpi=160)
        plt.close()

        if self.odom_error_rows:
            t = [row[0] for row in self.odom_error_rows]
            plt.figure(figsize=(10, 5))
            for index, label in zip([1, 2, 3], ["x", "y", "z"]):
                plt.plot(t, [row[index] for row in self.odom_error_rows], label=label)
            plt.xlabel("Time [s]")
            plt.ylabel("Odometry position measurement error [m]")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(output_directory / "odom_position_error.png", dpi=160)
            plt.close()

        if self.imu_error_rows:
            t = [row[0] for row in self.imu_error_rows]
            plt.figure(figsize=(10, 5))
            for index, label in zip([1, 2, 3], ["ax", "ay", "az"]):
                plt.plot(t, [row[index] for row in self.imu_error_rows], label=label)
            plt.xlabel("Time [s]")
            plt.ylabel("IMU acceleration measurement error [m/s²]")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(output_directory / "imu_acceleration_error.png", dpi=160)
            plt.close()

        plt.figure(figsize=(10, 5))
        for index in range(6, 10):
            plt.plot(time, [row[index] for row in self.flight_rows])
        plt.axhline(self.maximum_rpm * self.rpm_saturation_ratio, linestyle="--")
        plt.xlabel("Time [s]")
        plt.ylabel("Motor RPM")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_directory / "motor_rpm.png", dpi=160)
        plt.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorExperiment()
    try:
        # Do not call rclpy.shutdown() from inside a timer callback.
        # On ROS 2 Humble this can leave rclpy.spin() blocked and prevent
        # the launch OnProcessExit handler from firing.  Instead, keep the
        # executor responsive and exit cleanly once _finish() sets finished.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
