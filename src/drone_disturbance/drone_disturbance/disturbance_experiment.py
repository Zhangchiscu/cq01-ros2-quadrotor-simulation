#!/usr/bin/env python3

import csv
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class DisturbanceExperiment(Node):
    """Record and evaluate closed-loop recovery from an external disturbance."""

    def __init__(self) -> None:
        super().__init__("disturbance_experiment")

        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("startup_delay_s", 1.0)
        self.declare_parameter("duration_s", 20.0)
        self.declare_parameter("expected_disturbance_start_s", 8.0)
        self.declare_parameter("expected_disturbance_duration_s", 1.0)
        self.declare_parameter("recovery_tolerance_m", 0.20)
        self.declare_parameter("recovery_hold_s", 1.0)
        self.declare_parameter("final_tolerance_m", 0.30)
        self.declare_parameter("baseline_tolerance_m", 0.20)
        self.declare_parameter("maximum_tilt_deg", 45.0)
        self.declare_parameter("maximum_rpm", 10000.0)
        self.declare_parameter("saturation_ratio_threshold", 0.98)
        self.declare_parameter("maximum_saturation_fraction", 0.05)
        self.declare_parameter("final_window_s", 2.0)
        self.declare_parameter("results_root", "")

        self.goal = [
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        ]
        self.startup_delay_s = float(
            self.get_parameter("startup_delay_s").value
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.expected_start_s = float(
            self.get_parameter("expected_disturbance_start_s").value
        )
        self.expected_duration_s = float(
            self.get_parameter("expected_disturbance_duration_s").value
        )
        self.recovery_tolerance_m = float(
            self.get_parameter("recovery_tolerance_m").value
        )
        self.recovery_hold_s = float(
            self.get_parameter("recovery_hold_s").value
        )
        self.final_tolerance_m = float(
            self.get_parameter("final_tolerance_m").value
        )
        self.baseline_tolerance_m = float(
            self.get_parameter("baseline_tolerance_m").value
        )
        self.maximum_tilt_deg = float(
            self.get_parameter("maximum_tilt_deg").value
        )
        self.maximum_rpm = float(self.get_parameter("maximum_rpm").value)
        self.saturation_ratio_threshold = float(
            self.get_parameter("saturation_ratio_threshold").value
        )
        self.maximum_saturation_fraction = float(
            self.get_parameter("maximum_saturation_fraction").value
        )
        self.final_window_s = float(
            self.get_parameter("final_window_s").value
        )

        results_root = str(self.get_parameter("results_root").value)
        if not results_root:
            results_root = str(
                Path.home()
                / "drone_task"
                / "your_drone_sim_ws"
                / "results"
                / "disturbance_recovery"
            )
        self.results_root = Path(os.path.expanduser(results_root))

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "drone/goal",
            10,
        )
        self.create_subscription(
            Odometry,
            "drone/odom",
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            "drone/motor_rpm",
            self.rpm_callback,
            50,
        )
        self.create_subscription(
            WrenchStamped,
            "drone/external_wrench_applied",
            self.wrench_callback,
            50,
        )
        self.create_subscription(
            Bool,
            "drone/disturbance_active",
            self.active_callback,
            20,
        )

        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.latest_force = [0.0, 0.0, 0.0]
        self.latest_torque = [0.0, 0.0, 0.0]
        self.disturbance_active = False
        self.first_odom_received = False
        self.started = False
        self.finished = False
        self.passed = False
        self.start_ns: Optional[int] = None
        self.samples: List[Dict[str, float]] = []

        self.start_timer = self.create_timer(
            max(0.1, self.startup_delay_s),
            self.try_start,
        )
        self.goal_timer = None
        self.finish_timer = None

        self.get_logger().info(
            "Disturbance recovery experiment ready: "
            f"goal={self.goal}, duration={self.duration_s:.1f}s"
        )

    def try_start(self) -> None:
        if self.started:
            return
        if not self.first_odom_received:
            self.get_logger().warn("Waiting for the first drone/odom message")
            return

        self.started = True
        self.start_timer.cancel()
        self.start_ns = self.get_clock().now().nanoseconds
        self.publish_goal()
        self.goal_timer = self.create_timer(0.10, self.publish_goal)
        self.finish_timer = self.create_timer(
            self.duration_s,
            self.finish_experiment,
        )
        self.get_logger().info("Disturbance recovery experiment started")

    def publish_goal(self) -> None:
        if not self.started or self.finished:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.position.z = self.goal[2]
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def elapsed_s(self) -> float:
        if self.start_ns is None:
            return 0.0
        return (
            self.get_clock().now().nanoseconds - self.start_ns
        ) / 1.0e9

    def rpm_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 4:
            self.latest_rpm = [float(value) for value in message.data[:4]]

    def wrench_callback(self, message: WrenchStamped) -> None:
        self.latest_force = [
            float(message.wrench.force.x),
            float(message.wrench.force.y),
            float(message.wrench.force.z),
        ]
        self.latest_torque = [
            float(message.wrench.torque.x),
            float(message.wrench.torque.y),
            float(message.wrench.torque.z),
        ]

    def active_callback(self, message: Bool) -> None:
        self.disturbance_active = bool(message.data)

    @staticmethod
    def quaternion_to_euler(w: float, x: float, y: float, z: float) -> List[float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [roll, pitch, yaw]

    def odom_callback(self, message: Odometry) -> None:
        self.first_odom_received = True
        if not self.started or self.finished:
            return

        position = [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ]
        error = [self.goal[index] - position[index] for index in range(3)]
        error_norm = math.sqrt(sum(value * value for value in error))

        q = message.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.w, q.x, q.y, q.z)
        force_norm = math.sqrt(sum(value * value for value in self.latest_force))
        torque_norm = math.sqrt(sum(value * value for value in self.latest_torque))

        sample: Dict[str, float] = {
            "time_s": self.elapsed_s(),
            "position_x": position[0],
            "position_y": position[1],
            "position_z": position[2],
            "error_x": error[0],
            "error_y": error[1],
            "error_z": error[2],
            "error_norm": error_norm,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "force_x_n": self.latest_force[0],
            "force_y_n": self.latest_force[1],
            "force_z_n": self.latest_force[2],
            "force_norm_n": force_norm,
            "torque_x_nm": self.latest_torque[0],
            "torque_y_nm": self.latest_torque[1],
            "torque_z_nm": self.latest_torque[2],
            "torque_norm_nm": torque_norm,
            "disturbance_active": 1.0 if self.disturbance_active else 0.0,
            "rpm_front_left": self.latest_rpm[0],
            "rpm_front_right": self.latest_rpm[1],
            "rpm_rear_right": self.latest_rpm[2],
            "rpm_rear_left": self.latest_rpm[3],
        }
        self.samples.append(sample)

    @staticmethod
    def mean(values: List[float]) -> float:
        return sum(values) / len(values) if values else float("nan")

    def detect_disturbance_window(self) -> tuple[float, float]:
        active_times = [
            sample["time_s"]
            for sample in self.samples
            if sample["disturbance_active"] > 0.5
            or sample["force_norm_n"] > 1.0e-3
            or sample["torque_norm_nm"] > 1.0e-4
        ]
        if active_times:
            return min(active_times), max(active_times)
        return self.expected_start_s, self.expected_start_s + self.expected_duration_s

    def compute_recovery_time(self, disturbance_end_s: float) -> Optional[float]:
        after = [sample for sample in self.samples if sample["time_s"] >= disturbance_end_s]
        if not after:
            return None

        for index, sample in enumerate(after):
            if sample["error_norm"] > self.recovery_tolerance_m:
                continue
            window_end = sample["time_s"] + self.recovery_hold_s
            window = [
                candidate
                for candidate in after[index:]
                if candidate["time_s"] <= window_end
            ]
            if not window or window[-1]["time_s"] < window_end - 0.05:
                continue
            if all(candidate["error_norm"] <= self.recovery_tolerance_m for candidate in window):
                return sample["time_s"] - disturbance_end_s
        return None

    def compute_metrics(self) -> Dict[str, object]:
        if not self.samples:
            raise RuntimeError("No odometry samples were recorded")

        disturbance_start_s, disturbance_end_s = self.detect_disturbance_window()

        baseline_samples = [
            sample
            for sample in self.samples
            if max(0.0, disturbance_start_s - 2.0)
            <= sample["time_s"]
            < disturbance_start_s
        ]
        response_samples = [
            sample
            for sample in self.samples
            if sample["time_s"] >= disturbance_start_s
        ]
        final_start_s = max(0.0, self.samples[-1]["time_s"] - self.final_window_s)
        final_samples = [
            sample for sample in self.samples if sample["time_s"] >= final_start_s
        ]

        baseline_error = self.mean(
            [sample["error_norm"] for sample in baseline_samples]
        )
        peak_error = max(
            [sample["error_norm"] for sample in response_samples],
            default=float("nan"),
        )
        final_error = self.mean(
            [sample["error_norm"] for sample in final_samples]
        )
        recovery_time = self.compute_recovery_time(disturbance_end_s)

        max_roll = max(abs(sample["roll_deg"]) for sample in self.samples)
        max_pitch = max(abs(sample["pitch_deg"]) for sample in self.samples)
        max_tilt = max(max_roll, max_pitch)

        rpm_threshold = self.saturation_ratio_threshold * self.maximum_rpm
        rpm_values = [
            sample[key]
            for sample in self.samples
            for key in (
                "rpm_front_left",
                "rpm_front_right",
                "rpm_rear_right",
                "rpm_rear_left",
            )
        ]
        saturation_fraction = (
            sum(value >= rpm_threshold for value in rpm_values) / len(rpm_values)
            if rpm_values
            else 1.0
        )

        impulse_n_s = 0.0
        for previous, current in zip(self.samples[:-1], self.samples[1:]):
            dt = max(0.0, current["time_s"] - previous["time_s"])
            impulse_n_s += 0.5 * (
                previous["force_norm_n"] + current["force_norm_n"]
            ) * dt

        criteria = {
            "baseline_settled": baseline_error <= self.baseline_tolerance_m,
            "recovered": recovery_time is not None,
            "final_error_pass": final_error <= self.final_tolerance_m,
            "tilt_pass": max_tilt <= self.maximum_tilt_deg,
            "rpm_saturation_pass": (
                saturation_fraction <= self.maximum_saturation_fraction
            ),
        }
        passed = all(criteria.values())

        return {
            "experiment": "external_disturbance_recovery",
            "goal_m": self.goal,
            "sample_count": len(self.samples),
            "disturbance_start_s": disturbance_start_s,
            "disturbance_end_s": disturbance_end_s,
            "disturbance_impulse_n_s": impulse_n_s,
            "baseline_error_m": baseline_error,
            "peak_error_m": peak_error,
            "final_error_m": final_error,
            "recovery_time_s": recovery_time,
            "maximum_roll_deg": max_roll,
            "maximum_pitch_deg": max_pitch,
            "maximum_tilt_deg": max_tilt,
            "rpm_saturation_fraction": saturation_fraction,
            "criteria": criteria,
            "pass": passed,
        }

    def save_outputs(self, metrics: Dict[str, object]) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = self.results_root / f"disturbance_{timestamp}"
        output_dir.mkdir(parents=True, exist_ok=True)

        with (output_dir / "summary.json").open("w", encoding="utf-8") as file:
            json.dump(metrics, file, indent=2, ensure_ascii=False)

        fieldnames = list(self.samples[0].keys())
        with (output_dir / "samples.csv").open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)

        times = [sample["time_s"] for sample in self.samples]
        disturbance_start = float(metrics["disturbance_start_s"])
        disturbance_end = float(metrics["disturbance_end_s"])

        plt.figure(figsize=(9, 5))
        plt.plot(times, [sample["error_norm"] for sample in self.samples], label="Position error")
        plt.axvspan(disturbance_start, disturbance_end, alpha=0.2, label="Disturbance")
        plt.axhline(self.recovery_tolerance_m, linestyle="--", label="Recovery tolerance")
        plt.xlabel("Time (s)")
        plt.ylabel("Position error (m)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "position_error.png", dpi=180)
        plt.close()

        plt.figure(figsize=(9, 5))
        for key, label in (
            ("position_x", "x"),
            ("position_y", "y"),
            ("position_z", "z"),
        ):
            plt.plot(times, [sample[key] for sample in self.samples], label=label)
        plt.axvspan(disturbance_start, disturbance_end, alpha=0.2)
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "position_components.png", dpi=180)
        plt.close()

        plt.figure(figsize=(9, 5))
        plt.plot(times, [sample["force_norm_n"] for sample in self.samples], label="Force norm")
        plt.plot(times, [sample["torque_norm_nm"] for sample in self.samples], label="Torque norm")
        plt.xlabel("Time (s)")
        plt.ylabel("Applied disturbance")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "disturbance_profile.png", dpi=180)
        plt.close()

        plt.figure(figsize=(9, 5))
        plt.plot(times, [sample["roll_deg"] for sample in self.samples], label="roll")
        plt.plot(times, [sample["pitch_deg"] for sample in self.samples], label="pitch")
        plt.plot(times, [sample["yaw_deg"] for sample in self.samples], label="yaw")
        plt.axvspan(disturbance_start, disturbance_end, alpha=0.2)
        plt.xlabel("Time (s)")
        plt.ylabel("Attitude (deg)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output_dir / "attitude.png", dpi=180)
        plt.close()

        plt.figure(figsize=(9, 5))
        for key, label in (
            ("rpm_front_left", "front_left"),
            ("rpm_front_right", "front_right"),
            ("rpm_rear_right", "rear_right"),
            ("rpm_rear_left", "rear_left"),
        ):
            plt.plot(times, [sample[key] for sample in self.samples], label=label)
        plt.axvspan(disturbance_start, disturbance_end, alpha=0.2)
        plt.axhline(
            self.saturation_ratio_threshold * self.maximum_rpm,
            linestyle="--",
            label="Saturation threshold",
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Motor RPM")
        plt.grid(True)
        plt.legend(ncol=2)
        plt.tight_layout()
        plt.savefig(output_dir / "motor_rpm.png", dpi=180)
        plt.close()

        return output_dir

    def finish_experiment(self) -> None:
        if self.finished:
            return
        self.finished = True
        if self.goal_timer is not None:
            self.goal_timer.cancel()
        if self.finish_timer is not None:
            self.finish_timer.cancel()

        try:
            metrics = self.compute_metrics()
            output_dir = self.save_outputs(metrics)
            self.passed = bool(metrics["pass"])
            self.get_logger().info(
                "Disturbance recovery result: "
                f"pass={self.passed}, "
                f"baseline={metrics['baseline_error_m']:.3f} m, "
                f"peak={metrics['peak_error_m']:.3f} m, "
                f"final={metrics['final_error_m']:.3f} m, "
                f"recovery={metrics['recovery_time_s']} s"
            )
            self.get_logger().info(f"Results saved to: {output_dir}")
        except Exception as exception:  # noqa: BLE001
            self.passed = False
            self.get_logger().error(f"Failed to evaluate experiment: {exception}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DisturbanceExperiment()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        passed = node.passed
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if passed else 2)


if __name__ == "__main__":
    main()
