#!/usr/bin/env python3
"""Evaluate circle and figure-eight tracking performance."""

from __future__ import annotations

import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class PatternEvaluation(Node):
    def __init__(self) -> None:
        super().__init__("pattern_trajectory_evaluation")

        self.declare_parameter("pattern", "circle")
        self.declare_parameter("timeout_seconds", 50.0)
        self.declare_parameter("settle_seconds", 0.8)
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("rpm_limit", 12000.0)
        self.declare_parameter("results_root", "~/drone_task/your_drone_sim_ws/results/pattern_trajectory")

        self.pattern = str(self.get_parameter("pattern").value)
        self.timeout = float(self.get_parameter("timeout_seconds").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.sample_rate = float(self.get_parameter("sample_rate_hz").value)
        self.rpm_limit = float(self.get_parameter("rpm_limit").value)
        self.results_root = Path(
            str(self.get_parameter("results_root").value)
        ).expanduser()
        self.results_root.mkdir(parents=True, exist_ok=True)

        self.latest_odom: Optional[Odometry] = None
        self.latest_goal: Optional[PoseStamped] = None
        self.latest_rpm: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.latest_status: Dict[str, object] = {}
        self.started_wall = time.monotonic()
        self.pattern_started: Optional[float] = None
        self.complete_seen: Optional[float] = None
        self.finished = False
        self.records: List[Dict[str, float]] = []

        self.create_subscription(Odometry, "/drone/odom", self.odom_callback, 50)
        self.create_subscription(PoseStamped, "/drone/goal", self.goal_callback, 50)
        self.create_subscription(Float32MultiArray, "/drone/motor_rpm", self.rpm_callback, 50)
        self.create_subscription(String, "/drone/pattern/status", self.status_callback, 10)
        self.timer = self.create_timer(1.0 / self.sample_rate, self.sample)

        self.get_logger().info(f"Pattern evaluation started: {self.pattern}")

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def goal_callback(self, message: PoseStamped) -> None:
        self.latest_goal = message

    def rpm_callback(self, message: Float32MultiArray) -> None:
        values = [float(value) for value in message.data[:4]]
        if len(values) == 4:
            self.latest_rpm = values

    def status_callback(self, message: String) -> None:
        try:
            self.latest_status = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(self.latest_status.get("state", ""))
        if state == "pattern_running" and self.pattern_started is None:
            self.pattern_started = time.monotonic()
        if state == "pattern_complete" and self.complete_seen is None:
            self.complete_seen = time.monotonic()

    def sample(self) -> None:
        if self.finished:
            return

        now = time.monotonic()
        if now - self.started_wall > self.timeout:
            self.finish(False, "timeout")
            return

        if self.latest_odom is None or self.latest_goal is None:
            return

        position = self.latest_odom.pose.pose.position
        velocity = self.latest_odom.twist.twist.linear
        orientation = self.latest_odom.pose.pose.orientation
        goal = self.latest_goal.pose.position
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        tracking_error = math.sqrt(
            (position.x - goal.x) ** 2
            + (position.y - goal.y) ** 2
            + (position.z - goal.z) ** 2
        )
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        tilt = math.degrees(math.sqrt(roll**2 + pitch**2))
        progress = float(self.latest_status.get("progress", 0.0) or 0.0)
        state = str(self.latest_status.get("state", "unknown"))

        self.records.append({
            "elapsed_s": now - self.started_wall,
            "x_m": float(position.x),
            "y_m": float(position.y),
            "z_m": float(position.z),
            "goal_x_m": float(goal.x),
            "goal_y_m": float(goal.y),
            "goal_z_m": float(goal.z),
            "tracking_error_m": tracking_error,
            "altitude_error_m": float(position.z - goal.z),
            "speed_mps": speed,
            "tilt_deg": tilt,
            "yaw_deg": math.degrees(yaw),
            "rpm_fl": self.latest_rpm[0],
            "rpm_fr": self.latest_rpm[1],
            "rpm_rr": self.latest_rpm[2],
            "rpm_rl": self.latest_rpm[3],
            "progress": progress,
            "state": state,
        })

        if self.complete_seen is not None and now - self.complete_seen >= self.settle_seconds:
            self.finish(True, "pattern_complete")

    def finish(self, completed: bool, reason: str) -> None:
        if self.finished:
            return
        self.finished = True

        output = self.results_root / f"{self.pattern}_{time.strftime('%Y%m%d_%H%M%S')}"
        output.mkdir(parents=True, exist_ok=False)

        if self.records:
            with (output / "telemetry.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.records[0].keys()))
                writer.writeheader()
                writer.writerows(self.records)

        metrics = self.compute_metrics(completed, reason)
        (output / "summary.json").write_text(
            json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.create_plots(output)

        self.get_logger().info(
            f"Pattern result: {output} pass={metrics['pass']} "
            f"rms={metrics['rms_tracking_error_m']:.3f} m "
            f"max={metrics['maximum_tracking_error_m']:.3f} m"
        )
        raise SystemExit(0 if metrics["pass"] else 2)

    def compute_metrics(self, completed: bool, reason: str) -> Dict[str, object]:
        if not self.records:
            return {
                "experiment": "parametric_pattern_tracking",
                "pattern": self.pattern,
                "reason": reason,
                "sample_count": 0,
                "pass": False,
            }

        running = [record for record in self.records if record["state"] == "pattern_running"]
        analysis = running if len(running) >= 50 else self.records
        errors = np.asarray([record["tracking_error_m"] for record in analysis], dtype=float)
        altitude_errors = np.asarray([record["altitude_error_m"] for record in analysis], dtype=float)
        speeds = np.asarray([record["speed_mps"] for record in self.records], dtype=float)
        tilts = np.asarray([record["tilt_deg"] for record in self.records], dtype=float)
        rpms = np.asarray([
            value
            for record in self.records
            for value in (record["rpm_fl"], record["rpm_fr"], record["rpm_rr"], record["rpm_rl"])
        ], dtype=float)

        final = self.records[-1]
        final_error = float(final["tracking_error_m"])
        rpm_saturation_fraction = float(np.mean(rpms >= 0.98 * self.rpm_limit)) if len(rpms) else 0.0
        minimum_samples = 250
        rms_limit = 0.45 if self.pattern == "circle" else 0.55
        max_limit = 1.10 if self.pattern == "circle" else 1.25

        criteria = {
            "pattern_completed": bool(completed),
            "enough_samples": len(analysis) >= minimum_samples,
            "rms_tracking_error_pass": float(np.sqrt(np.mean(errors**2))) <= rms_limit,
            "maximum_tracking_error_pass": float(np.max(errors)) <= max_limit,
            "final_error_pass": final_error <= 0.25,
            "altitude_rmse_pass": float(np.sqrt(np.mean(altitude_errors**2))) <= 0.18,
            "tilt_pass": float(np.max(tilts)) <= 35.0,
            "rpm_saturation_pass": rpm_saturation_fraction <= 0.02,
        }

        return {
            "experiment": "parametric_pattern_tracking",
            "pattern": self.pattern,
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "reason": reason,
            "sample_count": len(self.records),
            "pattern_sample_count": len(analysis),
            "rms_tracking_error_m": float(np.sqrt(np.mean(errors**2))),
            "p95_tracking_error_m": float(np.percentile(errors, 95)),
            "maximum_tracking_error_m": float(np.max(errors)),
            "final_tracking_error_m": final_error,
            "altitude_rmse_m": float(np.sqrt(np.mean(altitude_errors**2))),
            "maximum_speed_mps": float(np.max(speeds)),
            "maximum_tilt_deg": float(np.max(tilts)),
            "maximum_rpm": float(np.max(rpms)) if len(rpms) else 0.0,
            "rpm_saturation_fraction": rpm_saturation_fraction,
            "criteria": criteria,
            "pass": all(criteria.values()),
        }

    def create_plots(self, output: Path) -> None:
        if not self.records:
            return
        x = [record["x_m"] for record in self.records]
        y = [record["y_m"] for record in self.records]
        gx = [record["goal_x_m"] for record in self.records]
        gy = [record["goal_y_m"] for record in self.records]
        t = [record["elapsed_s"] for record in self.records]
        error = [record["tracking_error_m"] for record in self.records]
        z = [record["z_m"] for record in self.records]
        gz = [record["goal_z_m"] for record in self.records]

        plt.figure(figsize=(7.2, 6.0))
        plt.plot(gx, gy, label="Reference")
        plt.plot(x, y, label="Actual")
        plt.axis("equal")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"{self.pattern.replace('_', ' ').title()} tracking")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output / "xy_tracking.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.0, 4.6))
        plt.plot(t, error)
        plt.xlabel("Time [s]")
        plt.ylabel("Tracking error [m]")
        plt.title("Reference tracking error")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output / "tracking_error.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.0, 4.6))
        plt.plot(t, gz, label="Reference altitude")
        plt.plot(t, z, label="Actual altitude")
        plt.xlabel("Time [s]")
        plt.ylabel("Altitude [m]")
        plt.title("Altitude tracking")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output / "altitude_tracking.png", dpi=180)
        plt.close()


def main() -> None:
    rclpy.init()
    node = PatternEvaluation()
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        exit_code = int(exc.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
