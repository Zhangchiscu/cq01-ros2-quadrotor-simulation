#!/usr/bin/env python3
"""Evaluate raw versus smoothed 3D trajectory execution."""

from __future__ import annotations

import csv
import json
import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String

Point3 = Tuple[float, float, float]


def polyline_length(
    points: Sequence[Sequence[float]],
) -> float:
    return sum(
        math.dist(first, second)
        for first, second in zip(points[:-1], points[1:])
    )


class TrajectoryEvaluation(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_evaluation")

        self.declare_parameter("mode", "smooth")
        self.declare_parameter("scenario", "overflight_3d")
        self.declare_parameter("goal_x", 9.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("timeout_seconds", 80.0)
        self.declare_parameter("arrival_tolerance_m", 0.25)
        self.declare_parameter("arrival_hold_seconds", 1.0)
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("drone_radius_m", 0.15)
        self.declare_parameter("minimum_clearance_m", 0.30)
        self.declare_parameter("rpm_limit", 10000.0)
        self.declare_parameter(
            "results_root",
            str(
                Path.home()
                / "drone_task"
                / "your_drone_sim_ws"
                / "results"
                / "trajectory_raw"
            ),
        )

        self.mode = str(self.get_parameter("mode").value)
        self.scenario = str(
            self.get_parameter("scenario").value
        )
        self.goal: Point3 = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        self.warmup_seconds = float(
            self.get_parameter("warmup_seconds").value
        )
        self.timeout_seconds = float(
            self.get_parameter("timeout_seconds").value
        )
        self.arrival_tolerance = float(
            self.get_parameter("arrival_tolerance_m").value
        )
        self.arrival_hold_seconds = float(
            self.get_parameter("arrival_hold_seconds").value
        )
        self.settle_seconds = float(
            self.get_parameter("settle_seconds").value
        )
        self.sample_period = 1.0 / float(
            self.get_parameter("sample_rate_hz").value
        )
        self.drone_radius = float(
            self.get_parameter("drone_radius_m").value
        )
        self.minimum_clearance = float(
            self.get_parameter("minimum_clearance_m").value
        )
        self.rpm_limit = float(
            self.get_parameter("rpm_limit").value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter("results_root").value)
            )
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_rpm = [float("nan")] * 4
        self.raw_voxel_points: Optional[np.ndarray] = None
        self.voxel_resolution: Optional[float] = None
        self.planner_ready = False
        self.planner_status: Optional[Dict[str, object]] = None
        self.trajectory_status: Optional[Dict[str, object]] = None
        self.reference_points: List[Point3] = []
        self.samples: List[Dict[str, float]] = []

        self.node_started = time.monotonic()
        self.goal_started: Optional[float] = None
        self.arrival_started: Optional[float] = None
        self.arrival_time: Optional[float] = None
        self.finish_after: Optional[float] = None
        self.next_sample = 0.0
        self.goal_sent = False
        self.finished = False
        self.previous_velocity: Optional[np.ndarray] = None
        self.previous_acceleration: Optional[np.ndarray] = None
        self.previous_sample_time: Optional[float] = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            "/drone/motor_rpm_cmd",
            self.rpm_callback,
            20,
        )
        self.create_subscription(
            NavPath,
            "/drone/trajectory_reference",
            self.reference_path_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/planner_3d/status",
            self.planner_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/trajectory/status",
            self.trajectory_status_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            "/drone/voxel_cloud_raw",
            self.cloud_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/voxel_metadata",
            self.metadata_callback,
            transient_qos,
        )

        self.timer = self.create_timer(0.05, self.control_step)

        self.get_logger().info(
            f"Trajectory evaluation ready: mode={self.mode}"
        )

    def planner_status_callback(self, message: String) -> None:
        data = json.loads(message.data)
        state = data.get("state")
        if state == "ready":
            self.planner_ready = True
        elif state == "path_ready":
            self.planner_status = data
        elif state == "planning_failed" and self.goal_sent:
            self.finish(False, "planning_failed")

    def trajectory_status_callback(self, message: String) -> None:
        data = json.loads(message.data)
        state = data.get("state")
        if state in {"trajectory_ready", "trajectory_complete"}:
            self.trajectory_status = data
        elif state == "trajectory_failed" and self.goal_sent:
            self.trajectory_status = data
            self.finish(False, "trajectory_failed")

    def reference_path_callback(self, message: NavPath) -> None:
        self.reference_points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

    def metadata_callback(self, message: String) -> None:
        data = json.loads(message.data)
        self.voxel_resolution = float(data["resolution_m"])

    def cloud_callback(self, message: PointCloud2) -> None:
        self.raw_voxel_points = self.pointcloud_to_xyz(message)

    def rpm_callback(self, message: Float32MultiArray) -> None:
        for index, value in enumerate(list(message.data)[:4]):
            self.latest_rpm[index] = float(value)

    def pointcloud_to_xyz(self, message: PointCloud2) -> np.ndarray:
        point_count = int(message.width) * int(message.height)
        if point_count <= 0:
            return np.empty((0, 3), dtype=np.float64)

        offsets = {
            field.name: int(field.offset)
            for field in message.fields
        }
        endian = ">" if message.is_bigendian else "<"
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [
                f"{endian}f4",
                f"{endian}f4",
                f"{endian}f4",
            ],
            "offsets": [
                offsets["x"],
                offsets["y"],
                offsets["z"],
            ],
            "itemsize": int(message.point_step),
        })
        structured = np.frombuffer(
            message.data,
            dtype=dtype,
            count=point_count,
        )
        points = np.column_stack((
            structured["x"],
            structured["y"],
            structured["z"],
        )).astype(np.float64, copy=False)
        return points[np.isfinite(points).all(axis=1)]

    def voxel_clearance(self, point: Point3) -> float:
        if (
            self.raw_voxel_points is None
            or self.voxel_resolution is None
            or self.raw_voxel_points.size == 0
        ):
            return float("nan")

        position = np.asarray(point, dtype=np.float64)
        squared_distances = np.sum(
            (self.raw_voxel_points - position) ** 2,
            axis=1,
        )
        center_distance = math.sqrt(
            float(np.min(squared_distances))
        )
        voxel_half_diagonal = (
            math.sqrt(3.0)
            * self.voxel_resolution
            / 2.0
        )
        return (
            center_distance
            - voxel_half_diagonal
            - self.drone_radius
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

        if self.goal_started is None or self.finished:
            return

        now = time.monotonic()
        elapsed = now - self.goal_started
        position: Point3 = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        velocity = np.asarray([
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
            float(message.twist.twist.linear.z),
        ], dtype=np.float64)
        error = math.dist(position, self.goal)

        if error <= self.arrival_tolerance:
            if self.arrival_started is None:
                self.arrival_started = now
            elif (
                self.arrival_time is None
                and now - self.arrival_started
                >= self.arrival_hold_seconds
            ):
                self.arrival_time = elapsed
                self.finish_after = now + self.settle_seconds
        else:
            self.arrival_started = None

        if now < self.next_sample:
            return
        self.next_sample = now + self.sample_period

        acceleration = np.zeros(3, dtype=np.float64)
        jerk = np.zeros(3, dtype=np.float64)

        if (
            self.previous_velocity is not None
            and self.previous_sample_time is not None
        ):
            dt = max(now - self.previous_sample_time, 1.0e-6)
            acceleration = (
                velocity - self.previous_velocity
            ) / dt

            if self.previous_acceleration is not None:
                jerk = (
                    acceleration - self.previous_acceleration
                ) / dt

        self.previous_velocity = velocity.copy()
        self.previous_acceleration = acceleration.copy()
        self.previous_sample_time = now

        self.samples.append({
            "time_s": elapsed,
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "speed_mps": float(np.linalg.norm(velocity)),
            "acceleration_mps2": float(
                np.linalg.norm(acceleration)
            ),
            "jerk_mps3": float(np.linalg.norm(jerk)),
            "position_error_m": error,
            "voxel_clearance_m": self.voxel_clearance(position),
            "rpm_1": self.latest_rpm[0],
            "rpm_2": self.latest_rpm[1],
            "rpm_3": self.latest_rpm[2],
            "rpm_4": self.latest_rpm[3],
        })

    def ready(self) -> bool:
        return (
            self.latest_odom is not None
            and self.raw_voxel_points is not None
            and self.voxel_resolution is not None
            and self.planner_ready
        )

    def publish_goal(self) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.position.z = self.goal[2]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)

        self.goal_started = time.monotonic()
        self.next_sample = self.goal_started
        self.goal_sent = True
        self.get_logger().info(
            f"Published planner goal once: {self.goal}"
        )

    def control_step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if not self.goal_sent:
            if now - self.node_started < self.warmup_seconds:
                return
            if not self.ready():
                return
            self.publish_goal()
            return

        assert self.goal_started is not None

        if (
            self.finish_after is not None
            and now >= self.finish_after
        ):
            self.finish(True, "")
            return

        if now - self.goal_started >= self.timeout_seconds:
            self.finish(False, "timeout")

    def compute_metrics(
        self,
        success: bool,
        failure_reason: str,
    ) -> Dict[str, object]:
        if not self.samples:
            return {
                "mode": self.mode,
                "success": False,
                "overall_pass": False,
                "failure_reason": "no_samples",
            }

        positions = [
            (sample["x_m"], sample["y_m"], sample["z_m"])
            for sample in self.samples
        ]
        errors = [
            float(sample["position_error_m"])
            for sample in self.samples
        ]
        clearances = [
            float(sample["voxel_clearance_m"])
            for sample in self.samples
            if math.isfinite(float(sample["voxel_clearance_m"]))
        ]
        speed_values = np.asarray(
            [float(sample["speed_mps"]) for sample in self.samples],
            dtype=np.float64,
        )
        acceleration_values = np.asarray(
            [
                float(sample["acceleration_mps2"])
                for sample in self.samples
            ],
            dtype=np.float64,
        )
        jerk_values = np.asarray(
            [float(sample["jerk_mps3"]) for sample in self.samples],
            dtype=np.float64,
        )
        rpm_values = [
            abs(float(sample[key]))
            for sample in self.samples
            for key in ("rpm_1", "rpm_2", "rpm_3", "rpm_4")
            if math.isfinite(float(sample[key]))
        ]

        reference_length = polyline_length(self.reference_points)
        actual_length = polyline_length(positions)
        final_error = errors[-1]
        minimum_clearance = (
            min(clearances) if clearances else float("nan")
        )
        maximum_rpm = (
            max(rpm_values) if rpm_values else float("nan")
        )
        path_ratio = (
            actual_length / reference_length
            if reference_length > 1.0e-9
            else float("nan")
        )

        planner_succeeded = (
            self.planner_status is not None
            and bool(self.planner_status.get("success", False))
        )
        trajectory_ready = (
            self.trajectory_status is not None
            and bool(self.trajectory_status.get("success", False))
        )

        criteria = {
            "mission_successful": bool(success),
            "planner_3d_succeeded": planner_succeeded,
            "trajectory_generated": trajectory_ready,
            "collision_checked_reference": (
                trajectory_ready
                and bool(
                    self.trajectory_status.get(
                        "collision_checked",
                        False,
                    )
                )
            ),
            "final_error_below_0_3_m": final_error < 0.30,
            "physical_clearance_above_0_3_m": (
                math.isfinite(minimum_clearance)
                and minimum_clearance >= self.minimum_clearance
            ),
            "path_ratio_below_1_5": (
                math.isfinite(path_ratio)
                and path_ratio < 1.50
            ),
            "no_motor_saturation": (
                math.isfinite(maximum_rpm)
                and maximum_rpm < self.rpm_limit
            ),
        }

        return {
            "mode": self.mode,
            "scenario": self.scenario,
            "goal": list(self.goal),
            "success": bool(success),
            "failure_reason": failure_reason,
            "overall_pass": all(criteria.values()),
            "criteria": criteria,
            "arrival_time_s": self.arrival_time,
            "final_error_m": final_error,
            "reference_path_length_m": reference_length,
            "actual_path_length_m": actual_length,
            "actual_over_reference_ratio": path_ratio,
            "minimum_voxel_clearance_m": minimum_clearance,
            "maximum_speed_mps": float(np.max(speed_values)),
            "maximum_acceleration_mps2": float(
                np.max(acceleration_values)
            ),
            "rms_acceleration_mps2": float(
                math.sqrt(np.mean(acceleration_values ** 2))
            ),
            "maximum_jerk_mps3": float(np.max(jerk_values)),
            "rms_jerk_mps3": float(
                math.sqrt(np.mean(jerk_values ** 2))
            ),
            "maximum_motor_rpm": maximum_rpm,
            "planner_status": self.planner_status,
            "trajectory_status": self.trajectory_status,
            "sample_count": len(self.samples),
        }

    def save_plots(self, output: Path) -> None:
        times = [
            float(sample["time_s"])
            for sample in self.samples
        ]

        plots = [
            (
                "speed_profile.png",
                "speed_mps",
                "speed (m/s)",
                f"{self.mode}: actual speed",
            ),
            (
                "acceleration_profile.png",
                "acceleration_mps2",
                "acceleration (m/s²)",
                f"{self.mode}: actual acceleration",
            ),
            (
                "jerk_profile.png",
                "jerk_mps3",
                "jerk (m/s³)",
                f"{self.mode}: actual jerk",
            ),
            (
                "clearance_profile.png",
                "voxel_clearance_m",
                "body clearance (m)",
                f"{self.mode}: voxel clearance",
            ),
        ]

        for filename, key, ylabel, title in plots:
            plt.figure(figsize=(8.2, 5.0))
            plt.plot(
                times,
                [float(sample[key]) for sample in self.samples],
            )
            if key == "voxel_clearance_m":
                plt.axhline(
                    self.minimum_clearance,
                    linestyle="--",
                    label="required clearance",
                )
                plt.legend()
            plt.xlabel("time (s)")
            plt.ylabel(ylabel)
            plt.title(title)
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(output / filename, dpi=180)
            plt.close()

    def finish(
        self,
        success: bool,
        failure_reason: str,
    ) -> None:
        if self.finished:
            return
        self.finished = True

        if not self.samples:
            if rclpy.ok():
                rclpy.shutdown()
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output = (
            self.results_root
            / f"{self.mode}_{timestamp}"
        )
        output.mkdir(parents=True, exist_ok=False)

        with (output / "flight_data.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as handle:
            fieldnames = list(self.samples[0].keys())
            writer = csv.DictWriter(
                handle,
                fieldnames=fieldnames,
            )
            writer.writeheader()
            writer.writerows(self.samples)

        metrics = self.compute_metrics(success, failure_reason)
        (output / "metrics.json").write_text(
            json.dumps(
                metrics,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        lines = [
            "===== Trajectory Execution Summary =====",
            f"Mode: {metrics['mode']}",
            f"Success: {metrics['success']}",
            f"Overall pass: {metrics['overall_pass']}",
            f"Arrival time: {metrics['arrival_time_s']} s",
            f"Final error: {metrics['final_error_m']:.4f} m",
            (
                "Reference path length: "
                f"{metrics['reference_path_length_m']:.3f} m"
            ),
            (
                "Actual path length: "
                f"{metrics['actual_path_length_m']:.3f} m"
            ),
            (
                "Minimum clearance: "
                f"{metrics['minimum_voxel_clearance_m']:.3f} m"
            ),
            (
                "Maximum acceleration: "
                f"{metrics['maximum_acceleration_mps2']:.3f} m/s^2"
            ),
            (
                "RMS acceleration: "
                f"{metrics['rms_acceleration_mps2']:.3f} m/s^2"
            ),
            (
                "Maximum jerk: "
                f"{metrics['maximum_jerk_mps3']:.3f} m/s^3"
            ),
            (
                "RMS jerk: "
                f"{metrics['rms_jerk_mps3']:.3f} m/s^3"
            ),
            (
                "Maximum motor RPM: "
                f"{metrics['maximum_motor_rpm']:.1f}"
            ),
            "",
            "Acceptance criteria:",
        ]
        for key, passed in metrics["criteria"].items():
            lines.append(
                f"- {key}: {'PASS' if passed else 'FAIL'}"
            )

        (output / "summary.txt").write_text(
            "\n".join(lines) + "\n",
            encoding="utf-8",
        )
        self.save_plots(output)

        self.get_logger().info(
            f"Trajectory result: {output}"
        )
        self.get_logger().info(
            f"Overall pass: {metrics['overall_pass']}"
        )

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryEvaluation()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
