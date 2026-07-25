#!/usr/bin/env python3
"""Evaluate one 3D A* mission."""

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


class Planner3DEvaluation(Node):
    def __init__(self) -> None:
        super().__init__("planner_3d_evaluation")

        self.declare_parameter("scenario", "overflight_3d")
        self.declare_parameter("mission_mode", "overflight")
        self.declare_parameter("goal_x", 9.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("timeout_seconds", 75.0)
        self.declare_parameter("arrival_tolerance_m", 0.25)
        self.declare_parameter("arrival_hold_seconds", 1.0)
        self.declare_parameter("settle_seconds", 1.2)
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
                / "planner_3d_raw"
            ),
        )

        self.scenario = str(
            self.get_parameter("scenario").value
        )
        self.mission_mode = str(
            self.get_parameter("mission_mode").value
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
        self.planned_points: List[Point3] = []
        self.samples: List[Dict[str, float]] = []

        self.node_started = time.monotonic()
        self.goal_started: Optional[float] = None
        self.arrival_started: Optional[float] = None
        self.arrival_time: Optional[float] = None
        self.finish_after: Optional[float] = None
        self.next_sample = 0.0
        self.goal_sent = False
        self.finished = False

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
            "/drone/planned_path_3d",
            self.path_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/planner_3d/status",
            self.status_callback,
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
            f"3D mission evaluator ready: "
            f"scenario={self.scenario}, "
            f"mode={self.mission_mode}"
        )

    def metadata_callback(self, message: String) -> None:
        data = json.loads(message.data)
        self.voxel_resolution = float(data["resolution_m"])

    def status_callback(self, message: String) -> None:
        data = json.loads(message.data)
        state = data.get("state")

        if state == "ready":
            self.planner_ready = True
        elif state == "path_ready":
            self.planner_status = data
        elif state == "planning_failed" and self.goal_sent:
            self.planner_status = data
            self.finish(False, "planning_failed")

    def path_callback(self, message: NavPath) -> None:
        self.planned_points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

    def rpm_callback(self, message: Float32MultiArray) -> None:
        for index, value in enumerate(list(message.data)[:4]):
            self.latest_rpm[index] = float(value)

    def cloud_callback(self, message: PointCloud2) -> None:
        self.raw_voxel_points = self.pointcloud_to_xyz(message)

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

        self.samples.append({
            "time_s": elapsed,
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "vx_mps": float(message.twist.twist.linear.x),
            "vy_mps": float(message.twist.twist.linear.y),
            "vz_mps": float(message.twist.twist.linear.z),
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
            f"Published 3D goal once: {self.goal}"
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
                "scenario": self.scenario,
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
        rpm_values = [
            abs(float(sample[key]))
            for sample in self.samples
            for key in ("rpm_1", "rpm_2", "rpm_3", "rpm_4")
            if math.isfinite(float(sample[key]))
        ]

        planned_length = polyline_length(self.planned_points)
        actual_length = polyline_length(positions)
        final_error = errors[-1]
        minimum_clearance = (
            min(clearances) if clearances else float("nan")
        )
        maximum_altitude = max(point[2] for point in positions)
        minimum_altitude = min(point[2] for point in positions)
        maximum_rpm = (
            max(rpm_values) if rpm_values else float("nan")
        )
        path_ratio = (
            actual_length / planned_length
            if planned_length > 1.0e-9
            else float("nan")
        )

        planner_succeeded = (
            self.planner_status is not None
            and bool(self.planner_status.get("success", False))
        )
        fixed_height_exists = (
            self.planner_status.get("fixed_height_path_exists")
            if planner_succeeded
            else None
        )

        scenario_behaviour = True
        if self.mission_mode == "overflight":
            scenario_behaviour = (
                fixed_height_exists is False
                and maximum_altitude > 3.15
            )
        elif self.mission_mode == "underpass":
            scenario_behaviour = (
                fixed_height_exists is True
                and maximum_altitude < 2.30
            )

        criteria = {
            "mission_successful": bool(success),
            "planner_3d_succeeded": planner_succeeded,
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
            "scenario_specific_3d_behaviour": scenario_behaviour,
        }

        return {
            "scenario": self.scenario,
            "mission_mode": self.mission_mode,
            "goal": list(self.goal),
            "success": bool(success),
            "failure_reason": failure_reason,
            "overall_pass": all(criteria.values()),
            "criteria": criteria,
            "arrival_time_s": self.arrival_time,
            "final_error_m": final_error,
            "planned_path_length_m": planned_length,
            "actual_path_length_m": actual_length,
            "actual_over_planned_path_ratio": path_ratio,
            "minimum_voxel_clearance_m": minimum_clearance,
            "maximum_altitude_m": maximum_altitude,
            "minimum_altitude_m": minimum_altitude,
            "maximum_motor_rpm": maximum_rpm,
            "planner_status": self.planner_status,
            "sample_count": len(self.samples),
        }

    def save_plots(self, output: Path) -> None:
        times = [float(sample["time_s"]) for sample in self.samples]
        x_values = [float(sample["x_m"]) for sample in self.samples]
        y_values = [float(sample["y_m"]) for sample in self.samples]
        z_values = [float(sample["z_m"]) for sample in self.samples]
        errors = [
            float(sample["position_error_m"])
            for sample in self.samples
        ]
        clearances = [
            float(sample["voxel_clearance_m"])
            for sample in self.samples
        ]

        plt.figure(figsize=(8.2, 5.0))
        plt.plot(times, z_values)
        plt.xlabel("time (s)")
        plt.ylabel("altitude z (m)")
        plt.title(f"{self.scenario}: altitude profile")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output / "altitude_profile.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.2, 5.0))
        plt.plot(times, errors)
        plt.axhline(
            self.arrival_tolerance,
            linestyle="--",
            label="arrival tolerance",
        )
        plt.xlabel("time (s)")
        plt.ylabel("position error (m)")
        plt.title(f"{self.scenario}: position error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output / "position_error.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.2, 5.0))
        plt.plot(times, clearances)
        plt.axhline(
            self.minimum_clearance,
            linestyle="--",
            label="required clearance",
        )
        plt.xlabel("time (s)")
        plt.ylabel("body clearance (m)")
        plt.title(f"{self.scenario}: voxel clearance")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output / "voxel_clearance.png", dpi=180)
        plt.close()

        plt.figure(figsize=(7.5, 6.2))
        if self.planned_points:
            plt.plot(
                [point[0] for point in self.planned_points],
                [point[1] for point in self.planned_points],
                linestyle="--",
                label="planned XY projection",
            )
        plt.plot(
            x_values,
            y_values,
            label="actual XY projection",
        )
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title(f"{self.scenario}: horizontal trajectory")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(output / "trajectory_xy.png", dpi=180)
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
            / f"{self.scenario}_{timestamp}"
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

        metrics = self.compute_metrics(
            success,
            failure_reason,
        )
        (output / "metrics.json").write_text(
            json.dumps(
                metrics,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        lines = [
            "===== 3D A* Mission Summary =====",
            f"Scenario: {metrics['scenario']}",
            f"Mode: {metrics['mission_mode']}",
            f"Success: {metrics['success']}",
            f"Overall pass: {metrics['overall_pass']}",
            f"Arrival time: {metrics['arrival_time_s']} s",
            f"Final error: {metrics['final_error_m']:.4f} m",
            (
                "Planned path length: "
                f"{metrics['planned_path_length_m']:.3f} m"
            ),
            (
                "Actual path length: "
                f"{metrics['actual_path_length_m']:.3f} m"
            ),
            (
                "Actual/planned ratio: "
                f"{metrics['actual_over_planned_path_ratio']:.3f}"
            ),
            (
                "Minimum voxel clearance: "
                f"{metrics['minimum_voxel_clearance_m']:.3f} m"
            ),
            (
                "Maximum altitude: "
                f"{metrics['maximum_altitude_m']:.3f} m"
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
            f"3D mission result: {output}"
        )
        self.get_logger().info(
            f"Overall pass: {metrics['overall_pass']}"
        )

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Planner3DEvaluation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
