#!/usr/bin/env python3
"""Run and evaluate one 2D A* mission in a drone_world scenario."""

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
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray


Vector3 = Tuple[float, float, float]


def distance(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    return math.sqrt(
        sum(
            (float(a) - float(b)) ** 2
            for a, b in zip(first, second)
        )
    )


def polyline_length(
    points: Sequence[Sequence[float]],
) -> float:
    if len(points) < 2:
        return 0.0

    return sum(
        distance(points[index - 1], points[index])
        for index in range(1, len(points))
    )


class ScenarioMission(Node):
    def __init__(self) -> None:
        super().__init__("scenario_mission")

        self.declare_parameter("scenario", "warehouse")
        self.declare_parameter("goal_x", 6.8)
        self.declare_parameter("goal_y", 4.5)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("warmup_seconds", 3.0)
        self.declare_parameter("timeout_seconds", 55.0)
        self.declare_parameter("arrival_tolerance_m", 0.25)
        self.declare_parameter("arrival_hold_seconds", 1.0)
        self.declare_parameter("settle_seconds", 1.5)
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
                / "world_validation_raw"
            ),
        )

        self.scenario = str(
            self.get_parameter("scenario").value
        )
        self.goal: Vector3 = (
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
        self.grid: Optional[OccupancyGrid] = None
        self.cloud_points: Optional[np.ndarray] = None
        self.planned_points: List[Vector3] = []

        self.samples: List[Dict[str, float]] = []
        self.node_start = time.monotonic()
        self.goal_start: Optional[float] = None
        self.arrival_hold_start: Optional[float] = None
        self.arrival_time: Optional[float] = None
        self.finish_after: Optional[float] = None
        self.next_sample_time = 0.0
        self.goal_sent = False
        self.finished = False

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
            "/drone/planned_path",
            self.path_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/drone/obstacle_grid_safety",
            self.grid_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            "/drone/obstacle_cloud",
            self.cloud_callback,
            transient_qos,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_step,
        )

        self.get_logger().info(
            "Scenario mission started: "
            f"scenario={self.scenario}, "
            f"goal=({self.goal[0]:.2f}, "
            f"{self.goal[1]:.2f}, "
            f"{self.goal[2]:.2f})"
        )

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        values = list(message.data)
        for index in range(min(4, len(values))):
            self.latest_rpm[index] = float(values[index])

    def grid_callback(
        self,
        message: OccupancyGrid,
    ) -> None:
        self.grid = message

    def path_callback(
        self,
        message: NavPath,
    ) -> None:
        if len(message.poses) < 2:
            return

        self.planned_points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

    def cloud_callback(
        self,
        message: PointCloud2,
    ) -> None:
        point_count = int(message.width) * int(message.height)

        if point_count <= 0:
            self.cloud_points = np.empty((0, 3))
            return

        field_offsets = {
            field.name: int(field.offset)
            for field in message.fields
        }

        required = {"x", "y", "z"}
        if not required.issubset(field_offsets):
            self.get_logger().error(
                "PointCloud2 does not contain x/y/z fields"
            )
            return

        endian = ">" if message.is_bigendian else "<"
        data_type = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [
                f"{endian}f4",
                f"{endian}f4",
                f"{endian}f4",
            ],
            "offsets": [
                field_offsets["x"],
                field_offsets["y"],
                field_offsets["z"],
            ],
            "itemsize": int(message.point_step),
        })

        structured = np.frombuffer(
            message.data,
            dtype=data_type,
            count=point_count,
        )
        points = np.column_stack((
            structured["x"],
            structured["y"],
            structured["z"],
        )).astype(np.float64, copy=False)

        finite_mask = np.isfinite(points).all(axis=1)
        self.cloud_points = points[finite_mask]

    def grid_value_at(
        self,
        x: float,
        y: float,
    ) -> Optional[int]:
        if self.grid is None:
            return None

        resolution = float(self.grid.info.resolution)
        origin_x = float(self.grid.info.origin.position.x)
        origin_y = float(self.grid.info.origin.position.y)

        column = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))

        if (
            column < 0
            or row < 0
            or column >= int(self.grid.info.width)
            or row >= int(self.grid.info.height)
        ):
            return None

        return int(
            self.grid.data[
                row * int(self.grid.info.width) + column
            ]
        )

    def nearest_cloud_clearance(
        self,
        position: Vector3,
    ) -> float:
        if self.cloud_points is None:
            return float("nan")
        if self.cloud_points.size == 0:
            return float("inf")

        position_array = np.asarray(
            position,
            dtype=np.float64,
        )
        squared_distances = np.sum(
            (self.cloud_points - position_array) ** 2,
            axis=1,
        )
        nearest_distance = math.sqrt(
            float(np.min(squared_distances))
        )
        return nearest_distance - self.drone_radius

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.latest_odom = message

        if self.goal_start is None or self.finished:
            return

        now = time.monotonic()
        elapsed = now - self.goal_start
        position: Vector3 = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        error = distance(position, self.goal)

        if error <= self.arrival_tolerance:
            if self.arrival_hold_start is None:
                self.arrival_hold_start = now
            elif (
                self.arrival_time is None
                and now - self.arrival_hold_start
                >= self.arrival_hold_seconds
            ):
                self.arrival_time = elapsed
                self.finish_after = now + self.settle_seconds
                self.get_logger().info(
                    f"Goal reached in {elapsed:.3f} s"
                )
        else:
            self.arrival_hold_start = None

        if now < self.next_sample_time:
            return
        self.next_sample_time = now + self.sample_period

        grid_value = self.grid_value_at(
            position[0],
            position[1],
        )
        clearance = self.nearest_cloud_clearance(position)

        sample: Dict[str, float] = {
            "time_s": elapsed,
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "vx_mps": float(message.twist.twist.linear.x),
            "vy_mps": float(message.twist.twist.linear.y),
            "vz_mps": float(message.twist.twist.linear.z),
            "position_error_m": error,
            "pointcloud_clearance_m": clearance,
            "inflated_grid_value": (
                float(grid_value)
                if grid_value is not None
                else float("nan")
            ),
            "rpm_front_left": self.latest_rpm[0],
            "rpm_front_right": self.latest_rpm[1],
            "rpm_rear_right": self.latest_rpm[2],
            "rpm_rear_left": self.latest_rpm[3],
        }
        self.samples.append(sample)

    def ready(self) -> bool:
        return (
            self.latest_odom is not None
            and self.grid is not None
            and self.cloud_points is not None
        )

    def publish_goal(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.position.z = self.goal[2]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)

        self.goal_start = time.monotonic()
        self.next_sample_time = self.goal_start
        self.goal_sent = True

        self.get_logger().info(
            "Raw planner goal published once"
        )

    def control_step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if not self.goal_sent:
            elapsed = now - self.node_start
            if elapsed < self.warmup_seconds:
                return
            if not self.ready():
                if int(elapsed) % 5 == 0:
                    self.get_logger().info(
                        "Waiting for odom, inflated map and point cloud"
                    )
                return
            self.publish_goal()
            return

        assert self.goal_start is not None
        mission_elapsed = now - self.goal_start

        if self.finish_after is not None and now >= self.finish_after:
            self.finish(success=True)
            return

        if mission_elapsed >= self.timeout_seconds:
            self.finish(success=False)

    def metric_values(
        self,
        success: bool,
    ) -> Dict[str, object]:
        if not self.samples:
            return {
                "scenario": self.scenario,
                "success": False,
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
            float(sample["pointcloud_clearance_m"])
            for sample in self.samples
            if math.isfinite(
                float(sample["pointcloud_clearance_m"])
            )
        ]
        grid_values = [
            float(sample["inflated_grid_value"])
            for sample in self.samples
            if math.isfinite(
                float(sample["inflated_grid_value"])
            )
        ]
        rpm_values = [
            abs(float(sample[key]))
            for sample in self.samples
            for key in [
                "rpm_front_left",
                "rpm_front_right",
                "rpm_rear_right",
                "rpm_rear_left",
            ]
            if math.isfinite(float(sample[key]))
        ]

        actual_length = polyline_length(positions)
        planned_length = polyline_length(self.planned_points)
        final_error = errors[-1]

        tail_count = max(
            1,
            int(round(1.0 / self.sample_period)),
        )
        steady_error = sum(errors[-tail_count:]) / min(
            tail_count,
            len(errors),
        )

        maximum_rpm = max(rpm_values) if rpm_values else float("nan")
        minimum_clearance = (
            min(clearances)
            if clearances
            else float("nan")
        )
        grid_collision_samples = sum(
            1 for value in grid_values if value >= 50.0
        )
        path_ratio = (
            actual_length / planned_length
            if planned_length > 1.0e-9
            else float("nan")
        )

        criteria = {
            "mission_successful": bool(success),
            "planned_path_received": (
                len(self.planned_points) >= 2
            ),
            "final_error_below_0_3_m": final_error < 0.30,
            "safety_grid_collision_free": (
                grid_collision_samples == 0
            ),
            "physical_clearance_above_required_margin": (
                math.isfinite(minimum_clearance)
                and minimum_clearance
                >= self.minimum_clearance
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
            "scenario": self.scenario,
            "goal": list(self.goal),
            "success": bool(success),
            "overall_pass": all(criteria.values()),
            "criteria": criteria,
            "arrival_time_s": self.arrival_time,
            "final_error_m": final_error,
            "steady_state_error_m": steady_error,
            "planned_path_length_m": planned_length,
            "actual_path_length_m": actual_length,
            "actual_over_planned_path_ratio": path_ratio,
            "minimum_pointcloud_clearance_m": minimum_clearance,
            "safety_grid_collision_samples": grid_collision_samples,
            "required_physical_clearance_m": self.minimum_clearance,
            "maximum_motor_rpm": maximum_rpm,
            "sample_count": len(self.samples),
            "planned_waypoint_count": len(self.planned_points),
        }

    def save_plots(
        self,
        output_directory: Path,
    ) -> None:
        times = [
            float(sample["time_s"])
            for sample in self.samples
        ]
        x_values = [
            float(sample["x_m"])
            for sample in self.samples
        ]
        y_values = [
            float(sample["y_m"])
            for sample in self.samples
        ]
        errors = [
            float(sample["position_error_m"])
            for sample in self.samples
        ]
        clearances = [
            float(sample["pointcloud_clearance_m"])
            for sample in self.samples
        ]

        plt.figure(figsize=(7.5, 6.5))
        if self.planned_points:
            plt.plot(
                [point[0] for point in self.planned_points],
                [point[1] for point in self.planned_points],
                linestyle="--",
                label="planned path",
            )
        plt.plot(
            x_values,
            y_values,
            label="actual path",
        )
        plt.scatter(
            [self.goal[0]],
            [self.goal[1]],
            marker="*",
            s=160,
            label="goal",
        )
        plt.axis("equal")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title(f"{self.scenario}: planned and actual path")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_directory / "trajectory_xy.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.0, 4.8))
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
        plt.savefig(
            output_directory / "position_error.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.0, 4.8))
        plt.plot(times, clearances)
        plt.axhline(
            self.minimum_clearance,
            linestyle="--",
            label="required physical clearance",
        )
        plt.xlabel("time (s)")
        plt.ylabel("point-cloud clearance (m)")
        plt.title(f"{self.scenario}: obstacle clearance")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_directory / "obstacle_clearance.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.0, 4.8))
        for key in [
            "rpm_front_left",
            "rpm_front_right",
            "rpm_rear_right",
            "rpm_rear_left",
        ]:
            plt.plot(
                times,
                [float(sample[key]) for sample in self.samples],
                label=key,
            )
        plt.axhline(
            self.rpm_limit,
            linestyle="--",
            label="RPM limit",
        )
        plt.xlabel("time (s)")
        plt.ylabel("RPM")
        plt.title(f"{self.scenario}: motor commands")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_directory / "motor_rpm.png",
            dpi=180,
        )
        plt.close()

    def save_result(
        self,
        success: bool,
    ) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_directory = (
            self.results_root
            / f"{self.scenario}_{timestamp}"
        )
        output_directory.mkdir(
            parents=True,
            exist_ok=False,
        )

        fieldnames = list(self.samples[0].keys())

        with (output_directory / "flight_data.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=fieldnames,
            )
            writer.writeheader()
            writer.writerows(self.samples)

        metrics = self.metric_values(success)
        (output_directory / "metrics.json").write_text(
            json.dumps(
                metrics,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        lines = [
            "===== World Scenario Mission Summary =====",
            f"Scenario: {metrics['scenario']}",
            f"Goal: {metrics['goal']}",
            f"Success: {metrics['success']}",
            f"Overall pass: {metrics['overall_pass']}",
            (
                "Arrival time: "
                f"{metrics['arrival_time_s']} s"
            ),
            (
                "Final error: "
                f"{metrics['final_error_m']:.4f} m"
            ),
            (
                "Steady-state error: "
                f"{metrics['steady_state_error_m']:.4f} m"
            ),
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
                "Minimum point-cloud clearance: "
                f"{metrics['minimum_pointcloud_clearance_m']:.3f} m"
            ),
            (
                "Safety-grid collision samples: "
                f"{metrics['safety_grid_collision_samples']}"
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

        (output_directory / "summary.txt").write_text(
            "\n".join(lines) + "\n",
            encoding="utf-8",
        )

        self.save_plots(output_directory)
        return output_directory

    def finish(
        self,
        success: bool,
    ) -> None:
        if self.finished:
            return

        self.finished = True

        if not self.samples:
            self.get_logger().error(
                "Mission finished without samples"
            )
            if rclpy.ok():
                rclpy.shutdown()
            return

        output_directory = self.save_result(success)
        metrics = json.loads(
            (output_directory / "metrics.json").read_text(
                encoding="utf-8"
            )
        )

        self.get_logger().info(
            f"Mission result: {output_directory}"
        )
        self.get_logger().info(
            f"Overall pass: {metrics['overall_pass']}"
        )

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioMission()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
