#!/usr/bin/env python3
"""Evaluate dynamic-obstacle prediction, hold, online replan, and recovery."""

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
from nav_msgs.msg import Odometry
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
        for first, second in zip(
            points[:-1],
            points[1:],
        )
    )


class DynamicEvaluation(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_replanning_evaluation")

        self.declare_parameter("goal_x", 9.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("timeout_seconds", 70.0)
        self.declare_parameter("arrival_tolerance_m", 0.25)
        self.declare_parameter("arrival_hold_seconds", 1.0)
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("drone_radius_m", 0.15)
        self.declare_parameter(
            "minimum_dynamic_clearance_m",
            0.30,
        )
        self.declare_parameter(
            "minimum_static_clearance_m",
            0.30,
        )
        self.declare_parameter("rpm_limit", 10000.0)
        self.declare_parameter(
            "results_root",
            str(
                Path.home()
                / "drone_task"
                / "your_drone_sim_ws"
                / "results"
                / "dynamic_replanning_raw"
            ),
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
        self.minimum_dynamic_clearance = float(
            self.get_parameter(
                "minimum_dynamic_clearance_m"
            ).value
        )
        self.minimum_static_clearance = float(
            self.get_parameter(
                "minimum_static_clearance_m"
            ).value
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
        self.static_points: Optional[np.ndarray] = None
        self.voxel_resolution: Optional[float] = None
        self.dynamic_state: Optional[Dict[str, object]] = None
        self.dynamic_map_ready = False
        self.planner_ready = False
        self.planner_path_count = 0
        self.supervisor_status: Optional[Dict[str, object]] = None

        self.samples: List[Dict[str, object]] = []
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
            PointCloud2,
            "/drone/voxel_cloud_raw",
            self.static_cloud_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/voxel_metadata",
            self.metadata_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/dynamic_obstacle/state",
            self.dynamic_state_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/dynamic_map/status",
            self.dynamic_map_status_callback,
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
            "/drone/replanning/status",
            self.supervisor_status_callback,
            transient_qos,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_step,
        )

        self.get_logger().info(
            "Dynamic replanning evaluation ready"
        )

    def metadata_callback(self, message: String) -> None:
        data = json.loads(message.data)
        self.voxel_resolution = float(
            data["resolution_m"]
        )

    def static_cloud_callback(
        self,
        message: PointCloud2,
    ) -> None:
        self.static_points = self.pointcloud_to_xyz(
            message
        )

    def dynamic_state_callback(
        self,
        message: String,
    ) -> None:
        self.dynamic_state = json.loads(message.data)

    def dynamic_map_status_callback(
        self,
        message: String,
    ) -> None:
        data = json.loads(message.data)
        self.dynamic_map_ready = (
            data.get("state") == "ready"
        )

    def planner_status_callback(
        self,
        message: String,
    ) -> None:
        data = json.loads(message.data)
        state = data.get("state")
        if state == "ready":
            self.planner_ready = True
        elif state == "path_ready":
            self.planner_path_count += 1

    def supervisor_status_callback(
        self,
        message: String,
    ) -> None:
        self.supervisor_status = json.loads(
            message.data
        )

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        for index, value in enumerate(
            list(message.data)[:4]
        ):
            self.latest_rpm[index] = float(value)

    def pointcloud_to_xyz(
        self,
        message: PointCloud2,
    ) -> np.ndarray:
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

    def static_clearance(
        self,
        point: Point3,
    ) -> float:
        if (
            self.static_points is None
            or self.voxel_resolution is None
            or self.static_points.size == 0
        ):
            return float("nan")

        position = np.asarray(point, dtype=np.float64)
        squared = np.sum(
            (self.static_points - position) ** 2,
            axis=1,
        )
        center_distance = math.sqrt(
            float(np.min(squared))
        )
        half_diagonal = (
            math.sqrt(3.0)
            * self.voxel_resolution
            / 2.0
        )
        return (
            center_distance
            - half_diagonal
            - self.drone_radius
        )

    def dynamic_clearance(
        self,
        point: Point3,
    ) -> float:
        if self.dynamic_state is None:
            return float("nan")

        center = self.dynamic_state["position"]
        radius = float(
            self.dynamic_state["radius_m"]
        )
        height = float(
            self.dynamic_state["height_m"]
        )

        horizontal = (
            math.hypot(
                point[0] - float(center[0]),
                point[1] - float(center[1]),
            )
            - radius
            - self.drone_radius
        )
        vertical = (
            abs(point[2] - float(center[2]))
            - 0.5 * height
            - self.drone_radius
        )

        if horizontal > 0.0 and vertical > 0.0:
            return math.hypot(horizontal, vertical)
        if horizontal > 0.0:
            return horizontal
        if vertical > 0.0:
            return vertical
        return max(horizontal, vertical)

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
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

        obstacle_position = (
            self.dynamic_state["position"]
            if self.dynamic_state is not None
            else [float("nan")] * 3
        )
        supervisor = (
            self.supervisor_status
            if self.supervisor_status is not None
            else {}
        )

        self.samples.append({
            "time_s": elapsed,
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "vx_mps": float(
                message.twist.twist.linear.x
            ),
            "vy_mps": float(
                message.twist.twist.linear.y
            ),
            "vz_mps": float(
                message.twist.twist.linear.z
            ),
            "position_error_m": error,
            "static_clearance_m": (
                self.static_clearance(position)
            ),
            "dynamic_clearance_m": (
                self.dynamic_clearance(position)
            ),
            "obstacle_x_m": float(
                obstacle_position[0]
            ),
            "obstacle_y_m": float(
                obstacle_position[1]
            ),
            "obstacle_z_m": float(
                obstacle_position[2]
            ),
            "supervisor_state": str(
                supervisor.get("state", "unknown")
            ),
            "replan_count": int(
                supervisor.get("replan_count", 0)
            ),
            "successful_replans": int(
                supervisor.get(
                    "successful_replans",
                    0,
                )
            ),
            "predicted_clearance_m": (
                float(
                    supervisor[
                        "predicted_clearance_m"
                    ]
                )
                if supervisor.get(
                    "predicted_clearance_m"
                ) is not None
                else float("nan")
            ),
            "rpm_1": self.latest_rpm[0],
            "rpm_2": self.latest_rpm[1],
            "rpm_3": self.latest_rpm[2],
            "rpm_4": self.latest_rpm[3],
        })

    def ready(self) -> bool:
        return (
            self.latest_odom is not None
            and self.static_points is not None
            and self.voxel_resolution is not None
            and self.dynamic_state is not None
            and self.dynamic_map_ready
            and self.planner_ready
            and self.supervisor_status is not None
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
            f"Published dynamic mission goal once: "
            f"{self.goal}"
        )

    def control_step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if not self.goal_sent:
            if (
                now - self.node_started
                < self.warmup_seconds
            ):
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

        if (
            now - self.goal_started
            >= self.timeout_seconds
        ):
            self.finish(False, "timeout")

    def compute_metrics(
        self,
        success: bool,
        failure_reason: str,
    ) -> Dict[str, object]:
        positions = [
            (
                float(sample["x_m"]),
                float(sample["y_m"]),
                float(sample["z_m"]),
            )
            for sample in self.samples
        ]
        errors = [
            float(sample["position_error_m"])
            for sample in self.samples
        ]
        static_clearances = [
            float(sample["static_clearance_m"])
            for sample in self.samples
            if math.isfinite(
                float(sample["static_clearance_m"])
            )
        ]
        dynamic_clearances = [
            float(sample["dynamic_clearance_m"])
            for sample in self.samples
            if math.isfinite(
                float(sample["dynamic_clearance_m"])
            )
        ]
        rpm_values = [
            abs(float(sample[key]))
            for sample in self.samples
            for key in (
                "rpm_1",
                "rpm_2",
                "rpm_3",
                "rpm_4",
            )
            if math.isfinite(float(sample[key]))
        ]

        supervisor = self.supervisor_status or {}
        minimum_static = (
            min(static_clearances)
            if static_clearances
            else float("nan")
        )
        minimum_dynamic = (
            min(dynamic_clearances)
            if dynamic_clearances
            else float("nan")
        )
        final_error = (
            errors[-1]
            if errors
            else float("inf")
        )
        maximum_rpm = (
            max(rpm_values)
            if rpm_values
            else float("nan")
        )
        actual_length = polyline_length(positions)

        predicted_collisions = int(
            supervisor.get(
                "predicted_collision_count",
                0,
            )
        )
        hold_count = int(
            supervisor.get("hold_count", 0)
        )
        replan_count = int(
            supervisor.get("replan_count", 0)
        )
        successful_replans = int(
            supervisor.get(
                "successful_replans",
                0,
            )
        )
        failed_replans = int(
            supervisor.get("failed_replans", 0)
        )
        recovery_wait_count = int(
            supervisor.get(
                "recovery_wait_count",
                0,
            )
        )
        replan_latency = supervisor.get(
            "last_replan_latency_ms"
        )

        criteria = {
            "mission_successful": bool(success),
            "dynamic_obstacle_detected": (
                self.dynamic_state is not None
            ),
            "predicted_collision_detected": (
                predicted_collisions >= 1
            ),
            "safety_hold_triggered": (
                hold_count >= 1
            ),
            "online_replan_requested": (
                replan_count >= 1
            ),
            "online_replan_completed": (
                successful_replans >= 1
            ),
            "at_least_two_planner_paths": (
                self.planner_path_count >= 2
            ),
            "final_error_below_0_3_m": (
                final_error < 0.30
            ),
            "dynamic_clearance_above_0_3_m": (
                math.isfinite(minimum_dynamic)
                and minimum_dynamic
                >= self.minimum_dynamic_clearance
            ),
            "static_clearance_above_0_3_m": (
                math.isfinite(minimum_static)
                and minimum_static
                >= self.minimum_static_clearance
            ),
            "no_motor_saturation": (
                math.isfinite(maximum_rpm)
                and maximum_rpm < self.rpm_limit
            ),
        }

        return {
            "scenario": "dynamic_crossing_3d",
            "goal": list(self.goal),
            "success": bool(success),
            "failure_reason": failure_reason,
            "overall_pass": all(criteria.values()),
            "criteria": criteria,
            "arrival_time_s": self.arrival_time,
            "final_error_m": final_error,
            "actual_path_length_m": actual_length,
            "minimum_static_clearance_m": (
                minimum_static
            ),
            "minimum_dynamic_clearance_m": (
                minimum_dynamic
            ),
            "maximum_motor_rpm": maximum_rpm,
            "predicted_collision_count": (
                predicted_collisions
            ),
            "hold_count": hold_count,
            "replan_count": replan_count,
            "successful_replans": successful_replans,
            "failed_replans": failed_replans,
            "recovery_wait_count": recovery_wait_count,
            "planner_path_count": self.planner_path_count,
            "last_replan_latency_ms": replan_latency,
            "supervisor_status": supervisor,
            "sample_count": len(self.samples),
        }

    def save_plots(self, output: Path) -> None:
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
        obstacle_x = [
            float(sample["obstacle_x_m"])
            for sample in self.samples
        ]
        obstacle_y = [
            float(sample["obstacle_y_m"])
            for sample in self.samples
        ]

        plt.figure(figsize=(8.0, 6.3))
        plt.plot(
            x_values,
            y_values,
            label="drone trajectory",
        )
        plt.plot(
            obstacle_x,
            obstacle_y,
            linestyle="--",
            label="moving obstacle trajectory",
        )
        plt.scatter(
            [self.goal[0]],
            [self.goal[1]],
            marker="*",
            s=150,
            label="goal",
        )
        plt.axis("equal")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title(
            "Dynamic obstacle avoidance and online replanning"
        )
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output / "dynamic_trajectory_xy.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.2, 5.0))
        plt.plot(
            times,
            [
                float(sample["dynamic_clearance_m"])
                for sample in self.samples
            ],
            label="dynamic clearance",
        )
        plt.plot(
            times,
            [
                float(sample["static_clearance_m"])
                for sample in self.samples
            ],
            label="static clearance",
        )
        plt.axhline(
            self.minimum_dynamic_clearance,
            linestyle="--",
            label="required clearance",
        )
        plt.xlabel("time (s)")
        plt.ylabel("body clearance (m)")
        plt.title("Static and dynamic obstacle clearance")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output / "clearance_profile.png",
            dpi=180,
        )
        plt.close()

        state_names = sorted({
            str(sample["supervisor_state"])
            for sample in self.samples
        })
        state_to_value = {
            state: index
            for index, state in enumerate(state_names)
        }
        state_values = [
            state_to_value[
                str(sample["supervisor_state"])
            ]
            for sample in self.samples
        ]

        plt.figure(figsize=(8.2, 5.0))
        plt.step(
            times,
            state_values,
            where="post",
        )
        plt.yticks(
            list(state_to_value.values()),
            list(state_to_value.keys()),
        )
        plt.xlabel("time (s)")
        plt.ylabel("supervisor state")
        plt.title("Hold, replan, and resume state sequence")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(
            output / "replanning_state_sequence.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.2, 5.0))
        plt.plot(
            times,
            [
                float(sample["position_error_m"])
                for sample in self.samples
            ],
        )
        plt.axhline(
            self.arrival_tolerance,
            linestyle="--",
            label="arrival tolerance",
        )
        plt.xlabel("time (s)")
        plt.ylabel("position error (m)")
        plt.title("Dynamic mission position error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output / "position_error.png",
            dpi=180,
        )
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
            / f"dynamic_replanning_{timestamp}"
        )
        output.mkdir(
            parents=True,
            exist_ok=False,
        )

        with (output / "flight_data.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as handle:
            fieldnames = list(
                self.samples[0].keys()
            )
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
            "===== Dynamic Replanning Mission Summary =====",
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
                "Actual path length: "
                f"{metrics['actual_path_length_m']:.3f} m"
            ),
            (
                "Minimum static clearance: "
                f"{metrics['minimum_static_clearance_m']:.3f} m"
            ),
            (
                "Minimum dynamic clearance: "
                f"{metrics['minimum_dynamic_clearance_m']:.3f} m"
            ),
            (
                "Predicted collision count: "
                f"{metrics['predicted_collision_count']}"
            ),
            f"Safety hold count: {metrics['hold_count']}",
            f"Replan requests: {metrics['replan_count']}",
            (
                "Successful replans: "
                f"{metrics['successful_replans']}"
            ),
            (
                "Temporary failed replans: "
                f"{metrics['failed_replans']}"
            ),
            (
                "Recovery wait count: "
                f"{metrics['recovery_wait_count']}"
            ),
            (
                "Planner path count: "
                f"{metrics['planner_path_count']}"
            ),
            (
                "Last replan latency: "
                f"{metrics['last_replan_latency_ms']} ms"
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
            f"Dynamic replanning result: {output}"
        )
        self.get_logger().info(
            f"Overall pass: {metrics['overall_pass']}"
        )

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicEvaluation()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
