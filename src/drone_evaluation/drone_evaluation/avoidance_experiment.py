#!/usr/bin/env python3

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

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray


Vector3 = Tuple[float, float, float]
Obstacle = Dict[str, float]


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Vector3:
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(
        sin_roll_cos_pitch,
        cos_roll_cos_pitch,
    )

    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(
        sin_yaw_cos_pitch,
        cos_yaw_cos_pitch,
    )

    return roll, pitch, yaw


def euclidean_distance(
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
        euclidean_distance(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def signed_distance_to_aabb(
    point: Vector3,
    obstacle: Obstacle,
) -> float:
    half_x = 0.5 * obstacle["size_x"]
    half_y = 0.5 * obstacle["size_y"]
    half_z = 0.5 * obstacle["size_z"]

    local_x = abs(point[0] - obstacle["center_x"])
    local_y = abs(point[1] - obstacle["center_y"])
    local_z = abs(point[2] - obstacle["center_z"])

    delta_x = local_x - half_x
    delta_y = local_y - half_y
    delta_z = local_z - half_z

    outside_distance = math.sqrt(
        max(delta_x, 0.0) ** 2
        + max(delta_y, 0.0) ** 2
        + max(delta_z, 0.0) ** 2
    )

    if delta_x <= 0.0 and delta_y <= 0.0 and delta_z <= 0.0:
        penetration = min(
            half_x - local_x,
            half_y - local_y,
            half_z - local_z,
        )
        return -penetration

    return outside_distance


class AvoidanceExperiment(Node):
    def __init__(self) -> None:
        super().__init__("avoidance_experiment")

        self.declare_parameter("goal_x", 6.5)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("warmup_seconds", 3.0)
        self.declare_parameter("arrival_tolerance_m", 0.20)
        self.declare_parameter("arrival_hold_seconds", 1.0)
        self.declare_parameter("settle_seconds", 2.0)
        self.declare_parameter("experiment_timeout_seconds", 40.0)
        self.declare_parameter("command_publish_hz", 5.0)
        self.declare_parameter("command_publish_seconds", 1.2)
        self.declare_parameter("drone_radius_m", 0.15)
        self.declare_parameter("rpm_limit", 10000.0)
        self.declare_parameter(
            "results_root",
            str(
                Path.home()
                / "drone_task"
                / "your_drone_sim_ws"
                / "results"
            ),
        )

        self.goal: Vector3 = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        self.warmup_seconds = float(
            self.get_parameter("warmup_seconds").value
        )
        self.arrival_tolerance_m = float(
            self.get_parameter("arrival_tolerance_m").value
        )
        self.arrival_hold_seconds = float(
            self.get_parameter("arrival_hold_seconds").value
        )
        self.settle_seconds = float(
            self.get_parameter("settle_seconds").value
        )
        self.experiment_timeout_seconds = float(
            self.get_parameter("experiment_timeout_seconds").value
        )
        self.command_publish_hz = float(
            self.get_parameter("command_publish_hz").value
        )
        self.command_publish_seconds = float(
            self.get_parameter("command_publish_seconds").value
        )
        self.drone_radius_m = float(
            self.get_parameter("drone_radius_m").value
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
        self.planned_points: List[Vector3] = []
        self.obstacles: List[Obstacle] = []
        self.safety_zones: List[Obstacle] = []

        self.samples: List[Dict[str, float]] = []
        self.goal_publish_count = 0
        self.goal_published = False
        self.plan_received = False
        self.completed = False
        self.finished = False

        self.node_start_monotonic = time.monotonic()
        self.goal_start_monotonic: Optional[float] = None
        self.arrival_hold_start: Optional[float] = None
        self.arrival_monotonic: Optional[float] = None
        self.finish_requested_monotonic: Optional[float] = None

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            20,
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
            self.planned_path_callback,
            10,
        )
        self.create_subscription(
            MarkerArray,
            "/drone/obstacle_markers",
            self.obstacle_callback,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.command_publish_hz,
            self.control_step,
        )

        self.get_logger().info(
            "Avoidance experiment node started"
        )
        self.get_logger().info(
            "Goal: "
            f"({self.goal[0]:.2f}, "
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

    def planned_path_callback(
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

        if not self.plan_received:
            self.plan_received = True
            self.get_logger().info(
                "Planned path received: "
                f"{len(self.planned_points)} waypoints, "
                f"{polyline_length(self.planned_points):.3f} m"
            )

    def obstacle_callback(
        self,
        message: MarkerArray,
    ) -> None:
        obstacles: List[Obstacle] = []
        safety_zones: List[Obstacle] = []

        for marker in message.markers:
            if marker.type != marker.CUBE:
                continue

            record: Obstacle = {
                "center_x": float(marker.pose.position.x),
                "center_y": float(marker.pose.position.y),
                "center_z": float(marker.pose.position.z),
                "size_x": float(marker.scale.x),
                "size_y": float(marker.scale.y),
                "size_z": float(marker.scale.z),
            }

            if marker.ns == "obstacles":
                obstacles.append(record)
            elif marker.ns == "safety_zones":
                safety_zones.append(record)

        if obstacles:
            self.obstacles = obstacles

        if safety_zones:
            self.safety_zones = safety_zones

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.latest_odom = message

        if self.goal_start_monotonic is None or self.finished:
            return

        now = time.monotonic()
        elapsed = now - self.goal_start_monotonic

        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        velocity = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
            float(message.twist.twist.linear.z),
        )

        orientation = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

        error = euclidean_distance(position, self.goal)

        obstacle_clearance = float("nan")
        safety_clearance = float("nan")

        if self.obstacles:
            obstacle_clearance = min(
                signed_distance_to_aabb(position, obstacle)
                - self.drone_radius_m
                for obstacle in self.obstacles
            )

        if self.safety_zones:
            safety_clearance = min(
                signed_distance_to_aabb(position, obstacle)
                for obstacle in self.safety_zones
            )

        sample: Dict[str, float] = {
            "time_s": elapsed,
            "x_m": position[0],
            "y_m": position[1],
            "z_m": position[2],
            "vx_mps": velocity[0],
            "vy_mps": velocity[1],
            "vz_mps": velocity[2],
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "goal_x_m": self.goal[0],
            "goal_y_m": self.goal[1],
            "goal_z_m": self.goal[2],
            "position_error_m": error,
            "obstacle_clearance_m": obstacle_clearance,
            "safety_zone_clearance_m": safety_clearance,
            "rpm_front_left": self.latest_rpm[0],
            "rpm_front_right": self.latest_rpm[1],
            "rpm_rear_right": self.latest_rpm[2],
            "rpm_rear_left": self.latest_rpm[3],
        }
        self.samples.append(sample)

        if error <= self.arrival_tolerance_m:
            if self.arrival_hold_start is None:
                self.arrival_hold_start = now

            if (
                self.arrival_monotonic is None
                and now - self.arrival_hold_start
                >= self.arrival_hold_seconds
            ):
                self.arrival_monotonic = now
                self.finish_requested_monotonic = now
                self.get_logger().info(
                    "Arrival condition satisfied; "
                    "collecting settling data"
                )
        else:
            self.arrival_hold_start = None

    def publish_goal(self) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.position.z = self.goal[2]
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.goal_publish_count += 1

    def control_step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if self.latest_odom is None:
            return

        if not self.obstacles:
            return

        if (
            not self.goal_published
            and now - self.node_start_monotonic >= self.warmup_seconds
        ):
            self.goal_published = True
            self.goal_start_monotonic = now
            self.get_logger().info(
                "Publishing avoidance experiment goal"
            )

        if self.goal_published:
            maximum_publications = max(
                1,
                int(
                    self.command_publish_seconds
                    * self.command_publish_hz
                ),
            )

            if self.goal_publish_count < maximum_publications:
                self.publish_goal()

        if self.goal_start_monotonic is None:
            return

        elapsed = now - self.goal_start_monotonic

        if (
            self.finish_requested_monotonic is not None
            and now - self.finish_requested_monotonic
            >= self.settle_seconds
        ):
            self.finish_experiment(
                success=True,
                reason="goal_reached",
            )
            return

        if elapsed >= self.experiment_timeout_seconds:
            self.finish_experiment(
                success=False,
                reason="timeout",
            )

    def finish_experiment(
        self,
        success: bool,
        reason: str,
    ) -> None:
        if self.finished:
            return

        self.finished = True
        self.completed = success

        try:
            output_dir = self.write_results(
                success=success,
                reason=reason,
            )
            self.get_logger().info(
                f"Results directory: {output_dir}"
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to write results: {error}"
            )

        self.get_logger().info(
            "Avoidance experiment complete; "
            "shutting down launch"
        )

        self.create_timer(
            0.5,
            self.request_shutdown,
        )

    @staticmethod
    def finite_values(
        samples: Sequence[Dict[str, float]],
        key: str,
    ) -> List[float]:
        values = []

        for sample in samples:
            value = float(sample[key])
            if math.isfinite(value):
                values.append(value)

        return values

    def calculate_metrics(
        self,
        success: bool,
        reason: str,
    ) -> Dict[str, object]:
        actual_points = [
            (
                sample["x_m"],
                sample["y_m"],
                sample["z_m"],
            )
            for sample in self.samples
        ]

        actual_path_length = polyline_length(actual_points)
        planned_path_length = polyline_length(self.planned_points)

        errors = self.finite_values(
            self.samples,
            "position_error_m",
        )
        obstacle_clearances = self.finite_values(
            self.samples,
            "obstacle_clearance_m",
        )
        safety_clearances = self.finite_values(
            self.samples,
            "safety_zone_clearance_m",
        )

        rpm_keys = [
            "rpm_front_left",
            "rpm_front_right",
            "rpm_rear_right",
            "rpm_rear_left",
        ]
        all_rpm_values: List[float] = []

        for key in rpm_keys:
            all_rpm_values.extend(
                self.finite_values(self.samples, key)
            )

        arrival_time = None

        if (
            self.arrival_monotonic is not None
            and self.goal_start_monotonic is not None
        ):
            arrival_time = (
                self.arrival_monotonic
                - self.goal_start_monotonic
            )

        steady_state_errors = []

        if self.samples:
            final_time = self.samples[-1]["time_s"]
            steady_state_errors = [
                sample["position_error_m"]
                for sample in self.samples
                if sample["time_s"] >= final_time - 1.0
            ]

        final_error = errors[-1] if errors else None
        steady_state_error = (
            sum(steady_state_errors) / len(steady_state_errors)
            if steady_state_errors
            else None
        )

        maximum_rpm = max(all_rpm_values) if all_rpm_values else None
        saturation_ratio = None

        if all_rpm_values:
            saturation_count = sum(
                1
                for value in all_rpm_values
                if value >= 0.99 * self.rpm_limit
            )
            saturation_ratio = saturation_count / len(all_rpm_values)

        max_roll = max(
            (
                abs(sample["roll_deg"])
                for sample in self.samples
            ),
            default=None,
        )
        max_pitch = max(
            (
                abs(sample["pitch_deg"])
                for sample in self.samples
            ),
            default=None,
        )

        path_ratio = None
        if planned_path_length > 1.0e-9:
            path_ratio = actual_path_length / planned_path_length

        return {
            "success": success,
            "termination_reason": reason,
            "goal_m": {
                "x": self.goal[0],
                "y": self.goal[1],
                "z": self.goal[2],
            },
            "samples": len(self.samples),
            "planned_waypoints": len(self.planned_points),
            "arrival_time_s": arrival_time,
            "final_error_m": final_error,
            "steady_state_error_m": steady_state_error,
            "maximum_position_error_m": max(errors) if errors else None,
            "z_overshoot_m": (
                max(
                    0.0,
                    max(
                        (
                            sample["z_m"]
                            for sample in self.samples
                        ),
                        default=self.goal[2],
                    )
                    - self.goal[2],
                )
            ),
            "planned_path_length_m": planned_path_length,
            "actual_path_length_m": actual_path_length,
            "actual_over_planned_path_ratio": path_ratio,
            "minimum_obstacle_clearance_m": (
                min(obstacle_clearances)
                if obstacle_clearances
                else None
            ),
            "minimum_safety_zone_clearance_m": (
                min(safety_clearances)
                if safety_clearances
                else None
            ),
            "maximum_absolute_roll_deg": max_roll,
            "maximum_absolute_pitch_deg": max_pitch,
            "maximum_motor_rpm": maximum_rpm,
            "rpm_saturation_ratio": saturation_ratio,
            "rpm_limit": self.rpm_limit,
        }

    def write_results(
        self,
        success: bool,
        reason: str,
    ) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = (
            self.results_root
            / f"avoidance_{timestamp}"
        )
        output_dir.mkdir(
            parents=True,
            exist_ok=False,
        )

        csv_path = output_dir / "flight_data.csv"

        if self.samples:
            fieldnames = list(self.samples[0].keys())

            with csv_path.open(
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

        metrics = self.calculate_metrics(
            success=success,
            reason=reason,
        )

        with (output_dir / "metrics.json").open(
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(
                metrics,
                handle,
                ensure_ascii=False,
                indent=2,
            )

        summary_text = self.format_summary(metrics)
        (output_dir / "summary.txt").write_text(
            summary_text,
            encoding="utf-8",
        )

        self.create_plots(output_dir)

        print()
        print(summary_text)
        return output_dir

    @staticmethod
    def metric_text(
        value: object,
        unit: str = "",
        digits: int = 4,
    ) -> str:
        if value is None:
            return "N/A"

        if isinstance(value, bool):
            return str(value)

        if isinstance(value, (int, float)):
            if not math.isfinite(float(value)):
                return "N/A"
            return f"{float(value):.{digits}f}{unit}"

        return str(value)

    def format_summary(
        self,
        metrics: Dict[str, object],
    ) -> str:
        lines = [
            "===== A* Avoidance Experiment Summary =====",
            f"Success: {metrics['success']}",
            f"Termination: {metrics['termination_reason']}",
            (
                "Arrival time: "
                + self.metric_text(
                    metrics["arrival_time_s"],
                    " s",
                    3,
                )
            ),
            (
                "Final error: "
                + self.metric_text(
                    metrics["final_error_m"],
                    " m",
                )
            ),
            (
                "Steady-state error: "
                + self.metric_text(
                    metrics["steady_state_error_m"],
                    " m",
                )
            ),
            (
                "Planned path length: "
                + self.metric_text(
                    metrics["planned_path_length_m"],
                    " m",
                    3,
                )
            ),
            (
                "Actual path length: "
                + self.metric_text(
                    metrics["actual_path_length_m"],
                    " m",
                    3,
                )
            ),
            (
                "Actual/planned path ratio: "
                + self.metric_text(
                    metrics["actual_over_planned_path_ratio"],
                    "",
                    3,
                )
            ),
            (
                "Minimum obstacle clearance: "
                + self.metric_text(
                    metrics["minimum_obstacle_clearance_m"],
                    " m",
                    3,
                )
            ),
            (
                "Minimum safety-zone clearance: "
                + self.metric_text(
                    metrics["minimum_safety_zone_clearance_m"],
                    " m",
                    3,
                )
            ),
            (
                "Maximum |roll|: "
                + self.metric_text(
                    metrics["maximum_absolute_roll_deg"],
                    " deg",
                    2,
                )
            ),
            (
                "Maximum |pitch|: "
                + self.metric_text(
                    metrics["maximum_absolute_pitch_deg"],
                    " deg",
                    2,
                )
            ),
            (
                "Maximum motor RPM: "
                + self.metric_text(
                    metrics["maximum_motor_rpm"],
                    " RPM",
                    1,
                )
            ),
            (
                "RPM saturation ratio: "
                + self.metric_text(
                    (
                        None
                        if metrics["rpm_saturation_ratio"] is None
                        else 100.0
                        * float(metrics["rpm_saturation_ratio"])
                    ),
                    " %",
                    3,
                )
            ),
        ]

        return "\n".join(lines)

    def create_plots(
        self,
        output_dir: Path,
    ) -> None:
        if not self.samples:
            return

        times = [
            sample["time_s"]
            for sample in self.samples
        ]
        actual_x = [
            sample["x_m"]
            for sample in self.samples
        ]
        actual_y = [
            sample["y_m"]
            for sample in self.samples
        ]
        actual_z = [
            sample["z_m"]
            for sample in self.samples
        ]

        plt.figure(figsize=(9, 7))
        plt.plot(actual_x, actual_y, label="actual trajectory")

        if self.planned_points:
            plt.plot(
                [point[0] for point in self.planned_points],
                [point[1] for point in self.planned_points],
                marker="o",
                label="planned path",
            )

        for obstacle in self.obstacles:
            half_x = 0.5 * obstacle["size_x"]
            half_y = 0.5 * obstacle["size_y"]
            corners_x = [
                obstacle["center_x"] - half_x,
                obstacle["center_x"] + half_x,
                obstacle["center_x"] + half_x,
                obstacle["center_x"] - half_x,
                obstacle["center_x"] - half_x,
            ]
            corners_y = [
                obstacle["center_y"] - half_y,
                obstacle["center_y"] - half_y,
                obstacle["center_y"] + half_y,
                obstacle["center_y"] + half_y,
                obstacle["center_y"] - half_y,
            ]
            plt.plot(corners_x, corners_y)

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Planned and Actual Avoidance Trajectories")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "trajectory_xy.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(10, 5.5))
        plt.plot(
            times,
            [
                sample["position_error_m"]
                for sample in self.samples
            ],
            label="position error",
        )
        plt.axhline(
            self.arrival_tolerance_m,
            linestyle="--",
            label="arrival tolerance",
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Position error (m)")
        plt.title("Avoidance Goal Tracking Error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "tracking_error.png",
            dpi=180,
        )
        plt.close()

        obstacle_values = self.finite_values(
            self.samples,
            "obstacle_clearance_m",
        )

        if len(obstacle_values) == len(self.samples):
            plt.figure(figsize=(10, 5.5))
            plt.plot(
                times,
                obstacle_values,
                label="obstacle clearance",
            )
            plt.axhline(
                0.0,
                linestyle="--",
                label="collision boundary",
            )
            plt.xlabel("Time (s)")
            plt.ylabel("Clearance (m)")
            plt.title("Minimum Physical Obstacle Clearance")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(
                output_dir / "obstacle_clearance.png",
                dpi=180,
            )
            plt.close()

        rpm_series = [
            (
                "front left",
                [
                    sample["rpm_front_left"]
                    for sample in self.samples
                ],
            ),
            (
                "front right",
                [
                    sample["rpm_front_right"]
                    for sample in self.samples
                ],
            ),
            (
                "rear right",
                [
                    sample["rpm_rear_right"]
                    for sample in self.samples
                ],
            ),
            (
                "rear left",
                [
                    sample["rpm_rear_left"]
                    for sample in self.samples
                ],
            ),
        ]

        if any(
            any(math.isfinite(float(value)) for value in values)
            for _, values in rpm_series
        ):
            plt.figure(figsize=(10, 5.5))

            for label, values in rpm_series:
                plt.plot(
                    times,
                    values,
                    label=label,
                )

            plt.xlabel("Time (s)")
            plt.ylabel("Motor speed (RPM)")
            plt.title("Avoidance Motor RPM")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(
                output_dir / "motor_rpm.png",
                dpi=180,
            )
            plt.close()

        plt.figure(figsize=(10, 5.5))
        plt.plot(times, actual_x, label="x")
        plt.plot(times, actual_y, label="y")
        plt.plot(times, actual_z, label="z")
        plt.axhline(
            self.goal[0],
            linestyle="--",
            label="goal x",
        )
        plt.axhline(
            self.goal[1],
            linestyle="--",
            label="goal y",
        )
        plt.axhline(
            self.goal[2],
            linestyle="--",
            label="goal z",
        )
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Avoidance Position Response")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "position_response.png",
            dpi=180,
        )
        plt.close()

    def request_shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceExperiment()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
