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
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray


Vector3 = Tuple[float, float, float]
Obstacle = Dict[str, float]


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


def signed_distance_to_aabb(
    point: Vector3,
    obstacle: Obstacle,
) -> float:
    half_x = 0.5 * obstacle["size_x"]
    half_y = 0.5 * obstacle["size_y"]
    half_z = 0.5 * obstacle["size_z"]

    dx = abs(point[0] - obstacle["center_x"]) - half_x
    dy = abs(point[1] - obstacle["center_y"]) - half_y
    dz = abs(point[2] - obstacle["center_z"]) - half_z

    outside = math.sqrt(
        max(dx, 0.0) ** 2
        + max(dy, 0.0) ** 2
        + max(dz, 0.0) ** 2
    )

    if dx <= 0.0 and dy <= 0.0 and dz <= 0.0:
        penetration = min(-dx, -dy, -dz)
        return -penetration

    return outside


class MultiGoalEvaluation(Node):
    def __init__(self) -> None:
        super().__init__("multi_goal_evaluation")

        self.declare_parameter(
            "goal_xs",
            [1.5, 6.5, 1.5, 0.0],
        )
        self.declare_parameter(
            "goal_ys",
            [-2.5, 0.0, 2.5, 0.0],
        )
        self.declare_parameter(
            "goal_zs",
            [1.5, 1.5, 1.5, 1.5],
        )
        self.declare_parameter("warmup_seconds", 3.0)
        self.declare_parameter("arrival_tolerance_m", 0.22)
        self.declare_parameter("arrival_hold_seconds", 0.8)
        self.declare_parameter("inter_goal_pause_seconds", 1.0)
        self.declare_parameter("per_goal_timeout_seconds", 40.0)
        self.declare_parameter("final_settle_seconds", 2.0)
        self.declare_parameter("drone_radius_m", 0.15)
        self.declare_parameter("rpm_limit", 10000.0)
        self.declare_parameter(
            "results_root",
            str(
                Path.home()
                / "drone_task"
                / "your_drone_sim_ws"
                / "results"
                / "multigoal"
            ),
        )

        goal_xs = list(self.get_parameter("goal_xs").value)
        goal_ys = list(self.get_parameter("goal_ys").value)
        goal_zs = list(self.get_parameter("goal_zs").value)

        if not (
            len(goal_xs)
            == len(goal_ys)
            == len(goal_zs)
            >= 2
        ):
            raise RuntimeError(
                "goal_xs, goal_ys and goal_zs must have "
                "the same length and contain at least 2 goals"
            )

        self.goals: List[Vector3] = [
            (
                float(goal_xs[index]),
                float(goal_ys[index]),
                float(goal_zs[index]),
            )
            for index in range(len(goal_xs))
        ]

        self.warmup_seconds = float(
            self.get_parameter("warmup_seconds").value
        )
        self.arrival_tolerance_m = float(
            self.get_parameter("arrival_tolerance_m").value
        )
        self.arrival_hold_seconds = float(
            self.get_parameter("arrival_hold_seconds").value
        )
        self.inter_goal_pause_seconds = float(
            self.get_parameter(
                "inter_goal_pause_seconds"
            ).value
        )
        self.per_goal_timeout_seconds = float(
            self.get_parameter(
                "per_goal_timeout_seconds"
            ).value
        )
        self.final_settle_seconds = float(
            self.get_parameter("final_settle_seconds").value
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
        self.latest_position: Optional[Vector3] = None
        self.latest_rpm = [float("nan")] * 4
        self.occupancy_grid: Optional[OccupancyGrid] = None
        self.obstacles: List[Obstacle] = []
        self.safety_zones: List[Obstacle] = []

        self.samples: List[Dict[str, float]] = []
        self.segments: List[Dict[str, object]] = []

        self.current_goal_index = -1
        self.current_goal: Optional[Vector3] = None
        self.current_goal_start_time: Optional[float] = None
        self.current_goal_start_sample = 0
        self.current_goal_start_position: Optional[Vector3] = None
        self.current_plan: List[Vector3] = []
        self.plan_pending = False
        self.segment_active = False

        self.arrival_hold_start: Optional[float] = None
        self.pending_next_goal_time: Optional[float] = None
        self.final_arrival_time: Optional[float] = None

        self.node_start_time = time.monotonic()
        self.mission_start_time: Optional[float] = None
        self.finished = False

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
        self.create_subscription(
            OccupancyGrid,
            "/drone/obstacle_grid",
            self.grid_callback,
            10,
        )

        self.create_timer(0.10, self.control_step)

        self.get_logger().info(
            "Multi-goal avoidance evaluation started"
        )
        self.get_logger().info(
            f"Configured goals: {self.goals}"
        )

    def grid_callback(
        self,
        message: OccupancyGrid,
    ) -> None:
        self.occupancy_grid = message

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        values = list(message.data)

        for index in range(min(4, len(values))):
            self.latest_rpm[index] = float(values[index])

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

    def planned_path_callback(
        self,
        message: NavPath,
    ) -> None:
        if (
            not self.plan_pending
            or self.current_goal is None
            or len(message.poses) < 2
        ):
            return

        points: List[Vector3] = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

        if distance(points[-1], self.current_goal) > 0.45:
            return

        self.current_plan = points
        self.plan_pending = False

        self.get_logger().info(
            f"Goal {self.current_goal_index + 1}: "
            f"planned path received, "
            f"{len(points)} waypoints, "
            f"{polyline_length(points):.3f} m"
        )

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.latest_odom = message
        self.latest_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )

        if (
            self.finished
            or not self.segment_active
            or self.current_goal is None
            or self.mission_start_time is None
            or self.current_goal_start_time is None
        ):
            return

        now = time.monotonic()
        mission_elapsed = now - self.mission_start_time
        segment_elapsed = now - self.current_goal_start_time
        error = distance(
            self.latest_position,
            self.current_goal,
        )

        obstacle_clearance = float("nan")
        safety_clearance = float("nan")

        if self.obstacles:
            obstacle_clearance = min(
                signed_distance_to_aabb(
                    self.latest_position,
                    obstacle,
                )
                - self.drone_radius_m
                for obstacle in self.obstacles
            )

        if self.safety_zones:
            safety_clearance = min(
                signed_distance_to_aabb(
                    self.latest_position,
                    obstacle,
                )
                for obstacle in self.safety_zones
            )

        self.samples.append({
            "mission_time_s": mission_elapsed,
            "segment_time_s": segment_elapsed,
            "goal_index": float(
                self.current_goal_index + 1
            ),
            "goal_x_m": self.current_goal[0],
            "goal_y_m": self.current_goal[1],
            "goal_z_m": self.current_goal[2],
            "x_m": self.latest_position[0],
            "y_m": self.latest_position[1],
            "z_m": self.latest_position[2],
            "position_error_m": error,
            "obstacle_clearance_m": obstacle_clearance,
            "safety_zone_clearance_m": safety_clearance,
            "rpm_front_left": self.latest_rpm[0],
            "rpm_front_right": self.latest_rpm[1],
            "rpm_rear_right": self.latest_rpm[2],
            "rpm_rear_left": self.latest_rpm[3],
        })

        if error <= self.arrival_tolerance_m:
            if self.arrival_hold_start is None:
                self.arrival_hold_start = now

            if (
                now - self.arrival_hold_start
                >= self.arrival_hold_seconds
            ):
                self.complete_current_goal(now)
        else:
            self.arrival_hold_start = None

    def goal_is_free(
        self,
        goal: Vector3,
    ) -> bool:
        if self.occupancy_grid is None:
            return False

        grid = self.occupancy_grid
        resolution = float(grid.info.resolution)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)

        cell_x = int(
            math.floor((goal[0] - origin_x) / resolution)
        )
        cell_y = int(
            math.floor((goal[1] - origin_y) / resolution)
        )

        if (
            cell_x < 0
            or cell_y < 0
            or cell_x >= int(grid.info.width)
            or cell_y >= int(grid.info.height)
        ):
            return False

        index = (
            cell_y * int(grid.info.width)
            + cell_x
        )
        value = int(grid.data[index])

        return value >= 0 and value < 50

    def publish_goal(
        self,
        goal: Vector3,
    ) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = goal[0]
        message.pose.position.y = goal[1]
        message.pose.position.z = goal[2]
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def start_goal(
        self,
        goal_index: int,
    ) -> None:
        if self.latest_position is None:
            return

        self.current_goal_index = goal_index
        self.current_goal = self.goals[goal_index]
        self.current_goal_start_time = time.monotonic()
        self.current_goal_start_sample = len(self.samples)
        self.current_goal_start_position = (
            self.latest_position
        )
        self.current_plan = []
        self.plan_pending = True
        self.segment_active = True
        self.arrival_hold_start = None
        self.pending_next_goal_time = None

        self.publish_goal(self.current_goal)

        self.get_logger().info(
            f"Publishing goal {goal_index + 1}/"
            f"{len(self.goals)}: "
            f"({self.current_goal[0]:.2f}, "
            f"{self.current_goal[1]:.2f}, "
            f"{self.current_goal[2]:.2f})"
        )

    def complete_current_goal(
        self,
        now: float,
    ) -> None:
        if (
            not self.segment_active
            or self.current_goal is None
            or self.current_goal_start_time is None
        ):
            return

        segment_samples = self.samples[
            self.current_goal_start_sample:
        ]
        actual_points = [
            (
                sample["x_m"],
                sample["y_m"],
                sample["z_m"],
            )
            for sample in segment_samples
        ]
        errors = [
            sample["position_error_m"]
            for sample in segment_samples
        ]
        obstacle_clearances = [
            sample["obstacle_clearance_m"]
            for sample in segment_samples
            if math.isfinite(
                sample["obstacle_clearance_m"]
            )
        ]
        safety_clearances = [
            sample["safety_zone_clearance_m"]
            for sample in segment_samples
            if math.isfinite(
                sample["safety_zone_clearance_m"]
            )
        ]
        rpm_values = []

        for sample in segment_samples:
            for key in [
                "rpm_front_left",
                "rpm_front_right",
                "rpm_rear_right",
                "rpm_rear_left",
            ]:
                value = sample[key]
                if math.isfinite(value):
                    rpm_values.append(value)

        segment = {
            "goal_index": self.current_goal_index + 1,
            "goal": {
                "x": self.current_goal[0],
                "y": self.current_goal[1],
                "z": self.current_goal[2],
            },
            "arrival_time_s": (
                now - self.current_goal_start_time
            ),
            "final_error_m": errors[-1] if errors else None,
            "maximum_error_m": max(errors) if errors else None,
            "planned_path_length_m": polyline_length(
                self.current_plan
            ),
            "actual_path_length_m": polyline_length(
                actual_points
            ),
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
            "maximum_motor_rpm": (
                max(rpm_values)
                if rpm_values
                else None
            ),
            "planned_points": [
                list(point)
                for point in self.current_plan
            ],
        }
        self.segments.append(segment)

        self.segment_active = False
        self.arrival_hold_start = None

        self.get_logger().info(
            f"Goal {self.current_goal_index + 1} reached: "
            f"time={segment['arrival_time_s']:.3f} s, "
            f"error={segment['final_error_m']:.4f} m"
        )

        if self.current_goal_index + 1 >= len(self.goals):
            self.final_arrival_time = now
        else:
            self.pending_next_goal_time = (
                now + self.inter_goal_pause_seconds
            )

    def start_mission(self) -> None:
        invalid_goals = [
            (index + 1, goal)
            for index, goal in enumerate(self.goals)
            if not self.goal_is_free(goal)
        ]

        if invalid_goals:
            self.get_logger().error(
                f"Blocked or out-of-map goals: {invalid_goals}"
            )
            self.finish_mission(
                success=False,
                reason="invalid_goal",
            )
            return

        self.mission_start_time = time.monotonic()
        self.get_logger().info(
            "All goals passed occupancy-grid validation"
        )
        self.start_goal(0)

    def control_step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if (
            self.mission_start_time is None
            and self.latest_position is not None
            and self.occupancy_grid is not None
            and self.obstacles
            and now - self.node_start_time
            >= self.warmup_seconds
        ):
            self.start_mission()
            return

        if self.segment_active:
            if (
                self.current_goal_start_time is not None
                and now - self.current_goal_start_time
                >= self.per_goal_timeout_seconds
            ):
                self.finish_mission(
                    success=False,
                    reason=(
                        f"goal_{self.current_goal_index + 1}"
                        "_timeout"
                    ),
                )
            return

        if (
            self.pending_next_goal_time is not None
            and now >= self.pending_next_goal_time
        ):
            self.start_goal(
                self.current_goal_index + 1
            )
            return

        if (
            self.final_arrival_time is not None
            and now - self.final_arrival_time
            >= self.final_settle_seconds
        ):
            self.finish_mission(
                success=True,
                reason="all_goals_reached",
            )

    def calculate_metrics(
        self,
        success: bool,
        reason: str,
    ) -> Dict[str, object]:
        total_planned = sum(
            float(segment["planned_path_length_m"])
            for segment in self.segments
            if segment["planned_path_length_m"] is not None
        )
        total_actual = sum(
            float(segment["actual_path_length_m"])
            for segment in self.segments
            if segment["actual_path_length_m"] is not None
        )

        obstacle_clearances = [
            sample["obstacle_clearance_m"]
            for sample in self.samples
            if math.isfinite(
                sample["obstacle_clearance_m"]
            )
        ]
        safety_clearances = [
            sample["safety_zone_clearance_m"]
            for sample in self.samples
            if math.isfinite(
                sample["safety_zone_clearance_m"]
            )
        ]
        rpm_values = []

        for sample in self.samples:
            for key in [
                "rpm_front_left",
                "rpm_front_right",
                "rpm_rear_right",
                "rpm_rear_left",
            ]:
                value = sample[key]
                if math.isfinite(value):
                    rpm_values.append(value)

        final_errors = [
            float(segment["final_error_m"])
            for segment in self.segments
            if segment["final_error_m"] is not None
        ]

        mission_time = None
        if (
            self.mission_start_time is not None
            and self.final_arrival_time is not None
        ):
            mission_time = (
                self.final_arrival_time
                - self.mission_start_time
            )

        path_ratio = None
        if total_planned > 1.0e-9:
            path_ratio = total_actual / total_planned

        maximum_rpm = max(rpm_values) if rpm_values else None
        saturation_ratio = None

        if rpm_values:
            saturation_ratio = (
                sum(
                    1
                    for value in rpm_values
                    if value >= 0.99 * self.rpm_limit
                )
                / len(rpm_values)
            )

        criteria = {
            "all_goals_reached": (
                success
                and len(self.segments) == len(self.goals)
            ),
            "all_final_errors_below_0_3_m": (
                len(final_errors) == len(self.goals)
                and all(
                    value < 0.3
                    for value in final_errors
                )
            ),
            "physical_clearance_positive": (
                bool(obstacle_clearances)
                and min(obstacle_clearances) > 0.0
            ),
            "safety_zone_clearance_positive": (
                bool(safety_clearances)
                and min(safety_clearances) > 0.0
            ),
            "no_motor_saturation": (
                maximum_rpm is not None
                and maximum_rpm
                < 0.99 * self.rpm_limit
            ),
            "path_ratio_below_1_4": (
                path_ratio is not None
                and path_ratio < 1.4
            ),
        }

        return {
            "scenario": "continuous_multi_goal_avoidance",
            "success": success,
            "termination_reason": reason,
            "overall_pass": all(criteria.values()),
            "criteria": criteria,
            "goal_count": len(self.goals),
            "goals_reached": len(self.segments),
            "total_mission_time_s": mission_time,
            "total_planned_path_length_m": total_planned,
            "total_actual_path_length_m": total_actual,
            "actual_over_planned_path_ratio": path_ratio,
            "maximum_final_error_m": (
                max(final_errors)
                if final_errors
                else None
            ),
            "mean_final_error_m": (
                sum(final_errors) / len(final_errors)
                if final_errors
                else None
            ),
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
            "maximum_motor_rpm": maximum_rpm,
            "rpm_saturation_ratio": saturation_ratio,
            "rpm_limit": self.rpm_limit,
            "segments": self.segments,
        }

    @staticmethod
    def value_text(
        value: object,
        unit: str = "",
        digits: int = 3,
    ) -> str:
        if value is None:
            return "N/A"
        return f"{float(value):.{digits}f}{unit}"

    def format_summary(
        self,
        metrics: Dict[str, object],
    ) -> str:
        lines = [
            "===== Continuous Multi-Goal Avoidance Summary =====",
            f"Overall pass: {metrics['overall_pass']}",
            (
                f"Goals reached: {metrics['goals_reached']}/"
                f"{metrics['goal_count']}"
            ),
            (
                "Total mission time: "
                + self.value_text(
                    metrics["total_mission_time_s"],
                    " s",
                )
            ),
            (
                "Total planned path length: "
                + self.value_text(
                    metrics[
                        "total_planned_path_length_m"
                    ],
                    " m",
                )
            ),
            (
                "Total actual path length: "
                + self.value_text(
                    metrics[
                        "total_actual_path_length_m"
                    ],
                    " m",
                )
            ),
            (
                "Actual/planned path ratio: "
                + self.value_text(
                    metrics[
                        "actual_over_planned_path_ratio"
                    ],
                    "",
                )
            ),
            (
                "Maximum final error: "
                + self.value_text(
                    metrics["maximum_final_error_m"],
                    " m",
                    4,
                )
            ),
            (
                "Minimum physical obstacle clearance: "
                + self.value_text(
                    metrics[
                        "minimum_obstacle_clearance_m"
                    ],
                    " m",
                )
            ),
            (
                "Minimum safety-zone clearance: "
                + self.value_text(
                    metrics[
                        "minimum_safety_zone_clearance_m"
                    ],
                    " m",
                )
            ),
            (
                "Maximum motor RPM: "
                + self.value_text(
                    metrics["maximum_motor_rpm"],
                    " RPM",
                    1,
                )
            ),
            "",
            "Per-goal results:",
        ]

        for segment in self.segments:
            lines.append(
                "- Goal "
                f"{segment['goal_index']}: "
                f"time={segment['arrival_time_s']:.3f} s, "
                f"error={segment['final_error_m']:.4f} m, "
                "planned="
                f"{segment['planned_path_length_m']:.3f} m, "
                "actual="
                f"{segment['actual_path_length_m']:.3f} m"
            )

        lines.append("")
        lines.append("Acceptance criteria:")

        for key, passed in metrics["criteria"].items():
            lines.append(
                f"- {key}: {'PASS' if passed else 'FAIL'}"
            )

        return "\n".join(lines) + "\n"

    def write_results(
        self,
        success: bool,
        reason: str,
    ) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = (
            self.results_root
            / f"multigoal_{timestamp}"
        )
        output_dir.mkdir(
            parents=True,
            exist_ok=False,
        )

        if self.samples:
            fieldnames = list(self.samples[0].keys())

            with (output_dir / "flight_data.csv").open(
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
        summary_text = self.format_summary(metrics)

        (output_dir / "mission_metrics.json").write_text(
            json.dumps(
                metrics,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        (output_dir / "mission_summary.txt").write_text(
            summary_text,
            encoding="utf-8",
        )

        self.create_plots(output_dir)

        print()
        print(summary_text)
        return output_dir

    def create_plots(
        self,
        output_dir: Path,
    ) -> None:
        if not self.samples:
            return

        x_values = [
            sample["x_m"]
            for sample in self.samples
        ]
        y_values = [
            sample["y_m"]
            for sample in self.samples
        ]

        plt.figure(figsize=(9, 7))
        plt.plot(
            x_values,
            y_values,
            label="actual trajectory",
        )

        for segment in self.segments:
            points = segment["planned_points"]
            if points:
                plt.plot(
                    [point[0] for point in points],
                    [point[1] for point in points],
                    marker="o",
                    label=(
                        f"planned segment "
                        f"{segment['goal_index']}"
                    ),
                )

        plt.scatter(
            [goal[0] for goal in self.goals],
            [goal[1] for goal in self.goals],
            marker="x",
            s=90,
            label="mission goals",
        )

        for index, goal in enumerate(
            self.goals,
            start=1,
        ):
            plt.text(
                goal[0],
                goal[1],
                f"G{index}",
            )

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Continuous Multi-Goal Avoidance")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "multi_goal_trajectory.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(10, 5.5))
        plt.plot(
            [
                sample["mission_time_s"]
                for sample in self.samples
            ],
            [
                sample["position_error_m"]
                for sample in self.samples
            ],
            label="active-goal error",
        )
        plt.axhline(
            self.arrival_tolerance_m,
            linestyle="--",
            label="arrival tolerance",
        )
        plt.xlabel("Mission time (s)")
        plt.ylabel("Position error (m)")
        plt.title("Multi-Goal Tracking Error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "multi_goal_error.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.5, 5.2))
        plt.bar(
            [
                f"Goal {segment['goal_index']}"
                for segment in self.segments
            ],
            [
                segment["arrival_time_s"]
                for segment in self.segments
            ],
        )
        plt.xlabel("Mission goal")
        plt.ylabel("Arrival time (s)")
        plt.title("Per-Goal Arrival Time")
        plt.grid(True, axis="y")
        plt.tight_layout()
        plt.savefig(
            output_dir / "per_goal_arrival_time.png",
            dpi=180,
        )
        plt.close()

        plt.figure(figsize=(8.5, 5.2))
        plt.bar(
            [
                f"Goal {segment['goal_index']}"
                for segment in self.segments
            ],
            [
                segment["final_error_m"]
                for segment in self.segments
            ],
        )
        plt.axhline(
            0.3,
            linestyle="--",
            label="0.3 m requirement",
        )
        plt.xlabel("Mission goal")
        plt.ylabel("Final error (m)")
        plt.title("Per-Goal Final Error")
        plt.grid(True, axis="y")
        plt.legend()
        plt.tight_layout()
        plt.savefig(
            output_dir / "per_goal_final_error.png",
            dpi=180,
        )
        plt.close()

        obstacle_clearances = [
            sample["obstacle_clearance_m"]
            for sample in self.samples
        ]

        if all(
            math.isfinite(value)
            for value in obstacle_clearances
        ):
            plt.figure(figsize=(10, 5.5))
            plt.plot(
                [
                    sample["mission_time_s"]
                    for sample in self.samples
                ],
                obstacle_clearances,
                label="physical obstacle clearance",
            )
            plt.axhline(
                0.0,
                linestyle="--",
                label="collision boundary",
            )
            plt.xlabel("Mission time (s)")
            plt.ylabel("Clearance (m)")
            plt.title("Multi-Goal Obstacle Clearance")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.savefig(
                output_dir
                / "multi_goal_obstacle_clearance.png",
                dpi=180,
            )
            plt.close()

    def finish_mission(
        self,
        success: bool,
        reason: str,
    ) -> None:
        if self.finished:
            return

        self.finished = True

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
            "Multi-goal mission complete; shutting down"
        )
        self.create_timer(0.5, self.request_shutdown)

    def request_shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiGoalEvaluation()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
