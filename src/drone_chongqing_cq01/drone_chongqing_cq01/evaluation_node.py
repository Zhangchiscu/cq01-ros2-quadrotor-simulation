#!/usr/bin/env python3
"""Automated CQ-01 evaluation."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, String


def quaternion_to_tilt_deg(x: float, y: float, z: float, w: float) -> float:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    return math.degrees(math.hypot(roll, pitch))


class CQ01Evaluation(Node):
    def __init__(self) -> None:
        super().__init__("cq01_eling_liziba_evaluation")

        self.declare_parameter("timeout_seconds", 120.0)
        self.declare_parameter("settle_seconds", 3.0)
        self.declare_parameter("final_error_limit_m", 0.50)
        self.declare_parameter("minimum_terrain_clearance_m", 0.20)
        self.declare_parameter("minimum_building_clearance_m", 0.30)
        self.declare_parameter("maximum_tilt_deg", 30.0)
        self.declare_parameter("maximum_rpm", 10000.0)

        self.timeout_seconds = float(
            self.get_parameter("timeout_seconds").value
        )
        self.settle_seconds = float(
            self.get_parameter("settle_seconds").value
        )
        self.final_error_limit = float(
            self.get_parameter("final_error_limit_m").value
        )
        self.minimum_terrain_clearance_limit = float(
            self.get_parameter("minimum_terrain_clearance_m").value
        )
        self.minimum_building_clearance_limit = float(
            self.get_parameter("minimum_building_clearance_m").value
        )
        self.maximum_tilt_limit = float(
            self.get_parameter("maximum_tilt_deg").value
        )
        self.maximum_rpm = float(
            self.get_parameter("maximum_rpm").value
        )

        share = Path(get_package_share_directory("drone_chongqing_cq01"))
        self.manifest = json.loads(
            (share / "runtime_data" / "cq01_manifest.json").read_text(
                encoding="utf-8"
            )
        )
        data = np.load(
            share / "runtime_data" / "cq01_eling_liziba_data.npz",
            allow_pickle=False,
        )
        self.terrain_surface = data["terrain_surface"]
        self.building_points = data["building_points"].astype(np.float64)
        self.goal = np.asarray(
            self.manifest["simulation"]["final_goal"],
            dtype=np.float64,
        )

        simulation = self.manifest["simulation"]
        self.bounds = simulation["bounds"]
        self.resolution = float(simulation["resolution_m"])

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Odometry, "/drone/odom", self.odom_callback, 50
        )
        self.create_subscription(
            Float32MultiArray,
            "/drone/motor_rpm",
            self.rpm_callback,
            50,
        )
        self.create_subscription(
            String,
            "/drone/planner_3d/status",
            self.planner_callback,
            qos,
        )
        self.create_subscription(
            NavPath,
            "/drone/planned_path_3d",
            self.path_callback,
            qos,
        )

        self.started_at = time.monotonic()
        self.path_ready_at: Optional[float] = None
        self.completed_at: Optional[float] = None
        self.settle_started_at: Optional[float] = None
        self.planner_payload: Dict[str, object] = {}
        self.planned_path_length = float("nan")
        self.latest_position: Optional[np.ndarray] = None

        self.rows: List[Dict[str, float]] = []
        self.actual_path_length = 0.0
        self.previous_position: Optional[np.ndarray] = None
        self.minimum_terrain_clearance = float("inf")
        self.minimum_building_clearance = float("inf")
        self.maximum_tilt = 0.0
        self.maximum_observed_rpm = 0.0
        self.rpm_samples = 0
        self.rpm_saturated_samples = 0
        self.finished = False
        self.last_sample_time = 0.0

        self.timer = self.create_timer(0.2, self.step)
        self.get_logger().info("CQ-01 evaluation started")

    def terrain_clearance(self, position: np.ndarray) -> float:
        ix = int(
            math.floor(
                (float(position[0]) - float(self.bounds["min_x"]))
                / self.resolution
            )
        )
        iy = int(
            math.floor(
                (float(position[1]) - float(self.bounds["min_y"]))
                / self.resolution
            )
        )

        if (
            ix < 0
            or iy < 0
            or ix >= self.terrain_surface.shape[0]
            or iy >= self.terrain_surface.shape[1]
        ):
            return float("-inf")

        ground = float(self.terrain_surface[ix, iy])
        if not np.isfinite(ground):
            return float("-inf")
        return float(position[2] - ground)

    def building_clearance(self, position: np.ndarray) -> float:
        if not len(self.building_points):
            return float("inf")
        distances = np.linalg.norm(
            self.building_points - position[None, :],
            axis=1,
        )
        # Voxel centres represent occupied cubes; subtract half a voxel.
        return float(np.min(distances) - 0.5 * self.resolution)

    def odom_callback(self, message: Odometry) -> None:
        now = time.monotonic()
        position = np.asarray(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        self.latest_position = position

        if self.previous_position is not None:
            self.actual_path_length += float(
                np.linalg.norm(position - self.previous_position)
            )
        self.previous_position = position.copy()

        orientation = message.pose.pose.orientation
        tilt = quaternion_to_tilt_deg(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.maximum_tilt = max(self.maximum_tilt, tilt)

        if now - self.last_sample_time < 0.2:
            return
        self.last_sample_time = now

        terrain_clearance = self.terrain_clearance(position)
        building_clearance = self.building_clearance(position)

        self.minimum_terrain_clearance = min(
            self.minimum_terrain_clearance,
            terrain_clearance,
        )
        self.minimum_building_clearance = min(
            self.minimum_building_clearance,
            building_clearance,
        )

        error = float(np.linalg.norm(position - self.goal))
        velocity = message.twist.twist.linear
        speed = math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )

        self.rows.append({
            "time_s": now - self.started_at,
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
            "speed_mps": speed,
            "goal_error_m": error,
            "terrain_clearance_m": terrain_clearance,
            "building_clearance_m": building_clearance,
            "tilt_deg": tilt,
        })

    def rpm_callback(self, message: Float32MultiArray) -> None:
        if not message.data:
            return
        values = [abs(float(item)) for item in message.data]
        self.rpm_samples += 1
        current_maximum = max(values)
        self.maximum_observed_rpm = max(
            self.maximum_observed_rpm,
            current_maximum,
        )
        if current_maximum >= 0.999 * self.maximum_rpm:
            self.rpm_saturated_samples += 1

    def planner_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(data.get("state", ""))
        if state == "path_ready":
            self.path_ready_at = time.monotonic()
            self.planner_payload = data
        elif state == "final_waypoint_reached":
            self.completed_at = time.monotonic()
            self.settle_started_at = time.monotonic()
        elif state == "planning_failed":
            self.planner_payload = data
            self.finish(force_failure_reason=str(data.get("reason", "unknown")))

    def path_callback(self, message: NavPath) -> None:
        points = [
            np.asarray(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ],
                dtype=np.float64,
            )
            for pose in message.poses
        ]
        if len(points) >= 2:
            self.planned_path_length = float(
                sum(
                    np.linalg.norm(second - first)
                    for first, second in zip(points[:-1], points[1:])
                )
            )

    def step(self) -> None:
        if self.finished:
            return

        now = time.monotonic()
        if now - self.started_at >= self.timeout_seconds:
            self.finish(force_failure_reason="timeout")
            return

        if (
            self.settle_started_at is not None
            and now - self.settle_started_at >= self.settle_seconds
        ):
            self.finish()

    def result_directory(self) -> Path:
        root = (
            Path.home()
            / "drone_task"
            / "your_drone_sim_ws"
            / "results"
            / "chongqing_cq01"
        )
        stamp = time.strftime("%Y%m%d_%H%M%S")
        directory = root / f"cq01_eling_liziba_{stamp}"
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def finish(self, force_failure_reason: str = "") -> None:
        if self.finished:
            return
        self.finished = True

        final_error = (
            float(np.linalg.norm(self.latest_position - self.goal))
            if self.latest_position is not None
            else float("inf")
        )
        arrival_time = (
            self.completed_at - self.path_ready_at
            if self.completed_at is not None and self.path_ready_at is not None
            else float("nan")
        )
        saturation_fraction = (
            self.rpm_saturated_samples / self.rpm_samples
            if self.rpm_samples
            else 0.0
        )

        checks = {
            "planner_completed": (
                self.completed_at is not None and not force_failure_reason
            ),
            "final_error": final_error <= self.final_error_limit,
            "terrain_clearance": (
                self.minimum_terrain_clearance
                >= self.minimum_terrain_clearance_limit
            ),
            "building_clearance": (
                self.minimum_building_clearance
                >= self.minimum_building_clearance_limit
            ),
            "maximum_tilt": self.maximum_tilt <= self.maximum_tilt_limit,
            "rpm_saturation": saturation_fraction <= 0.01,
            "three_dimensional_need": (
                not bool(
                    self.manifest["offline_validation"][
                        "fixed_height_path_exists"
                    ]
                )
            ),
            "static_obstacle_requirement": (
                int(
                    self.manifest["real_world"][
                        "direct_corridor_buildings"
                    ]
                )
                >= 5
            ),
            "mountain_relief_requirement": (
                float(
                    self.manifest["real_world"][
                        "terrain_elevation_difference_m"
                    ]
                )
                >= 180.0
            ),
        }
        passed = all(checks.values()) and not force_failure_reason

        summary = {
            "scenario": self.manifest["scenario_cn"],
            "pass": passed,
            "failure_reason": force_failure_reason,
            "checks": checks,
            "goal": self.goal.tolist(),
            "final_position": (
                self.latest_position.tolist()
                if self.latest_position is not None
                else None
            ),
            "final_error_m": final_error,
            "arrival_time_s": arrival_time,
            "planned_path_length_m": self.planned_path_length,
            "actual_path_length_m": self.actual_path_length,
            "minimum_terrain_clearance_m": self.minimum_terrain_clearance,
            "minimum_building_clearance_m": self.minimum_building_clearance,
            "maximum_tilt_deg": self.maximum_tilt,
            "maximum_rpm": self.maximum_observed_rpm,
            "rpm_saturation_fraction": saturation_fraction,
            "planner_status": self.planner_payload,
            "geometric_scale": self.manifest["geometric_scale"],
            "real_world": self.manifest["real_world"],
            "requirements_mapping": self.manifest[
                "requirements_mapping"
            ],
        }

        directory = self.result_directory()
        (directory / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        if self.rows:
            with (directory / "telemetry.csv").open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=list(self.rows[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.rows)

        try:
            import matplotlib.pyplot as plt

            if self.rows:
                times = [row["time_s"] for row in self.rows]

                figure = plt.figure(figsize=(8, 5))
                axis = figure.add_subplot(111)
                axis.plot(
                    [row["x"] for row in self.rows],
                    [row["y"] for row in self.rows],
                    label="Actual trajectory",
                )
                axis.scatter(
                    [self.goal[0]],
                    [self.goal[1]],
                    marker="x",
                    s=80,
                    label="Goal",
                )
                axis.set_aspect("equal", adjustable="box")
                axis.set_xlabel("x (m)")
                axis.set_ylabel("y (m)")
                axis.legend()
                axis.grid(True)
                figure.tight_layout()
                figure.savefig(directory / "xy_trajectory.png", dpi=160)
                plt.close(figure)

                figure = plt.figure(figsize=(8, 5))
                axis = figure.add_subplot(111)
                axis.plot(
                    times,
                    [row["goal_error_m"] for row in self.rows],
                    label="Goal error",
                )
                axis.set_xlabel("time (s)")
                axis.set_ylabel("error (m)")
                axis.grid(True)
                axis.legend()
                figure.tight_layout()
                figure.savefig(directory / "goal_error.png", dpi=160)
                plt.close(figure)

                figure = plt.figure(figsize=(8, 5))
                axis = figure.add_subplot(111)
                axis.plot(
                    times,
                    [row["terrain_clearance_m"] for row in self.rows],
                    label="Terrain clearance",
                )
                axis.plot(
                    times,
                    [row["building_clearance_m"] for row in self.rows],
                    label="Building clearance",
                )
                axis.axhline(
                    self.minimum_terrain_clearance_limit,
                    linestyle="--",
                    label="Terrain limit",
                )
                axis.axhline(
                    self.minimum_building_clearance_limit,
                    linestyle=":",
                    label="Building limit",
                )
                axis.set_xlabel("time (s)")
                axis.set_ylabel("clearance (m)")
                axis.grid(True)
                axis.legend()
                figure.tight_layout()
                figure.savefig(directory / "clearance.png", dpi=160)
                plt.close(figure)
        except Exception as error:
            self.get_logger().warning(
                f"Plot generation skipped: {error}"
            )

        self.get_logger().info(
            "CQ-01 result: "
            f"pass={passed}, "
            f"final_error={final_error:.3f} m, "
            f"terrain_clearance={self.minimum_terrain_clearance:.3f} m, "
            f"building_clearance={self.minimum_building_clearance:.3f} m, "
            f"max_tilt={self.maximum_tilt:.2f} deg"
        )
        self.get_logger().info(f"Result directory: {directory}")

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01Evaluation()
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
