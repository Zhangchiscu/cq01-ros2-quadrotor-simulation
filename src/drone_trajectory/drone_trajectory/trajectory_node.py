#!/usr/bin/env python3
"""Collision-aware path smoothing and dynamically limited trajectory execution."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

Point3 = Tuple[float, float, float]
Voxel = Tuple[int, int, int]


class TrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_executor")

        self.declare_parameter("mode", "smooth")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("sample_spacing_m", 0.06)
        self.declare_parameter("maximum_speed_mps", 0.90)
        self.declare_parameter("maximum_acceleration_mps2", 0.70)
        self.declare_parameter("maximum_lateral_acceleration_mps2", 0.55)
        self.declare_parameter("maximum_vertical_speed_mps", 0.50)
        self.declare_parameter("maximum_jerk_mps3", 0.90)
        self.declare_parameter("minimum_trajectory_duration_s", 8.0)
        self.declare_parameter("time_scaling_safety_factor", 1.08)
        self.declare_parameter("smoothing_iterations", 3)
        self.declare_parameter("collision_check_spacing_m", 0.05)
        self.declare_parameter("start_delay_s", 0.35)
        self.declare_parameter("goal_hold_seconds", 2.0)

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {"raw", "smooth"}:
            raise ValueError("mode must be 'raw' or 'smooth'")

        self.control_rate = float(
            self.get_parameter("control_rate_hz").value
        )
        self.sample_spacing = float(
            self.get_parameter("sample_spacing_m").value
        )
        self.maximum_speed = float(
            self.get_parameter("maximum_speed_mps").value
        )
        self.maximum_acceleration = float(
            self.get_parameter("maximum_acceleration_mps2").value
        )
        self.maximum_lateral_acceleration = float(
            self.get_parameter(
                "maximum_lateral_acceleration_mps2"
            ).value
        )
        self.maximum_vertical_speed = float(
            self.get_parameter("maximum_vertical_speed_mps").value
        )
        self.maximum_jerk = float(
            self.get_parameter("maximum_jerk_mps3").value
        )
        self.minimum_trajectory_duration = float(
            self.get_parameter(
                "minimum_trajectory_duration_s"
            ).value
        )
        self.time_scaling_safety_factor = float(
            self.get_parameter(
                "time_scaling_safety_factor"
            ).value
        )
        self.smoothing_iterations = int(
            self.get_parameter("smoothing_iterations").value
        )
        self.collision_check_spacing = float(
            self.get_parameter("collision_check_spacing_m").value
        )
        self.start_delay = float(
            self.get_parameter("start_delay_s").value
        )
        self.goal_hold_seconds = float(
            self.get_parameter("goal_hold_seconds").value
        )

        self.metadata: Optional[Dict[str, object]] = None
        self.pending_voxel_points: Optional[np.ndarray] = None
        self.occupied: Set[Voxel] = set()
        self.latest_odom: Optional[Odometry] = None

        self.reference_points: Optional[np.ndarray] = None
        self.reference_times: Optional[np.ndarray] = None
        self.reference_status: Optional[Dict[str, object]] = None
        self.execution_started: Optional[float] = None
        self.completed_at: Optional[float] = None
        self.complete_published = False

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal",
            10,
        )
        self.path_publisher = self.create_publisher(
            Path,
            "/drone/trajectory_reference",
            transient_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/drone/trajectory/status",
            transient_qos,
        )

        self.create_subscription(
            Path,
            "/drone/planned_path_3d",
            self.path_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            "/drone/voxel_cloud",
            self.cloud_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/voxel_metadata",
            self.metadata_callback,
            transient_qos,
        )
        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            50,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.execution_step,
        )

        self.get_logger().info(
            f"Trajectory executor started: mode={self.mode}"
        )

    def metadata_callback(self, message: String) -> None:
        self.metadata = json.loads(message.data)
        self.rebuild_occupancy()

    def cloud_callback(self, message: PointCloud2) -> None:
        self.pending_voxel_points = self.pointcloud_to_xyz(message)
        self.rebuild_occupancy()

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def rebuild_occupancy(self) -> None:
        if self.metadata is None or self.pending_voxel_points is None:
            return

        occupied: Set[Voxel] = set()
        for point in self.pending_voxel_points:
            voxel = self.world_to_voxel(point)
            if self.index_in_bounds(voxel):
                occupied.add(voxel)
        self.occupied = occupied

    def pointcloud_to_xyz(self, message: PointCloud2) -> np.ndarray:
        point_count = int(message.width) * int(message.height)
        if point_count <= 0:
            return np.empty((0, 3), dtype=np.float64)

        offsets = {
            field.name: int(field.offset)
            for field in message.fields
        }
        required = {"x", "y", "z"}
        if not required.issubset(offsets):
            raise RuntimeError(
                "Voxel PointCloud2 does not contain x/y/z"
            )

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

    @property
    def resolution(self) -> float:
        assert self.metadata is not None
        return float(self.metadata["resolution_m"])

    @property
    def bounds(self) -> Dict[str, float]:
        assert self.metadata is not None
        return {
            key: float(value)
            for key, value in self.metadata["bounds"].items()
        }

    @property
    def dimensions(self) -> Voxel:
        assert self.metadata is not None
        dimensions = self.metadata["dimensions"]
        return (
            int(dimensions["x"]),
            int(dimensions["y"]),
            int(dimensions["z"]),
        )

    def world_to_voxel(self, point: Sequence[float]) -> Voxel:
        bounds = self.bounds
        return (
            int(math.floor(
                (float(point[0]) - bounds["min_x"])
                / self.resolution
            )),
            int(math.floor(
                (float(point[1]) - bounds["min_y"])
                / self.resolution
            )),
            int(math.floor(
                (float(point[2]) - bounds["min_z"])
                / self.resolution
            )),
        )

    def index_in_bounds(self, voxel: Voxel) -> bool:
        dimensions = self.dimensions
        return (
            0 <= voxel[0] < dimensions[0]
            and 0 <= voxel[1] < dimensions[1]
            and 0 <= voxel[2] < dimensions[2]
        )

    def point_is_free(self, point: Sequence[float]) -> bool:
        voxel = self.world_to_voxel(point)
        return (
            self.index_in_bounds(voxel)
            and voxel not in self.occupied
        )

    def segment_is_free(
        self,
        first: Sequence[float],
        second: Sequence[float],
    ) -> bool:
        segment_length = math.dist(first, second)
        sample_count = max(
            1,
            int(math.ceil(
                segment_length / self.collision_check_spacing
            )),
        )
        for index in range(sample_count + 1):
            ratio = index / sample_count
            point = (
                float(first[0])
                + ratio * (float(second[0]) - float(first[0])),
                float(first[1])
                + ratio * (float(second[1]) - float(first[1])),
                float(first[2])
                + ratio * (float(second[2]) - float(first[2])),
            )
            if not self.point_is_free(point):
                return False
        return True

    def polyline_is_free(self, points: np.ndarray) -> bool:
        return all(
            self.segment_is_free(first, second)
            for first, second in zip(points[:-1], points[1:])
        )

    def chaikin(
        self,
        points: np.ndarray,
        iterations: int,
    ) -> np.ndarray:
        output = np.asarray(points, dtype=np.float64)

        for _ in range(iterations):
            if len(output) < 3:
                break
            new_points = [output[0]]
            for first, second in zip(output[:-1], output[1:]):
                q_point = 0.75 * first + 0.25 * second
                r_point = 0.25 * first + 0.75 * second
                new_points.extend([q_point, r_point])
            new_points.append(output[-1])
            output = np.asarray(new_points, dtype=np.float64)

        return output

    def resample_polyline(
        self,
        points: np.ndarray,
        spacing: float,
    ) -> np.ndarray:
        if len(points) <= 1:
            return points.copy()

        segment_lengths = np.linalg.norm(
            np.diff(points, axis=0),
            axis=1,
        )
        cumulative = np.concatenate((
            [0.0],
            np.cumsum(segment_lengths),
        ))
        total_length = float(cumulative[-1])

        if total_length <= 1.0e-9:
            return points[[0, -1]].copy()

        sample_count = max(
            2,
            int(math.ceil(total_length / spacing)) + 1,
        )
        target_distances = np.linspace(
            0.0,
            total_length,
            sample_count,
        )

        output = np.zeros((sample_count, 3), dtype=np.float64)
        for axis in range(3):
            output[:, axis] = np.interp(
                target_distances,
                cumulative,
                points[:, axis],
            )
        return output

    def curvature(self, points: np.ndarray) -> np.ndarray:
        values = np.zeros(len(points), dtype=np.float64)
        if len(points) < 3:
            return values

        for index in range(1, len(points) - 1):
            first = points[index] - points[index - 1]
            second = points[index + 1] - points[index]
            first_length = float(np.linalg.norm(first))
            second_length = float(np.linalg.norm(second))

            if first_length <= 1.0e-9 or second_length <= 1.0e-9:
                continue

            cosine = float(
                np.clip(
                    np.dot(first, second)
                    / (first_length * second_length),
                    -1.0,
                    1.0,
                )
            )
            angle = math.acos(cosine)
            values[index] = (
                angle
                / max(
                    0.5 * (first_length + second_length),
                    1.0e-6,
                )
            )

        return values

    def derivative_metrics(
        self,
        points: np.ndarray,
        time_array: np.ndarray,
    ) -> Dict[str, float]:
        if len(points) < 3:
            return {
                "maximum_reference_speed_mps": 0.0,
                "maximum_reference_acceleration_mps2": 0.0,
                "rms_reference_acceleration_mps2": 0.0,
                "maximum_reference_jerk_mps3": 0.0,
                "rms_reference_jerk_mps3": 0.0,
            }

        edge_order = 2 if len(points) >= 3 else 1

        velocities = np.gradient(
            points,
            time_array,
            axis=0,
            edge_order=edge_order,
        )
        accelerations = np.gradient(
            velocities,
            time_array,
            axis=0,
            edge_order=edge_order,
        )
        jerks = np.gradient(
            accelerations,
            time_array,
            axis=0,
            edge_order=edge_order,
        )

        speed_norm = np.linalg.norm(
            velocities,
            axis=1,
        )
        acceleration_norm = np.linalg.norm(
            accelerations,
            axis=1,
        )
        jerk_norm = np.linalg.norm(
            jerks,
            axis=1,
        )

        return {
            "maximum_reference_speed_mps": float(
                np.max(speed_norm)
            ),
            "maximum_reference_acceleration_mps2": float(
                np.max(acceleration_norm)
            ),
            "rms_reference_acceleration_mps2": float(
                math.sqrt(
                    np.mean(acceleration_norm ** 2)
                )
            ),
            "maximum_reference_jerk_mps3": float(
                np.max(jerk_norm)
            ),
            "rms_reference_jerk_mps3": float(
                math.sqrt(
                    np.mean(jerk_norm ** 2)
                )
            ),
        }

    def inverse_quintic_progress(
        self,
        progress: float,
    ) -> float:
        if progress <= 0.0:
            return 0.0
        if progress >= 1.0:
            return 1.0

        lower = 0.0
        upper = 1.0

        for _ in range(60):
            middle = 0.5 * (lower + upper)
            value = (
                10.0 * middle ** 3
                - 15.0 * middle ** 4
                + 6.0 * middle ** 5
            )
            if value < progress:
                lower = middle
            else:
                upper = middle

        return 0.5 * (lower + upper)

    def minimum_jerk_time_parameterize(
        self,
        points: np.ndarray,
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        segment_lengths = np.linalg.norm(
            np.diff(points, axis=0),
            axis=1,
        )
        cumulative = np.concatenate((
            [0.0],
            np.cumsum(segment_lengths),
        ))
        path_length = float(cumulative[-1])

        if path_length <= 1.0e-9:
            time_array = np.linspace(
                0.0,
                self.minimum_trajectory_duration,
                len(points),
            )
            metrics = self.derivative_metrics(
                points,
                time_array,
            )
            metrics.update({
                "duration_s": float(time_array[-1]),
                "path_length_m": path_length,
                "time_parameterization": (
                    "minimum_jerk_quintic"
                ),
            })
            return time_array, metrics

        progress = cumulative / path_length
        unit_time = np.asarray(
            [
                self.inverse_quintic_progress(
                    float(value)
                )
                for value in progress
            ],
            dtype=np.float64,
        )

        unit_time[0] = 0.0
        unit_time[-1] = 1.0

        unit_metrics = self.derivative_metrics(
            points,
            unit_time,
        )

        speed_scale = (
            unit_metrics[
                "maximum_reference_speed_mps"
            ]
            / max(self.maximum_speed, 1.0e-6)
        )
        acceleration_scale = math.sqrt(
            unit_metrics[
                "maximum_reference_acceleration_mps2"
            ]
            / max(self.maximum_acceleration, 1.0e-6)
        )
        jerk_scale = (
            unit_metrics[
                "maximum_reference_jerk_mps3"
            ]
            / max(self.maximum_jerk, 1.0e-6)
        ) ** (1.0 / 3.0)

        unit_velocities = np.gradient(
            points,
            unit_time,
            axis=0,
            edge_order=2,
        )
        maximum_unit_vertical_speed = float(
            np.max(
                np.abs(unit_velocities[:, 2])
            )
        )
        vertical_scale = (
            maximum_unit_vertical_speed
            / max(
                self.maximum_vertical_speed,
                1.0e-6,
            )
        )

        duration = max(
            self.minimum_trajectory_duration,
            speed_scale,
            acceleration_scale,
            jerk_scale,
            vertical_scale,
        )
        duration *= self.time_scaling_safety_factor

        time_array = unit_time * duration
        metrics = self.derivative_metrics(
            points,
            time_array,
        )
        metrics.update({
            "duration_s": float(time_array[-1]),
            "path_length_m": path_length,
            "time_parameterization": (
                "minimum_jerk_quintic"
            ),
            "configured_maximum_speed_mps": (
                self.maximum_speed
            ),
            "configured_maximum_acceleration_mps2": (
                self.maximum_acceleration
            ),
            "configured_maximum_jerk_mps3": (
                self.maximum_jerk
            ),
            "configured_maximum_vertical_speed_mps": (
                self.maximum_vertical_speed
            ),
        })
        return time_array, metrics

    def acceleration_limited_time_parameterize(
        self,
        points: np.ndarray,
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        segment_vectors = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(
            segment_vectors,
            axis=1,
        )

        curvature_values = self.curvature(points)
        speed_limits = np.full(
            len(points),
            self.maximum_speed,
            dtype=np.float64,
        )

        for index, curvature_value in enumerate(
            curvature_values
        ):
            if curvature_value > 1.0e-6:
                speed_limits[index] = min(
                    speed_limits[index],
                    math.sqrt(
                        self.maximum_lateral_acceleration
                        / curvature_value
                    ),
                )

        for index, vector in enumerate(
            segment_vectors
        ):
            length = float(segment_lengths[index])
            if length <= 1.0e-9:
                continue

            vertical_ratio = (
                abs(float(vector[2])) / length
            )
            if vertical_ratio > 1.0e-6:
                limit = (
                    self.maximum_vertical_speed
                    / vertical_ratio
                )
                speed_limits[index] = min(
                    speed_limits[index],
                    limit,
                )
                speed_limits[index + 1] = min(
                    speed_limits[index + 1],
                    limit,
                )

        speed_limits[0] = 0.0
        speed_limits[-1] = 0.0
        speeds = speed_limits.copy()

        for index in range(1, len(points)):
            ds = float(
                segment_lengths[index - 1]
            )
            reachable = math.sqrt(
                max(
                    0.0,
                    speeds[index - 1] ** 2
                    + 2.0
                    * self.maximum_acceleration
                    * ds,
                )
            )
            speeds[index] = min(
                speeds[index],
                reachable,
            )

        for index in range(
            len(points) - 2,
            -1,
            -1,
        ):
            ds = float(segment_lengths[index])
            reachable = math.sqrt(
                max(
                    0.0,
                    speeds[index + 1] ** 2
                    + 2.0
                    * self.maximum_acceleration
                    * ds,
                )
            )
            speeds[index] = min(
                speeds[index],
                reachable,
            )

        time_values = [0.0]

        for index, ds in enumerate(
            segment_lengths
        ):
            denominator = (
                speeds[index]
                + speeds[index + 1]
            )
            if denominator <= 1.0e-4:
                dt = max(
                    0.05,
                    float(ds)
                    / max(
                        self.maximum_speed * 0.25,
                        0.10,
                    ),
                )
            else:
                dt = (
                    2.0 * float(ds)
                    / denominator
                )
            time_values.append(
                time_values[-1] + dt
            )

        time_array = np.asarray(
            time_values,
            dtype=np.float64,
        )
        metrics = self.derivative_metrics(
            points,
            time_array,
        )
        metrics.update({
            "duration_s": float(time_array[-1]),
            "path_length_m": float(
                np.sum(segment_lengths)
            ),
            "time_parameterization": (
                "acceleration_limited"
            ),
        })
        return time_array, metrics

    def time_parameterize(
        self,
        points: np.ndarray,
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        if self.mode == "smooth":
            return self.minimum_jerk_time_parameterize(
                points
            )

        return self.acceleration_limited_time_parameterize(
            points
        )

    def path_callback(self, message: Path) -> None:
        if (
            self.metadata is None
            or not self.occupied
            or len(message.poses) < 2
        ):
            return

        source = np.asarray(
            [
                [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                    float(pose.pose.position.z),
                ]
                for pose in message.poses
            ],
            dtype=np.float64,
        )

        smoothing_used = 0
        fallback_to_raw = False
        processed = source.copy()

        if self.mode == "smooth":
            found = False
            for iterations in range(
                self.smoothing_iterations,
                0,
                -1,
            ):
                candidate = self.chaikin(source, iterations)
                candidate = self.resample_polyline(
                    candidate,
                    self.sample_spacing,
                )
                if self.polyline_is_free(candidate):
                    processed = candidate
                    smoothing_used = iterations
                    found = True
                    break

            if not found:
                fallback_to_raw = True
                processed = self.resample_polyline(
                    source,
                    self.sample_spacing,
                )
        else:
            processed = self.resample_polyline(
                source,
                self.sample_spacing,
            )

        if not self.polyline_is_free(processed):
            self.publish_status({
                "state": "trajectory_failed",
                "success": False,
                "reason": "processed_path_in_collision",
                "mode": self.mode,
            })
            self.get_logger().error(
                "Processed trajectory is not collision-free"
            )
            return

        reference_times, metrics = self.time_parameterize(processed)

        self.reference_points = processed
        self.reference_times = reference_times
        self.execution_started = None
        self.completed_at = None
        self.complete_published = False

        status = {
            "state": "trajectory_ready",
            "success": True,
            "mode": self.mode,
            "source_path_points": int(len(source)),
            "reference_points": int(len(processed)),
            "smoothing_iterations_used": int(smoothing_used),
            "fallback_to_raw": bool(fallback_to_raw),
            "collision_checked": True,
            **metrics,
        }
        self.reference_status = status
        self.publish_reference_path(processed)
        self.publish_status(status)

        self.get_logger().info(
            "Trajectory ready: "
            f"mode={self.mode}, "
            f"points={len(processed)}, "
            f"duration={metrics['duration_s']:.2f} s, "
            f"max_acc={metrics['maximum_reference_acceleration_mps2']:.3f}, "
            f"max_jerk={metrics['maximum_reference_jerk_mps3']:.3f}"
        )

    def publish_reference_path(self, points: np.ndarray) -> None:
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()

        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.path_publisher.publish(message)

    def publish_status(self, data: Dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(
            data,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def interpolate_reference(self, elapsed: float) -> Point3:
        assert self.reference_points is not None
        assert self.reference_times is not None

        x = float(np.interp(
            elapsed,
            self.reference_times,
            self.reference_points[:, 0],
        ))
        y = float(np.interp(
            elapsed,
            self.reference_times,
            self.reference_points[:, 1],
        ))
        z = float(np.interp(
            elapsed,
            self.reference_times,
            self.reference_points[:, 2],
        ))
        return (x, y, z)

    def publish_goal(self, target: Point3) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)

    def execution_step(self) -> None:
        if (
            self.reference_points is None
            or self.reference_times is None
            or self.latest_odom is None
        ):
            return

        now = time.monotonic()

        if self.execution_started is None:
            self.execution_started = now + self.start_delay
            return

        if now < self.execution_started:
            self.publish_goal(tuple(
                float(value)
                for value in self.reference_points[0]
            ))
            return

        elapsed = now - self.execution_started
        duration = float(self.reference_times[-1])

        if elapsed <= duration:
            self.publish_goal(
                self.interpolate_reference(elapsed)
            )
            return

        final_point = tuple(
            float(value)
            for value in self.reference_points[-1]
        )
        self.publish_goal(final_point)

        if self.completed_at is None:
            self.completed_at = now

        if (
            not self.complete_published
            and now - self.completed_at >= self.goal_hold_seconds
        ):
            self.complete_published = True
            complete_status = dict(self.reference_status or {})
            complete_status["state"] = "trajectory_complete"
            complete_status["success"] = True
            self.publish_status(complete_status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryExecutor()

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
