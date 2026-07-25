#!/usr/bin/env python3
"""Fast collision-aware lookahead follower for CQ-01 V3.4D.

The legacy A* planner advances through dense 1 m waypoints.  With a pure
position controller that keeps the position error small, that execution mode
is unnecessarily slow.  This node consumes the collision-checked A* Path and
publishes one adaptive lookahead target.  The target remains on the path and
is shortened until the direct current-to-target segment is free in the same
inflated voxel map used by the planner.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

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


@dataclass
class Projection:
    segment_index: int
    t: float
    point: Point3
    arc_length: float
    distance: float


class FastPathFollower(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v34d_fast_path_follower')

        self.declare_parameter('control_rate_hz', 25.0)
        self.declare_parameter('target_cruise_speed_mps', 5.5)
        self.declare_parameter('base_lookahead_m', 6.0)
        self.declare_parameter('speed_error_lookahead_gain_s', 2.0)
        self.declare_parameter('minimum_lookahead_m', 3.5)
        self.declare_parameter('maximum_lookahead_m', 18.0)
        self.declare_parameter('lookahead_step_m', 1.0)
        self.declare_parameter('collision_check_spacing_m', 0.45)
        self.declare_parameter('completion_tolerance_m', 1.25)
        self.declare_parameter('completion_hold_seconds', 0.35)
        self.declare_parameter('stall_timeout_seconds', 18.0)
        self.declare_parameter('minimum_progress_m', 1.5)
        self.declare_parameter('status_period_seconds', 1.0)
        self.declare_parameter('accept_map_updates', False)

        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.target_speed = float(
            self.get_parameter('target_cruise_speed_mps').value
        )
        self.base_lookahead = float(
            self.get_parameter('base_lookahead_m').value
        )
        self.speed_gain = float(
            self.get_parameter('speed_error_lookahead_gain_s').value
        )
        self.minimum_lookahead = float(
            self.get_parameter('minimum_lookahead_m').value
        )
        self.maximum_lookahead = float(
            self.get_parameter('maximum_lookahead_m').value
        )
        self.lookahead_step = float(
            self.get_parameter('lookahead_step_m').value
        )
        self.collision_spacing = float(
            self.get_parameter('collision_check_spacing_m').value
        )
        self.completion_tolerance = float(
            self.get_parameter('completion_tolerance_m').value
        )
        self.completion_hold = float(
            self.get_parameter('completion_hold_seconds').value
        )
        self.stall_timeout = float(
            self.get_parameter('stall_timeout_seconds').value
        )
        self.minimum_progress = float(
            self.get_parameter('minimum_progress_m').value
        )
        self.status_period = float(
            self.get_parameter('status_period_seconds').value
        )
        self.accept_map_updates = bool(
            self.get_parameter('accept_map_updates').value
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/drone/goal',
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            '/cq01_v34d/follower_status',
            transient_qos,
        )

        self.create_subscription(
            Path,
            '/drone/planned_path_3d',
            self.path_callback,
            transient_qos,
        )
        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            String,
            '/drone/voxel_metadata',
            self.metadata_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            '/drone/voxel_cloud',
            self.cloud_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            '/cq01_v34d/follower_command',
            self.command_callback,
            10,
        )

        self.metadata: Optional[Dict[str, object]] = None
        self.pending_points: Optional[np.ndarray] = None
        self.occupied_grid: Optional[np.ndarray] = None
        self.occupied_count = 0
        self.map_ready = False

        self.path_points: List[Point3] = []
        self.cumulative: List[float] = []
        self.path_frame = 'map'
        self.path_active = False
        self.path_sequence = 0

        self.latest_position: Optional[Point3] = None
        self.latest_speed = 0.0
        self.progress_s = 0.0
        self.best_progress_s = 0.0
        self.last_progress_time: Optional[float] = None
        self.completion_started: Optional[float] = None
        self.last_status_time = 0.0
        self.last_target: Optional[Point3] = None

        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0),
            self.control_step,
        )
        self.publish_status('waiting_for_map_and_path')
        self.get_logger().info(
            'V3.4D fast lookahead follower started: '
            f'target_speed={self.target_speed:.2f} m/s, '
            f'lookahead={self.minimum_lookahead:.1f}..'
            f'{self.maximum_lookahead:.1f} m'
        )


    def command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command in {'stop', 'cancel', 'reset'}:
            self.path_active = False
            self.path_points = []
            self.cumulative = []
            self.last_target = None
            self.publish_status('stopped', command=command)

    @property
    def resolution(self) -> float:
        assert self.metadata is not None
        return float(self.metadata['resolution_m'])

    @property
    def bounds(self) -> Dict[str, float]:
        assert self.metadata is not None
        return {
            key: float(value)
            for key, value in self.metadata['bounds'].items()
        }

    @property
    def dimensions(self) -> Voxel:
        assert self.metadata is not None
        dimensions = self.metadata['dimensions']
        return (
            int(dimensions['x']),
            int(dimensions['y']),
            int(dimensions['z']),
        )

    def metadata_callback(self, message: String) -> None:
        if self.map_ready and not self.accept_map_updates:
            return
        self.metadata = json.loads(message.data)
        self.rebuild_occupancy()

    def cloud_callback(self, message: PointCloud2) -> None:
        if self.map_ready and not self.accept_map_updates:
            return
        self.pending_points = self.pointcloud_to_xyz(message)
        self.rebuild_occupancy()

    def rebuild_occupancy(self) -> None:
        if self.metadata is None or self.pending_points is None:
            return
        dimensions = self.dimensions
        grid = np.zeros(dimensions, dtype=np.bool_)
        if len(self.pending_points) > 0:
            bounds = self.bounds
            indices = np.floor((self.pending_points - np.array([
                bounds['min_x'], bounds['min_y'], bounds['min_z']
            ], dtype=np.float64)) / self.resolution).astype(np.int64)
            valid = (
                (indices[:, 0] >= 0)
                & (indices[:, 0] < dimensions[0])
                & (indices[:, 1] >= 0)
                & (indices[:, 1] < dimensions[1])
                & (indices[:, 2] >= 0)
                & (indices[:, 2] < dimensions[2])
            )
            valid_indices = indices[valid]
            grid[
                valid_indices[:, 0],
                valid_indices[:, 1],
                valid_indices[:, 2],
            ] = True
        self.occupied_grid = grid
        self.occupied_count = int(np.count_nonzero(grid))
        self.map_ready = True
        self.publish_status(
            'map_ready',
            occupied_voxels=self.occupied_count,
        )

    def pointcloud_to_xyz(self, message: PointCloud2) -> np.ndarray:
        point_count = int(message.width) * int(message.height)
        if point_count <= 0:
            return np.empty((0, 3), dtype=np.float64)
        offsets = {field.name: int(field.offset) for field in message.fields}
        if not {'x', 'y', 'z'}.issubset(offsets):
            raise RuntimeError('Voxel PointCloud2 does not contain x/y/z')
        endian = '>' if message.is_bigendian else '<'
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [f'{endian}f4', f'{endian}f4', f'{endian}f4'],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': int(message.point_step),
        })
        structured = np.frombuffer(
            message.data,
            dtype=dtype,
            count=point_count,
        )
        points = np.column_stack((
            structured['x'],
            structured['y'],
            structured['z'],
        )).astype(np.float64, copy=False)
        return points[np.isfinite(points).all(axis=1)]

    def world_to_voxel(self, point: Sequence[float]) -> Voxel:
        bounds = self.bounds
        return (
            int(math.floor((float(point[0]) - bounds['min_x']) / self.resolution)),
            int(math.floor((float(point[1]) - bounds['min_y']) / self.resolution)),
            int(math.floor((float(point[2]) - bounds['min_z']) / self.resolution)),
        )

    def index_in_bounds(self, voxel: Voxel) -> bool:
        dimensions = self.dimensions
        return (
            0 <= voxel[0] < dimensions[0]
            and 0 <= voxel[1] < dimensions[1]
            and 0 <= voxel[2] < dimensions[2]
        )

    def point_free(self, point: Point3) -> bool:
        if self.metadata is None:
            return False
        voxel = self.world_to_voxel(point)
        return (
            self.index_in_bounds(voxel)
            and self.occupied_grid is not None
            and not bool(self.occupied_grid[voxel])
        )

    def segment_free(self, first: Point3, second: Point3) -> bool:
        length = math.dist(first, second)
        if length <= 1.0e-6:
            return self.point_free(second)
        samples = max(2, int(math.ceil(length / self.collision_spacing)))
        # Skip the exact current position; it may lie close to a voxel face.
        for index in range(1, samples + 1):
            t = index / samples
            point = (
                first[0] + (second[0] - first[0]) * t,
                first[1] + (second[1] - first[1]) * t,
                first[2] + (second[2] - first[2]) * t,
            )
            if not self.point_free(point):
                return False
        return True

    def path_callback(self, message: Path) -> None:
        points: List[Point3] = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]
        if len(points) < 2:
            self.path_active = False
            self.publish_status('invalid_path', point_count=len(points))
            return

        cumulative = [0.0]
        for first, second in zip(points[:-1], points[1:]):
            cumulative.append(cumulative[-1] + math.dist(first, second))

        self.path_points = points
        self.cumulative = cumulative
        self.path_frame = message.header.frame_id or 'map'
        self.path_active = True
        self.path_sequence += 1
        self.progress_s = 0.0
        self.best_progress_s = 0.0
        now = time.monotonic()
        self.last_progress_time = now
        self.completion_started = None
        self.last_target = None
        self.publish_status(
            'path_accepted',
            path_sequence=self.path_sequence,
            path_points=len(points),
            path_length_m=cumulative[-1],
        )
        self.get_logger().info(
            f'Fast follower accepted path {self.path_sequence}: '
            f'points={len(points)}, length={cumulative[-1]:.2f} m'
        )

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.latest_position = (float(p.x), float(p.y), float(p.z))
        v = message.twist.twist.linear
        self.latest_speed = math.sqrt(
            float(v.x) ** 2 + float(v.y) ** 2 + float(v.z) ** 2
        )

    @staticmethod
    def interpolate(first: Point3, second: Point3, t: float) -> Point3:
        return (
            first[0] + (second[0] - first[0]) * t,
            first[1] + (second[1] - first[1]) * t,
            first[2] + (second[2] - first[2]) * t,
        )

    def project_to_path(self, position: Point3) -> Projection:
        best: Optional[Projection] = None
        for index, (first, second) in enumerate(
            zip(self.path_points[:-1], self.path_points[1:])
        ):
            vx = second[0] - first[0]
            vy = second[1] - first[1]
            vz = second[2] - first[2]
            length_sq = vx * vx + vy * vy + vz * vz
            if length_sq <= 1.0e-12:
                t = 0.0
            else:
                t = (
                    (position[0] - first[0]) * vx
                    + (position[1] - first[1]) * vy
                    + (position[2] - first[2]) * vz
                ) / length_sq
                t = max(0.0, min(1.0, t))
            point = self.interpolate(first, second, t)
            distance = math.dist(position, point)
            arc = self.cumulative[index] + t * math.sqrt(length_sq)
            candidate = Projection(index, t, point, arc, distance)
            if best is None or candidate.distance < best.distance:
                best = candidate
        assert best is not None
        return best

    def point_at_arc(self, arc: float) -> Point3:
        if arc <= 0.0:
            return self.path_points[0]
        total = self.cumulative[-1]
        if arc >= total:
            return self.path_points[-1]
        for index in range(len(self.cumulative) - 1):
            start = self.cumulative[index]
            end = self.cumulative[index + 1]
            if start <= arc <= end:
                length = end - start
                t = 0.0 if length <= 1.0e-9 else (arc - start) / length
                return self.interpolate(
                    self.path_points[index],
                    self.path_points[index + 1],
                    t,
                )
        return self.path_points[-1]

    def choose_target(self, position: Point3) -> Tuple[Optional[Point3], float]:
        remaining = max(0.0, self.cumulative[-1] - self.progress_s)
        desired = (
            self.base_lookahead
            + self.speed_gain * max(0.0, self.target_speed - self.latest_speed)
        )
        desired = min(self.maximum_lookahead, max(self.minimum_lookahead, desired))
        desired = min(desired, remaining)

        distance = desired
        while distance >= min(self.minimum_lookahead, remaining) - 1.0e-6:
            target = self.point_at_arc(self.progress_s + distance)
            if self.segment_free(position, target):
                return target, distance
            distance -= self.lookahead_step

        # Near the final point the remaining distance can be smaller than the
        # configured minimum lookahead.
        if remaining > 0.0:
            target = self.path_points[-1]
            if self.segment_free(position, target):
                return target, remaining
        return None, 0.0

    def publish_goal(self, target: Point3) -> None:
        message = PoseStamped()
        message.header.frame_id = self.path_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)
        self.last_target = target

    def control_step(self) -> None:
        if not (
            self.path_active
            and self.map_ready
            and self.latest_position is not None
            and len(self.path_points) >= 2
        ):
            return

        now = time.monotonic()
        projection = self.project_to_path(self.latest_position)
        # Do not let a temporary nearest-segment ambiguity move progress back.
        self.progress_s = max(self.progress_s, projection.arc_length)
        if self.progress_s >= self.best_progress_s + self.minimum_progress:
            self.best_progress_s = self.progress_s
            self.last_progress_time = now

        final = self.path_points[-1]
        final_distance = math.dist(self.latest_position, final)
        if final_distance <= self.completion_tolerance:
            if self.completion_started is None:
                self.completion_started = now
            elif now - self.completion_started >= self.completion_hold:
                self.path_active = False
                self.publish_status(
                    'goal_reached',
                    path_sequence=self.path_sequence,
                    final_error_m=final_distance,
                    path_length_m=self.cumulative[-1],
                )
                self.get_logger().info(
                    f'Fast follower reached path goal: error={final_distance:.3f} m'
                )
                return
        else:
            self.completion_started = None

        if (
            self.last_progress_time is not None
            and now - self.last_progress_time >= self.stall_timeout
        ):
            self.path_active = False
            self.publish_status(
                'stalled',
                path_sequence=self.path_sequence,
                progress_m=self.progress_s,
                path_length_m=self.cumulative[-1],
                final_error_m=final_distance,
                speed_mps=self.latest_speed,
            )
            self.get_logger().error(
                'Fast follower stalled: '
                f'progress={self.progress_s:.1f}/{self.cumulative[-1]:.1f} m, '
                f'speed={self.latest_speed:.2f} m/s'
            )
            return

        target, lookahead = self.choose_target(self.latest_position)
        if target is not None:
            self.publish_goal(target)

        if now - self.last_status_time >= self.status_period:
            self.last_status_time = now
            self.publish_status(
                'tracking',
                path_sequence=self.path_sequence,
                progress_m=self.progress_s,
                remaining_m=max(0.0, self.cumulative[-1] - self.progress_s),
                progress_ratio=(
                    self.progress_s / self.cumulative[-1]
                    if self.cumulative[-1] > 1.0e-9 else 1.0
                ),
                speed_mps=self.latest_speed,
                lookahead_m=lookahead,
                cross_track_error_m=projection.distance,
                final_error_m=final_distance,
            )

    def publish_status(self, state: str, **extra: object) -> None:
        payload: Dict[str, object] = {
            'state': state,
            'success': state not in {'invalid_path', 'stalled'},
            'path_active': self.path_active,
            'map_ready': self.map_ready,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
