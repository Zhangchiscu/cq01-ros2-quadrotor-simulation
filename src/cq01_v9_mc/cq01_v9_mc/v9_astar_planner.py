#!/usr/bin/env python3
"""26-neighbour 3D A* planner with waypoint execution."""

from __future__ import annotations

import heapq
import json
import math
import time
from collections import deque
from itertools import count
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

Voxel = Tuple[int, int, int]
Point3 = Tuple[float, float, float]


class Planner3D(Node):
    def __init__(self) -> None:
        super().__init__("astar_3d_planner")

        self.declare_parameter("minimum_flight_z_m", 0.35)
        self.declare_parameter("maximum_flight_z_m", 5.20)
        self.declare_parameter("vertical_cost_factor", 1.18)
        self.declare_parameter("waypoint_tolerance_m", 0.28)
        self.declare_parameter("waypoint_hold_seconds", 0.15)
        self.declare_parameter("maximum_waypoint_spacing_m", 0.65)
        self.declare_parameter("line_of_sight_step_m", 0.08)
        self.declare_parameter("fixed_height_test_z_m", 1.50)
        self.declare_parameter(
            "dynamic_voxel_topic",
            "/cq01_v9/dynamic/voxel_cloud",
        )

        self.minimum_flight_z = float(
            self.get_parameter("minimum_flight_z_m").value
        )
        self.maximum_flight_z = float(
            self.get_parameter("maximum_flight_z_m").value
        )
        self.vertical_cost_factor = float(
            self.get_parameter("vertical_cost_factor").value
        )
        self.waypoint_tolerance = float(
            self.get_parameter("waypoint_tolerance_m").value
        )
        self.waypoint_hold_seconds = float(
            self.get_parameter("waypoint_hold_seconds").value
        )
        self.maximum_waypoint_spacing = float(
            self.get_parameter("maximum_waypoint_spacing_m").value
        )
        self.line_of_sight_step = float(
            self.get_parameter("line_of_sight_step_m").value
        )
        self.fixed_height_test_z = float(
            self.get_parameter("fixed_height_test_z_m").value
        )
        self.dynamic_voxel_topic = str(
            self.get_parameter("dynamic_voxel_topic").value
        )

        self.metadata: Optional[Dict[str, object]] = None
        self.pending_points: Optional[np.ndarray] = None
        self.occupied: Set[Voxel] = set()
        self.dynamic_pending_points: Optional[np.ndarray] = None
        self.dynamic_occupied: Set[Voxel] = set()
        self.dynamic_update_count = 0
        self.latest_odom: Optional[Odometry] = None
        self.active_waypoints: List[Point3] = []
        self.active_index = 0
        self.hold_started: Optional[float] = None
        self.ready_announced = False

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.control_goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal",
            10,
        )
        self.path_publisher = self.create_publisher(
            Path,
            "/drone/planned_path_3d",
            transient_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/drone/planner_3d/status",
            transient_qos,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            50,
        )
        self.create_subscription(
            PoseStamped,
            "/drone/goal_raw",
            self.goal_callback,
            10,
        )
        self.create_subscription(
            String,
            "/drone/voxel_metadata",
            self.metadata_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            "/drone/voxel_cloud",
            self.cloud_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.dynamic_voxel_topic,
            self.dynamic_cloud_callback,
            transient_qos,
        )

        self.neighbours = self.build_neighbours()
        self.timer = self.create_timer(0.05, self.execution_step)

        self.get_logger().info(
            "26-neighbour 3D A* planner started"
        )

    def build_neighbours(
        self,
    ) -> List[Tuple[int, int, int, float]]:
        neighbours: List[Tuple[int, int, int, float]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    horizontal = math.hypot(dx, dy)
                    cost = math.sqrt(
                        horizontal * horizontal
                        + (self.vertical_cost_factor * dz) ** 2
                    )
                    neighbours.append((dx, dy, dz, cost))
        return neighbours

    def metadata_callback(self, message: String) -> None:
        self.metadata = json.loads(message.data)
        self.rebuild_occupancy()

    def cloud_callback(self, message: PointCloud2) -> None:
        self.pending_points = self.pointcloud_to_xyz(message)
        self.rebuild_occupancy()

    def dynamic_cloud_callback(
        self,
        message: PointCloud2,
    ) -> None:
        self.dynamic_pending_points = self.pointcloud_to_xyz(message)
        self.rebuild_dynamic_occupancy()

    def rebuild_dynamic_occupancy(self) -> None:
        if (
            self.metadata is None
            or self.dynamic_pending_points is None
        ):
            return

        dynamic: Set[Voxel] = set()
        for point in self.dynamic_pending_points:
            voxel = self.world_to_voxel(point)
            if self.index_in_bounds(voxel):
                dynamic.add(voxel)

        self.dynamic_occupied = dynamic
        self.dynamic_update_count += 1

        # Do not replace the planner's terminal status with a high-rate
        # dynamic-map message. Logging is enough; goal acknowledgements
        # remain ready/path_ready/final_waypoint_reached.
        if self.dynamic_update_count == 1 or (
            self.dynamic_update_count % 10 == 0
        ):
            self.get_logger().info(
                "Dynamic occupancy updated: "
                f"{len(self.dynamic_occupied)} voxels, "
                f"updates={self.dynamic_update_count}"
            )

    def rebuild_occupancy(self) -> None:
        if self.metadata is None or self.pending_points is None:
            return

        occupied: Set[Voxel] = set()
        for point in self.pending_points:
            voxel = self.world_to_voxel(point)
            if self.index_in_bounds(voxel):
                occupied.add(voxel)

        self.occupied = occupied
        self.rebuild_dynamic_occupancy()

        if not self.ready_announced:
            self.ready_announced = True
            self.publish_status({
                "state": "ready",
                "success": True,
                "occupied_voxels": len(self.occupied),
            })
            self.get_logger().info(
                f"Inflated 3D occupancy loaded: "
                f"{len(self.occupied)} voxels"
            )

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

    def voxel_to_world(self, voxel: Voxel) -> Point3:
        bounds = self.bounds
        return (
            bounds["min_x"]
            + (voxel[0] + 0.5) * self.resolution,
            bounds["min_y"]
            + (voxel[1] + 0.5) * self.resolution,
            bounds["min_z"]
            + (voxel[2] + 0.5) * self.resolution,
        )

    def index_in_bounds(self, voxel: Voxel) -> bool:
        dimensions = self.dimensions
        return (
            0 <= voxel[0] < dimensions[0]
            and 0 <= voxel[1] < dimensions[1]
            and 0 <= voxel[2] < dimensions[2]
        )

    def free(self, voxel: Voxel) -> bool:
        if not self.index_in_bounds(voxel):
            return False
        z = self.voxel_to_world(voxel)[2]
        if not (
            self.minimum_flight_z
            <= z
            <= self.maximum_flight_z
        ):
            return False
        return (
            voxel not in self.occupied
            and voxel not in self.dynamic_occupied
        )

    def nearest_free(
        self,
        seed: Voxel,
        maximum_radius: int = 6,
    ) -> Optional[Voxel]:
        if self.free(seed):
            return seed

        for radius in range(1, maximum_radius + 1):
            candidates: List[Voxel] = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != radius:
                            continue
                        candidate = (
                            seed[0] + dx,
                            seed[1] + dy,
                            seed[2] + dz,
                        )
                        if self.free(candidate):
                            candidates.append(candidate)

            if candidates:
                return min(
                    candidates,
                    key=lambda item: (
                        (item[0] - seed[0]) ** 2
                        + (item[1] - seed[1]) ** 2
                        + (item[2] - seed[2]) ** 2
                    ),
                )
        return None

    def heuristic(self, first: Voxel, second: Voxel) -> float:
        dx = first[0] - second[0]
        dy = first[1] - second[1]
        dz = first[2] - second[2]
        return math.sqrt(
            dx * dx
            + dy * dy
            + (self.vertical_cost_factor * dz) ** 2
        )

    def astar(
        self,
        start: Voxel,
        goal: Voxel,
    ) -> Optional[List[Voxel]]:
        queue = []
        sequence = count()
        heapq.heappush(
            queue,
            (
                self.heuristic(start, goal),
                next(sequence),
                start,
            ),
        )

        came_from: Dict[Voxel, Voxel] = {}
        g_score: Dict[Voxel, float] = {start: 0.0}
        closed: Set[Voxel] = set()

        while queue:
            _, _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                return self.reconstruct(came_from, current)

            closed.add(current)

            for dx, dy, dz, movement_cost in self.neighbours:
                neighbour = (
                    current[0] + dx,
                    current[1] + dy,
                    current[2] + dz,
                )
                if neighbour in closed or not self.free(neighbour):
                    continue

                tentative = g_score[current] + movement_cost
                if tentative >= g_score.get(
                    neighbour,
                    float("inf"),
                ):
                    continue

                came_from[neighbour] = current
                g_score[neighbour] = tentative
                priority = tentative + self.heuristic(
                    neighbour,
                    goal,
                )
                heapq.heappush(
                    queue,
                    (
                        priority,
                        next(sequence),
                        neighbour,
                    ),
                )

        return None

    def reconstruct(
        self,
        came_from: Dict[Voxel, Voxel],
        current: Voxel,
    ) -> List[Voxel]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def line_of_sight(
        self,
        first: Point3,
        second: Point3,
    ) -> bool:
        segment_length = math.dist(first, second)
        sample_count = max(
            1,
            int(math.ceil(
                segment_length / self.line_of_sight_step
            )),
        )
        for index in range(sample_count + 1):
            ratio = index / sample_count
            point = (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
                first[2] + ratio * (second[2] - first[2]),
            )
            if not self.free(self.world_to_voxel(point)):
                return False
        return True

    def simplify_path(self, points: List[Point3]) -> List[Point3]:
        if len(points) <= 2:
            return points

        simplified = [points[0]]
        anchor = 0

        while anchor < len(points) - 1:
            furthest = anchor + 1
            for candidate in range(anchor + 2, len(points)):
                if self.line_of_sight(
                    points[anchor],
                    points[candidate],
                ):
                    furthest = candidate
                else:
                    break
            simplified.append(points[furthest])
            anchor = furthest

        return simplified

    def densify(self, points: List[Point3]) -> List[Point3]:
        if len(points) <= 1:
            return points

        output = [points[0]]
        for first, second in zip(points[:-1], points[1:]):
            segment_length = math.dist(first, second)
            segment_count = max(
                1,
                int(math.ceil(
                    segment_length
                    / self.maximum_waypoint_spacing
                )),
            )
            for index in range(1, segment_count + 1):
                ratio = index / segment_count
                output.append((
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                    first[2] + ratio * (second[2] - first[2]),
                ))
        return output

    def nearest_free_xy(
        self,
        seed: Voxel,
        maximum_radius: int = 4,
    ) -> Optional[Voxel]:
        if self.free(seed):
            return seed

        for radius in range(1, maximum_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (
                        seed[0] + dx,
                        seed[1] + dy,
                        seed[2],
                    )
                    if self.free(candidate):
                        return candidate
        return None

    def fixed_height_path_exists(
        self,
        start: Voxel,
        goal: Voxel,
    ) -> bool:
        test_index = self.world_to_voxel(
            (0.0, 0.0, self.fixed_height_test_z)
        )[2]
        fixed_start = self.nearest_free_xy(
            (start[0], start[1], test_index)
        )
        fixed_goal = self.nearest_free_xy(
            (goal[0], goal[1], test_index)
        )

        if fixed_start is None or fixed_goal is None:
            return False

        queue = deque([fixed_start])
        visited = {fixed_start}
        neighbours_2d = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        while queue:
            current = queue.popleft()
            if current == fixed_goal:
                return True

            for dx, dy in neighbours_2d:
                candidate = (
                    current[0] + dx,
                    current[1] + dy,
                    test_index,
                )
                if candidate in visited or not self.free(candidate):
                    continue
                visited.add(candidate)
                queue.append(candidate)

        return False

    def goal_callback(self, message: PoseStamped) -> None:
        if (
            self.metadata is None
            or not self.occupied
            or self.latest_odom is None
        ):
            self.publish_failure("planner_not_ready")
            return

        current = self.latest_odom.pose.pose.position
        start_point: Point3 = (
            float(current.x),
            float(current.y),
            max(float(current.z), self.minimum_flight_z),
        )
        goal_point: Point3 = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )

        start_voxel = self.nearest_free(
            self.world_to_voxel(start_point)
        )
        goal_voxel = self.nearest_free(
            self.world_to_voxel(goal_point)
        )

        if start_voxel is None:
            self.publish_failure("no_free_start_voxel")
            return
        if goal_voxel is None:
            self.publish_failure("no_free_goal_voxel")
            return

        planning_started = time.perf_counter()
        voxel_path = self.astar(start_voxel, goal_voxel)
        planning_time_ms = (
            time.perf_counter() - planning_started
        ) * 1000.0

        if voxel_path is None:
            self.publish_failure(
                "astar_no_path",
                planning_time_ms,
            )
            return

        raw_points = [
            self.voxel_to_world(voxel)
            for voxel in voxel_path
        ]
        raw_points[0] = start_point
        raw_points[-1] = goal_point

        simplified = self.simplify_path(raw_points)
        waypoints = self.densify(simplified)

        self.active_waypoints = waypoints[1:]
        self.active_index = 0
        self.hold_started = None

        fixed_height_exists = self.fixed_height_path_exists(
            start_voxel,
            goal_voxel,
        )

        self.publish_path(waypoints)
        self.publish_status({
            "state": "path_ready",
            "success": True,
            "planning_time_ms": planning_time_ms,
            "raw_voxel_path_nodes": len(voxel_path),
            "simplified_path_nodes": len(simplified),
            "execution_waypoints": len(self.active_waypoints),
            "planned_path_length_m": self.path_length(waypoints),
            "fixed_height_test_z_m": self.fixed_height_test_z,
            "fixed_height_path_exists": fixed_height_exists,
            "start": list(start_point),
            "goal": list(goal_point),
        })

        self.get_logger().info(
            "3D A* path ready: "
            f"raw={len(voxel_path)}, "
            f"simplified={len(simplified)}, "
            f"waypoints={len(self.active_waypoints)}, "
            f"time={planning_time_ms:.2f} ms, "
            f"fixed-height path={fixed_height_exists}"
        )

    def publish_failure(
        self,
        reason: str,
        planning_time_ms: float = 0.0,
    ) -> None:
        self.active_waypoints = []
        self.publish_status({
            "state": "planning_failed",
            "success": False,
            "reason": reason,
            "planning_time_ms": planning_time_ms,
        })
        self.get_logger().error(
            f"3D planning failed: {reason}"
        )

    def publish_status(self, data: Dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(
            data,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def publish_path(self, points: List[Point3]) -> None:
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()

        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.path_publisher.publish(message)

    def path_length(self, points: List[Point3]) -> float:
        return sum(
            math.dist(first, second)
            for first, second in zip(points[:-1], points[1:])
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def execution_step(self) -> None:
        if (
            self.latest_odom is None
            or not self.active_waypoints
            or self.active_index >= len(self.active_waypoints)
        ):
            return

        target = self.active_waypoints[self.active_index]
        position = self.latest_odom.pose.pose.position
        current = (
            float(position.x),
            float(position.y),
            float(position.z),
        )
        error = math.dist(current, target)
        now = time.monotonic()

        if error <= self.waypoint_tolerance:
            if self.hold_started is None:
                self.hold_started = now
            elif (
                now - self.hold_started
                >= self.waypoint_hold_seconds
            ):
                self.active_index += 1
                self.hold_started = None
                if self.active_index >= len(self.active_waypoints):
                    self.publish_status({
                        "state": "final_waypoint_reached",
                        "success": True,
                    })
                    return
        else:
            self.hold_started = None

        if self.active_index < len(self.active_waypoints):
            self.publish_control_goal(
                self.active_waypoints[self.active_index]
            )

    def publish_control_goal(self, target: Point3) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.w = 1.0
        self.control_goal_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Planner3D()
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
