#!/usr/bin/env python3
"""Build a filled and inflated 3D voxel map from drone_world geometry."""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from drone_world.geometry import (
    expand_obstacles,
    generate_random_obstacles,
    inverse_rotate_xy,
)


Voxel = Tuple[int, int, int]
Point3 = Tuple[float, float, float]


class VoxelMapServer(Node):
    """Publish base and inflated occupancy as 3D voxel clouds and markers."""

    def __init__(self) -> None:
        super().__init__("voxel_map_server")

        self.declare_parameter("scenario", "overflight_3d")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("resolution_m", 0.30)
        self.declare_parameter("inflation_radius_m", 0.45)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("marker_stride", 1)
        self.declare_parameter("slice_z_m", 1.50)
        self.declare_parameter(
            "raw_cloud_topic",
            "/drone/voxel_cloud_raw",
        )
        self.declare_parameter(
            "inflated_cloud_topic",
            "/drone/voxel_cloud",
        )
        self.declare_parameter(
            "marker_topic",
            "/drone/voxel_markers",
        )
        self.declare_parameter(
            "metadata_topic",
            "/drone/voxel_metadata",
        )

        self.scenario = str(
            self.get_parameter("scenario").value
        )
        self.resolution = float(
            self.get_parameter("resolution_m").value
        )
        self.inflation_radius = float(
            self.get_parameter("inflation_radius_m").value
        )
        self.marker_stride = max(
            1,
            int(self.get_parameter("marker_stride").value),
        )
        self.slice_z = float(
            self.get_parameter("slice_z_m").value
        )

        if self.resolution <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.inflation_radius < 0.0:
            raise ValueError(
                "inflation_radius_m must be non-negative"
            )

        configuration_path = self.resolve_configuration_path(
            str(self.get_parameter("scenario_file").value)
        )
        self.configuration = self.load_configuration(
            configuration_path
        )
        self.world = dict(self.configuration["world"])
        self.frame_id = str(
            self.world.get("frame_id", "map")
        )
        self.bounds = {
            key: float(value)
            for key, value in self.world["bounds"].items()
        }

        raw_obstacles = list(
            self.world.get("obstacles", [])
        )
        generated = generate_random_obstacles(
            dict(self.world.get("random", {}))
        )
        self.obstacles = [
            obstacle
            for obstacle in expand_obstacles(
                raw_obstacles + generated
            )
            if bool(obstacle.get("collidable", True))
        ]

        self.dimensions = self.compute_dimensions()
        self.base_occupied = self.voxelize_obstacles()
        self.inflated_occupied = self.inflate_voxels(
            self.base_occupied
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.raw_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter(
                    "raw_cloud_topic"
                ).value
            ),
            transient_qos,
        )
        self.inflated_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter(
                    "inflated_cloud_topic"
                ).value
            ),
            transient_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(
                self.get_parameter("marker_topic").value
            ),
            transient_qos,
        )
        self.metadata_publisher = self.create_publisher(
            String,
            str(
                self.get_parameter("metadata_topic").value
            ),
            transient_qos,
        )

        self.raw_cloud = self.build_cloud(
            self.base_occupied
        )
        self.inflated_cloud = self.build_cloud(
            self.inflated_occupied
        )
        self.markers = self.build_markers()
        self.metadata = self.build_metadata_message()

        publish_hz = max(
            0.2,
            float(self.get_parameter("publish_hz").value),
        )
        self.timer = self.create_timer(
            1.0 / publish_hz,
            self.publish_all,
        )
        self.publish_all()

        self.get_logger().info(
            "3D voxel map ready: "
            f"scenario={self.scenario}, "
            f"resolution={self.resolution:.2f} m, "
            f"base={len(self.base_occupied)}, "
            f"inflated={len(self.inflated_occupied)}"
        )
        self.get_logger().info(
            "Voxel dimensions: "
            f"{self.dimensions[0]} x "
            f"{self.dimensions[1]} x "
            f"{self.dimensions[2]}"
        )

    def resolve_configuration_path(
        self,
        explicit_path: str,
    ) -> Path:
        if explicit_path:
            return Path(explicit_path).expanduser()

        world_share = Path(
            get_package_share_directory("drone_world")
        )
        return (
            world_share
            / "config"
            / f"{self.scenario}.yaml"
        )

    def load_configuration(
        self,
        path: Path,
    ) -> Dict[str, object]:
        if not path.exists():
            raise FileNotFoundError(
                f"Scenario configuration not found: {path}"
            )

        with path.open("r", encoding="utf-8") as handle:
            configuration = yaml.safe_load(handle)

        if not isinstance(configuration, dict):
            raise ValueError(
                f"Invalid YAML structure: {path}"
            )
        if "world" not in configuration:
            raise ValueError(
                f"Missing world section: {path}"
            )

        self.get_logger().info(
            f"Loaded voxel scenario: {path}"
        )
        return configuration

    def compute_dimensions(self) -> Tuple[int, int, int]:
        return (
            int(
                math.ceil(
                    (
                        self.bounds["max_x"]
                        - self.bounds["min_x"]
                    )
                    / self.resolution
                )
            ),
            int(
                math.ceil(
                    (
                        self.bounds["max_y"]
                        - self.bounds["min_y"]
                    )
                    / self.resolution
                )
            ),
            int(
                math.ceil(
                    (
                        self.bounds["max_z"]
                        - self.bounds["min_z"]
                    )
                    / self.resolution
                )
            ),
        )

    def in_bounds(self, voxel: Voxel) -> bool:
        return (
            0 <= voxel[0] < self.dimensions[0]
            and 0 <= voxel[1] < self.dimensions[1]
            and 0 <= voxel[2] < self.dimensions[2]
        )

    def world_to_voxel(
        self,
        point: Sequence[float],
    ) -> Voxel:
        return (
            int(
                math.floor(
                    (
                        float(point[0])
                        - self.bounds["min_x"]
                    )
                    / self.resolution
                )
            ),
            int(
                math.floor(
                    (
                        float(point[1])
                        - self.bounds["min_y"]
                    )
                    / self.resolution
                )
            ),
            int(
                math.floor(
                    (
                        float(point[2])
                        - self.bounds["min_z"]
                    )
                    / self.resolution
                )
            ),
        )

    def voxel_to_world(
        self,
        voxel: Voxel,
    ) -> Point3:
        return (
            self.bounds["min_x"]
            + (voxel[0] + 0.5) * self.resolution,
            self.bounds["min_y"]
            + (voxel[1] + 0.5) * self.resolution,
            self.bounds["min_z"]
            + (voxel[2] + 0.5) * self.resolution,
        )

    def obstacle_aabb(
        self,
        obstacle: Dict[str, object],
    ) -> Tuple[Point3, Point3]:
        shape = str(
            obstacle.get(
                "shape",
                obstacle.get("type", "box"),
            )
        )
        center = [
            float(value)
            for value in obstacle["center"]
        ]

        if shape == "box":
            size = [
                float(value)
                for value in obstacle["size"]
            ]
            yaw = float(obstacle.get("yaw", 0.0))
            half_x = size[0] / 2.0
            half_y = size[1] / 2.0

            projected_half_x = (
                abs(math.cos(yaw)) * half_x
                + abs(math.sin(yaw)) * half_y
            )
            projected_half_y = (
                abs(math.sin(yaw)) * half_x
                + abs(math.cos(yaw)) * half_y
            )
            half_z = size[2] / 2.0

            minimum = (
                center[0] - projected_half_x,
                center[1] - projected_half_y,
                center[2] - half_z,
            )
            maximum = (
                center[0] + projected_half_x,
                center[1] + projected_half_y,
                center[2] + half_z,
            )
            return minimum, maximum

        if shape == "cylinder":
            radius = float(obstacle["radius"])
            half_z = float(obstacle["height"]) / 2.0
            minimum = (
                center[0] - radius,
                center[1] - radius,
                center[2] - half_z,
            )
            maximum = (
                center[0] + radius,
                center[1] + radius,
                center[2] + half_z,
            )
            return minimum, maximum

        if shape == "sphere":
            radius = float(obstacle["radius"])
            minimum = (
                center[0] - radius,
                center[1] - radius,
                center[2] - radius,
            )
            maximum = (
                center[0] + radius,
                center[1] + radius,
                center[2] + radius,
            )
            return minimum, maximum

        raise ValueError(
            f"Unsupported obstacle shape: {shape}"
        )

    def point_inside_obstacle(
        self,
        point: Point3,
        obstacle: Dict[str, object],
    ) -> bool:
        shape = str(
            obstacle.get(
                "shape",
                obstacle.get("type", "box"),
            )
        )
        center = [
            float(value)
            for value in obstacle["center"]
        ]

        if shape == "box":
            size = [
                float(value)
                for value in obstacle["size"]
            ]
            yaw = float(obstacle.get("yaw", 0.0))
            local_x, local_y = inverse_rotate_xy(
                point[0] - center[0],
                point[1] - center[1],
                yaw,
            )
            local_z = point[2] - center[2]

            return (
                abs(local_x) <= size[0] / 2.0
                and abs(local_y) <= size[1] / 2.0
                and abs(local_z) <= size[2] / 2.0
            )

        if shape == "cylinder":
            radius = float(obstacle["radius"])
            height = float(obstacle["height"])
            return (
                math.hypot(
                    point[0] - center[0],
                    point[1] - center[1],
                )
                <= radius
                and abs(point[2] - center[2])
                <= height / 2.0
            )

        if shape == "sphere":
            radius = float(obstacle["radius"])
            return (
                math.sqrt(
                    (point[0] - center[0]) ** 2
                    + (point[1] - center[1]) ** 2
                    + (point[2] - center[2]) ** 2
                )
                <= radius
            )

        raise ValueError(
            f"Unsupported obstacle shape: {shape}"
        )

    def index_range_for_axis(
        self,
        minimum: float,
        maximum: float,
        axis_minimum: float,
        dimension: int,
    ) -> range:
        start = max(
            0,
            int(
                math.floor(
                    (minimum - axis_minimum)
                    / self.resolution
                )
            ),
        )
        stop = min(
            dimension - 1,
            int(
                math.floor(
                    (maximum - axis_minimum)
                    / self.resolution
                )
            ),
        )
        return range(start, stop + 1)

    def voxelize_obstacles(self) -> Set[Voxel]:
        occupied: Set[Voxel] = set()

        for obstacle in self.obstacles:
            minimum, maximum = self.obstacle_aabb(
                obstacle
            )

            x_range = self.index_range_for_axis(
                minimum[0],
                maximum[0],
                self.bounds["min_x"],
                self.dimensions[0],
            )
            y_range = self.index_range_for_axis(
                minimum[1],
                maximum[1],
                self.bounds["min_y"],
                self.dimensions[1],
            )
            z_range = self.index_range_for_axis(
                minimum[2],
                maximum[2],
                self.bounds["min_z"],
                self.dimensions[2],
            )

            for ix in x_range:
                for iy in y_range:
                    for iz in z_range:
                        voxel = (ix, iy, iz)
                        point = self.voxel_to_world(
                            voxel
                        )
                        if self.point_inside_obstacle(
                            point,
                            obstacle,
                        ):
                            occupied.add(voxel)

        return occupied

    def inflation_offsets(self) -> List[Voxel]:
        if self.inflation_radius <= 0.0:
            return [(0, 0, 0)]

        radius_cells = int(
            math.ceil(
                self.inflation_radius
                / self.resolution
            )
        )
        offsets: List[Voxel] = []

        for dx in range(
            -radius_cells,
            radius_cells + 1,
        ):
            for dy in range(
                -radius_cells,
                radius_cells + 1,
            ):
                for dz in range(
                    -radius_cells,
                    radius_cells + 1,
                ):
                    distance = (
                        self.resolution
                        * math.sqrt(
                            dx * dx
                            + dy * dy
                            + dz * dz
                        )
                    )
                    if (
                        distance
                        <= self.inflation_radius + 1.0e-9
                    ):
                        offsets.append(
                            (dx, dy, dz)
                        )

        return offsets

    def inflate_voxels(
        self,
        base: Set[Voxel],
    ) -> Set[Voxel]:
        output: Set[Voxel] = set(base)
        offsets = self.inflation_offsets()

        for voxel in base:
            for offset in offsets:
                candidate = (
                    voxel[0] + offset[0],
                    voxel[1] + offset[1],
                    voxel[2] + offset[2],
                )
                if self.in_bounds(candidate):
                    output.add(candidate)

        return output

    def build_cloud(
        self,
        voxels: Iterable[Voxel],
    ) -> PointCloud2:
        ordered = sorted(voxels)
        points = [
            self.voxel_to_world(voxel)
            for voxel in ordered
        ]

        message = PointCloud2()
        message.header = Header(
            frame_id=self.frame_id
        )
        message.height = 1
        message.width = len(points)
        message.is_bigendian = False
        message.is_dense = True
        message.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = b"".join(
            struct.pack("<fff", *point)
            for point in points
        )
        return message

    def build_markers(self) -> MarkerArray:
        markers = MarkerArray()

        raw_marker = Marker()
        raw_marker.header.frame_id = self.frame_id
        raw_marker.ns = "voxel_raw"
        raw_marker.id = 0
        raw_marker.type = Marker.CUBE_LIST
        raw_marker.action = Marker.ADD
        raw_marker.pose.orientation.w = 1.0
        raw_marker.scale.x = self.resolution * 0.92
        raw_marker.scale.y = self.resolution * 0.92
        raw_marker.scale.z = self.resolution * 0.92
        raw_marker.color.r = 0.82
        raw_marker.color.g = 0.22
        raw_marker.color.b = 0.16
        raw_marker.color.a = 0.70

        for index, voxel in enumerate(
            sorted(self.base_occupied)
        ):
            if index % self.marker_stride != 0:
                continue
            point = Point()
            point.x, point.y, point.z = (
                self.voxel_to_world(voxel)
            )
            raw_marker.points.append(point)

        inflated_shell = (
            self.inflated_occupied
            - self.base_occupied
        )
        inflated_marker = Marker()
        inflated_marker.header.frame_id = self.frame_id
        inflated_marker.ns = "voxel_inflated_shell"
        inflated_marker.id = 1
        inflated_marker.type = Marker.CUBE_LIST
        inflated_marker.action = Marker.ADD
        inflated_marker.pose.orientation.w = 1.0
        inflated_marker.scale.x = self.resolution * 0.86
        inflated_marker.scale.y = self.resolution * 0.86
        inflated_marker.scale.z = self.resolution * 0.86
        inflated_marker.color.r = 1.00
        inflated_marker.color.g = 0.72
        inflated_marker.color.b = 0.10
        inflated_marker.color.a = 0.22

        for index, voxel in enumerate(
            sorted(inflated_shell)
        ):
            if index % self.marker_stride != 0:
                continue
            point = Point()
            point.x, point.y, point.z = (
                self.voxel_to_world(voxel)
            )
            inflated_marker.points.append(point)

        slice_marker = Marker()
        slice_marker.header.frame_id = self.frame_id
        slice_marker.ns = "voxel_slice"
        slice_marker.id = 2
        slice_marker.type = Marker.CUBE_LIST
        slice_marker.action = Marker.ADD
        slice_marker.pose.orientation.w = 1.0
        slice_marker.scale.x = self.resolution * 0.96
        slice_marker.scale.y = self.resolution * 0.96
        slice_marker.scale.z = 0.035
        slice_marker.color.r = 0.10
        slice_marker.color.g = 0.45
        slice_marker.color.b = 0.95
        slice_marker.color.a = 0.65

        slice_index = int(
            math.floor(
                (
                    self.slice_z
                    - self.bounds["min_z"]
                )
                / self.resolution
            )
        )
        for voxel in sorted(self.inflated_occupied):
            if voxel[2] != slice_index:
                continue
            point = Point()
            point.x, point.y, _ = (
                self.voxel_to_world(voxel)
            )
            point.z = self.slice_z
            slice_marker.points.append(point)

        markers.markers.extend([
            inflated_marker,
            raw_marker,
            slice_marker,
        ])
        return markers

    def build_metadata_message(self) -> String:
        data = {
            "scenario": self.scenario,
            "frame_id": self.frame_id,
            "resolution_m": self.resolution,
            "inflation_radius_m": self.inflation_radius,
            "bounds": self.bounds,
            "dimensions": {
                "x": self.dimensions[0],
                "y": self.dimensions[1],
                "z": self.dimensions[2],
            },
            "base_occupied_voxels": len(
                self.base_occupied
            ),
            "inflated_occupied_voxels": len(
                self.inflated_occupied
            ),
            "free_voxels": (
                self.dimensions[0]
                * self.dimensions[1]
                * self.dimensions[2]
                - len(self.inflated_occupied)
            ),
            "slice_z_m": self.slice_z,
        }
        message = String()
        message.data = json.dumps(
            data,
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.raw_cloud.header.stamp = stamp
        self.inflated_cloud.header.stamp = stamp

        for marker in self.markers.markers:
            marker.header.stamp = stamp

        self.raw_cloud_publisher.publish(
            self.raw_cloud
        )
        self.inflated_cloud_publisher.publish(
            self.inflated_cloud
        )
        self.marker_publisher.publish(
            self.markers
        )
        self.metadata_publisher.publish(
            self.metadata
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoxelMapServer()

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
