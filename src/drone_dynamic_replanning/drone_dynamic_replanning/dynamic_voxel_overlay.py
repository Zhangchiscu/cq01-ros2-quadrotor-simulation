#!/usr/bin/env python3
"""Merge static inflated voxels with predicted dynamic-obstacle voxels."""

from __future__ import annotations

import json
import math
import struct
from typing import Dict, Iterable, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String


Voxel = Tuple[int, int, int]
Point3 = Tuple[float, float, float]


class DynamicVoxelOverlay(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_voxel_overlay")

        self.declare_parameter(
            "static_cloud_topic",
            "/drone/voxel_cloud",
        )
        self.declare_parameter(
            "combined_cloud_topic",
            "/drone/voxel_cloud_dynamic",
        )
        self.declare_parameter(
            "dynamic_cloud_topic",
            "/drone/dynamic_voxel_cloud",
        )
        self.declare_parameter(
            "planning_inflation_radius_m",
            0.70,
        )
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter(
            "start_escape_radius_m",
            0.90,
        )
        self.declare_parameter(
            "planning_prediction_horizon_s",
            2.50,
        )

        self.planning_inflation = float(
            self.get_parameter(
                "planning_inflation_radius_m"
            ).value
        )
        publish_hz = float(
            self.get_parameter("publish_hz").value
        )
        self.start_escape_radius = float(
            self.get_parameter(
                "start_escape_radius_m"
            ).value
        )
        self.planning_prediction_horizon = float(
            self.get_parameter(
                "planning_prediction_horizon_s"
            ).value
        )

        self.metadata: Optional[Dict[str, object]] = None
        self.static_points: Optional[np.ndarray] = None
        self.static_voxels: Set[Voxel] = set()
        self.dynamic_state: Optional[Dict[str, object]] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_dynamic_voxels: Set[Voxel] = set()
        self.last_current_voxels: Set[Voxel] = set()
        self.last_future_voxels: Set[Voxel] = set()
        self.last_carved_voxels: Set[Voxel] = set()

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.combined_publisher = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter(
                    "combined_cloud_topic"
                ).value
            ),
            transient_qos,
        )
        self.dynamic_publisher = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter(
                    "dynamic_cloud_topic"
                ).value
            ),
            transient_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/drone/dynamic_map/status",
            transient_qos,
        )

        self.create_subscription(
            PointCloud2,
            str(
                self.get_parameter(
                    "static_cloud_topic"
                ).value
            ),
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
            Odometry,
            "/drone/odom",
            self.odom_callback,
            20,
        )

        self.timer = self.create_timer(
            1.0 / publish_hz,
            self.publish_overlay,
        )

        self.get_logger().info(
            "Dynamic voxel overlay started"
        )

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

    def metadata_callback(self, message: String) -> None:
        self.metadata = json.loads(message.data)
        self.rebuild_static_voxels()

    def static_cloud_callback(
        self,
        message: PointCloud2,
    ) -> None:
        self.static_points = self.pointcloud_to_xyz(
            message
        )
        self.rebuild_static_voxels()

    def dynamic_state_callback(
        self,
        message: String,
    ) -> None:
        self.dynamic_state = json.loads(message.data)

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.latest_odom = message

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
        required = {"x", "y", "z"}
        if not required.issubset(offsets):
            raise RuntimeError(
                "PointCloud2 does not contain x/y/z"
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

    def world_to_voxel(
        self,
        point: Sequence[float],
    ) -> Voxel:
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

    def voxel_to_world(
        self,
        voxel: Voxel,
    ) -> Point3:
        bounds = self.bounds
        return (
            bounds["min_x"]
            + (voxel[0] + 0.5) * self.resolution,
            bounds["min_y"]
            + (voxel[1] + 0.5) * self.resolution,
            bounds["min_z"]
            + (voxel[2] + 0.5) * self.resolution,
        )

    def in_bounds(self, voxel: Voxel) -> bool:
        dimensions = self.dimensions
        return (
            0 <= voxel[0] < dimensions[0]
            and 0 <= voxel[1] < dimensions[1]
            and 0 <= voxel[2] < dimensions[2]
        )

    def rebuild_static_voxels(self) -> None:
        if self.metadata is None or self.static_points is None:
            return

        voxels: Set[Voxel] = set()
        for point in self.static_points:
            voxel = self.world_to_voxel(point)
            if self.in_bounds(voxel):
                voxels.add(voxel)
        self.static_voxels = voxels

    def cylinder_voxels(
        self,
        center: Sequence[float],
        radius: float,
        height: float,
    ) -> Set[Voxel]:
        output: Set[Voxel] = set()

        inflated_radius = (
            radius + self.planning_inflation
        )
        inflated_half_height = (
            0.5 * height + self.planning_inflation
        )

        minimum = (
            float(center[0]) - inflated_radius,
            float(center[1]) - inflated_radius,
            float(center[2]) - inflated_half_height,
        )
        maximum = (
            float(center[0]) + inflated_radius,
            float(center[1]) + inflated_radius,
            float(center[2]) + inflated_half_height,
        )

        minimum_index = self.world_to_voxel(minimum)
        maximum_index = self.world_to_voxel(maximum)

        for ix in range(
            max(0, minimum_index[0]),
            min(
                self.dimensions[0] - 1,
                maximum_index[0],
            ) + 1,
        ):
            for iy in range(
                max(0, minimum_index[1]),
                min(
                    self.dimensions[1] - 1,
                    maximum_index[1],
                ) + 1,
            ):
                for iz in range(
                    max(0, minimum_index[2]),
                    min(
                        self.dimensions[2] - 1,
                        maximum_index[2],
                    ) + 1,
                ):
                    voxel = (ix, iy, iz)
                    point = self.voxel_to_world(voxel)
                    horizontal = math.hypot(
                        point[0] - float(center[0]),
                        point[1] - float(center[1]),
                    )
                    vertical = abs(
                        point[2] - float(center[2])
                    )
                    if (
                        horizontal <= inflated_radius
                        and vertical <= inflated_half_height
                    ):
                        output.add(voxel)

        return output

    def current_drone_position(
        self,
    ) -> Optional[Point3]:
        if self.latest_odom is None:
            return None

        position = self.latest_odom.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def within_start_escape_bubble(
        self,
        voxel: Voxel,
        drone_position: Point3,
    ) -> bool:
        point = self.voxel_to_world(voxel)
        return (
            math.dist(point, drone_position)
            <= self.start_escape_radius
        )

    def build_dynamic_voxels(self) -> Set[Voxel]:
        if self.dynamic_state is None:
            return set()

        radius = float(
            self.dynamic_state["radius_m"]
        )
        height = float(
            self.dynamic_state["height_m"]
        )
        current_position = self.dynamic_state["position"]

        current_voxels = self.cylinder_voxels(
            current_position,
            radius,
            height,
        )

        predictions = self.dynamic_state.get(
            "predicted_positions",
            [],
        )
        future_voxels: Set[Voxel] = set()

        for prediction in predictions:
            prediction_time = float(
                prediction.get("t_s", 0.0)
            )
            if prediction_time <= 1.0e-9:
                continue
            if (
                prediction_time
                > self.planning_prediction_horizon
            ):
                continue

            future_voxels.update(
                self.cylinder_voxels(
                    prediction["position"],
                    radius,
                    height,
                )
            )

        drone_position = self.current_drone_position()
        carved_voxels: Set[Voxel] = set()

        if drone_position is not None:
            for voxel in (
                future_voxels - current_voxels
            ):
                if self.within_start_escape_bubble(
                    voxel,
                    drone_position,
                ):
                    carved_voxels.add(voxel)

        safe_future_voxels = (
            future_voxels - carved_voxels
        )
        output = (
            current_voxels | safe_future_voxels
        )

        self.last_current_voxels = current_voxels
        self.last_future_voxels = future_voxels
        self.last_carved_voxels = carved_voxels
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
        message.header = Header(frame_id="map")
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

    def publish_overlay(self) -> None:
        if (
            self.metadata is None
            or not self.static_voxels
            or self.dynamic_state is None
        ):
            return

        dynamic_voxels = self.build_dynamic_voxels()
        combined_voxels = (
            self.static_voxels | dynamic_voxels
        )

        dynamic_cloud = self.build_cloud(dynamic_voxels)
        combined_cloud = self.build_cloud(combined_voxels)
        stamp = self.get_clock().now().to_msg()
        dynamic_cloud.header.stamp = stamp
        combined_cloud.header.stamp = stamp

        self.dynamic_publisher.publish(dynamic_cloud)
        self.combined_publisher.publish(combined_cloud)
        self.last_dynamic_voxels = dynamic_voxels

        status = String()
        status.data = json.dumps({
            "state": "ready",
            "success": True,
            "phase": self.dynamic_state["phase"],
            "prediction_active": bool(
                self.dynamic_state["prediction_active"]
            ),
            "static_voxels": len(self.static_voxels),
            "dynamic_voxels": len(dynamic_voxels),
            "current_dynamic_voxels": len(
                self.last_current_voxels
            ),
            "future_swept_voxels": len(
                self.last_future_voxels
            ),
            "start_escape_carved_voxels": len(
                self.last_carved_voxels
            ),
            "start_escape_radius_m": (
                self.start_escape_radius
            ),
            "planning_prediction_horizon_s": (
                self.planning_prediction_horizon
            ),
            "combined_voxels": len(combined_voxels),
            "planning_inflation_radius_m": (
                self.planning_inflation
            ),
        }, sort_keys=True)
        self.status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicVoxelOverlay()

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
