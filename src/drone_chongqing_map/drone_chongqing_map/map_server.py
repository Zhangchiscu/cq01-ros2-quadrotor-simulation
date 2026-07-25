#!/usr/bin/env python3
"""Publish real Yuzhong terrain, buildings and planner-compatible 3D occupancy."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np
import rclpy
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
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE_NAME = "drone_chongqing_map"


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def point_message(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def color_message(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


def cloud_message(points: np.ndarray, frame_id: str) -> PointCloud2:
    values = np.asarray(points, dtype="<f4")
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("Point cloud must have shape (N, 3)")

    message = PointCloud2()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = int(values.shape[0])
    message.is_bigendian = False
    message.is_dense = True
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.data = values.tobytes(order="C")
    return message


class ChongqingMapServer(Node):
    """Load preprocessed real geographic assets and publish ROS2 map products."""

    def __init__(self) -> None:
        super().__init__("chongqing_yuzhong_map_server")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_hz", 0.25)
        self.declare_parameter("terrain_alpha", 0.94)
        self.declare_parameter("building_alpha", 0.92)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_hz = max(0.05, float(self.get_parameter("publish_hz").value))
        self.terrain_alpha = float(self.get_parameter("terrain_alpha").value)
        self.building_alpha = float(self.get_parameter("building_alpha").value)

        share = Path(get_package_share_directory(PACKAGE_NAME))
        data_path = share / "runtime_data" / "chongqing_map_data.npz"
        manifest_path = share / "runtime_data" / "chongqing_map_manifest.json"

        if not data_path.exists() or not manifest_path.exists():
            raise FileNotFoundError(
                f"Missing packaged Chongqing runtime assets below {share}"
            )

        self.manifest: Dict[str, object] = json.loads(
            manifest_path.read_text(encoding="utf-8")
        )
        arrays = np.load(data_path, allow_pickle=False)
        self.terrain_vertices = arrays["terrain_vertices"]
        self.building_vertices = arrays["building_vertices"]
        self.raw_voxel_points = arrays["raw_voxel_points"]
        self.inflated_voxel_points = arrays["inflated_voxel_points"]
        self.roads_segments = arrays["roads_segments"]
        self.railways_segments = arrays["railways_segments"]
        self.bridges_segments = arrays["bridges_segments"]
        self.roi_ring = arrays["roi_ring"]

        qos = transient_qos()
        topics = self.manifest["topics"]

        self.terrain_publisher = self.create_publisher(
            MarkerArray, str(topics["terrain_markers"]), qos
        )
        self.building_publisher = self.create_publisher(
            MarkerArray, str(topics["building_markers"]), qos
        )
        self.context_publisher = self.create_publisher(
            MarkerArray, str(topics["context_markers"]), qos
        )
        self.raw_cloud_publisher = self.create_publisher(
            PointCloud2, str(topics["raw_voxel_cloud"]), qos
        )
        self.inflated_cloud_publisher = self.create_publisher(
            PointCloud2, str(topics["inflated_voxel_cloud"]), qos
        )
        self.metadata_publisher = self.create_publisher(
            String, str(topics["voxel_metadata"]), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(topics["status"]), qos
        )

        self.terrain_markers = self.build_terrain_markers()
        self.building_markers = self.build_building_markers()
        self.context_markers = self.build_context_markers()
        self.raw_cloud = cloud_message(self.raw_voxel_points, self.frame_id)
        self.inflated_cloud = cloud_message(
            self.inflated_voxel_points, self.frame_id
        )
        self.metadata_message = self.build_metadata_message()
        self.status_message = self.build_status_message()

        self.publish_all()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_all)

        features = self.manifest["features"]
        self.get_logger().info(
            "Real Yuzhong map ready: "
            f"buildings={features['real_building_footprints_in_roi']}, "
            f"terrain_relief={self.manifest['terrain']['relief_m']:.1f} m, "
            f"raw_voxels={len(self.raw_voxel_points)}, "
            f"inflated_voxels={len(self.inflated_voxel_points)}"
        )
        self.get_logger().info(
            "Planner topics: /drone/voxel_cloud_raw, "
            "/drone/voxel_cloud, /drone/voxel_metadata"
        )

    def triangle_marker(
        self,
        marker_id: int,
        namespace: str,
        vertices: np.ndarray,
        base_color: ColorRGBA,
        per_vertex_terrain_color: bool = False,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = base_color

        z_min = float(np.min(vertices[:, 2])) if len(vertices) else 0.0
        z_max = float(np.max(vertices[:, 2])) if len(vertices) else 1.0
        span = max(1.0, z_max - z_min)

        for x, y, z in vertices:
            marker.points.append(point_message(x, y, z))
            if per_vertex_terrain_color:
                value = max(0.0, min(1.0, (float(z) - z_min) / span))
                marker.colors.append(
                    color_message(
                        0.20 + 0.22 * value,
                        0.32 + 0.28 * value,
                        0.18 + 0.12 * value,
                        self.terrain_alpha,
                    )
                )
        return marker

    def build_terrain_markers(self) -> MarkerArray:
        marker = self.triangle_marker(
            0,
            "yuzhong_terrain",
            self.terrain_vertices,
            color_message(0.28, 0.45, 0.24, self.terrain_alpha),
            per_vertex_terrain_color=True,
        )
        result = MarkerArray()
        result.markers = [marker]
        return result

    def build_building_markers(self) -> MarkerArray:
        marker = self.triangle_marker(
            0,
            "yuzhong_buildings",
            self.building_vertices,
            color_message(0.72, 0.74, 0.78, self.building_alpha),
        )
        result = MarkerArray()
        result.markers = [marker]
        return result

    def line_marker(
        self,
        marker_id: int,
        namespace: str,
        vertices: np.ndarray,
        width: float,
        color: ColorRGBA,
        marker_type: int = Marker.LINE_LIST,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color = color
        marker.points = [
            point_message(x, y, z) for x, y, z in vertices
        ]
        return marker

    def build_context_markers(self) -> MarkerArray:
        markers: List[Marker] = [
            self.line_marker(
                0, "roads", self.roads_segments, 1.2,
                color_message(0.92, 0.72, 0.30, 0.82),
            ),
            self.line_marker(
                1, "railways", self.railways_segments, 1.6,
                color_message(0.60, 0.26, 0.72, 0.95),
            ),
            self.line_marker(
                2, "bridges", self.bridges_segments, 2.2,
                color_message(0.18, 0.62, 0.82, 0.95),
            ),
            self.line_marker(
                3, "planning_roi", self.roi_ring, 2.8,
                color_message(0.92, 0.24, 0.18, 1.0),
                marker_type=Marker.LINE_STRIP,
            ),
        ]

        landmark_points = Marker()
        landmark_points.header.frame_id = self.frame_id
        landmark_points.ns = "landmark_points"
        landmark_points.id = 10
        landmark_points.type = Marker.SPHERE_LIST
        landmark_points.action = Marker.ADD
        landmark_points.pose.orientation.w = 1.0
        landmark_points.scale.x = 8.0
        landmark_points.scale.y = 8.0
        landmark_points.scale.z = 8.0
        landmark_points.color = color_message(0.96, 0.28, 0.12, 0.95)

        for item in self.manifest.get("landmarks", []):
            landmark_points.points.append(
                point_message(item["x"], item["y"], item["z"])
            )
        markers.append(landmark_points)

        text_id = 20
        for item in self.manifest.get("landmarks", []):
            if item.get("category") != "landmarks":
                continue
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = "landmark_labels"
            text.id = text_id
            text_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = point_message(
                item["x"], item["y"], item["z"] + 10.0
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 8.0
            text.color = color_message(1.0, 1.0, 1.0, 1.0)
            text.text = str(item.get("name") or "Landmark")
            markers.append(text)

        result = MarkerArray()
        result.markers = markers
        return result

    def build_metadata_message(self) -> String:
        data = {
            "scenario": self.manifest["scenario"],
            "frame_id": self.frame_id,
            "resolution_m": self.manifest["resolution_m"],
            "inflation_radius_m": self.manifest["inflation_radius_m"],
            "bounds": self.manifest["bounds"],
            "dimensions": self.manifest["dimensions"],
            "base_occupied_voxels": self.manifest["base_occupied_voxels"],
            "inflated_occupied_voxels": self.manifest[
                "inflated_occupied_voxels"
            ],
            "free_voxels": self.manifest["free_voxels"],
            "source_crs": self.manifest["source_crs"],
            "local_origin": self.manifest["local_origin"],
            "features": self.manifest["features"],
        }
        message = String()
        message.data = json.dumps(data, ensure_ascii=False, sort_keys=True)
        return message

    def build_status_message(self) -> String:
        message = String()
        message.data = json.dumps(
            {
                "state": "ready",
                "success": True,
                "dataset": self.manifest["dataset_name"],
                "scenario": self.manifest["scenario"],
                "frame_id": self.frame_id,
                "terrain_relief_m": self.manifest["terrain"]["relief_m"],
                "buildings": self.manifest["features"][
                    "real_building_footprints_in_roi"
                ],
                "raw_voxels": len(self.raw_voxel_points),
                "inflated_voxels": len(self.inflated_voxel_points),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    def stamp_marker_array(self, array: MarkerArray, stamp) -> None:
        for marker in array.markers:
            marker.header.stamp = stamp

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.stamp_marker_array(self.terrain_markers, stamp)
        self.stamp_marker_array(self.building_markers, stamp)
        self.stamp_marker_array(self.context_markers, stamp)
        self.raw_cloud.header.stamp = stamp
        self.inflated_cloud.header.stamp = stamp

        self.terrain_publisher.publish(self.terrain_markers)
        self.building_publisher.publish(self.building_markers)
        self.context_publisher.publish(self.context_markers)
        self.raw_cloud_publisher.publish(self.raw_cloud)
        self.inflated_cloud_publisher.publish(self.inflated_cloud)
        self.metadata_publisher.publish(self.metadata_message)
        self.status_publisher.publish(self.status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChongqingMapServer()
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
