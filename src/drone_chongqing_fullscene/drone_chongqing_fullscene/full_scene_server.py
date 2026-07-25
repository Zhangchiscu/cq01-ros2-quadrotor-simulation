#!/usr/bin/env python3
"""Publish the full Yuzhong peninsula as a ROS2/RViz overview scene."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List

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
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE_NAME = "drone_chongqing_fullscene"
TRIANGLE_CHUNK_POINTS = 30000
LINE_CHUNK_POINTS = 20000


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


class FullYuzhongScene(Node):
    """Publish a full-peninsula display scene without changing physical scale data."""

    def __init__(self) -> None:
        super().__init__("chongqing_full_yuzhong_scene")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("display_vertical_scale", 2.5)
        self.declare_parameter("publish_period_s", 5.0)
        self.declare_parameter("show_buildings", True)
        self.declare_parameter("show_transport", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.display_vertical_scale = max(
            1.0,
            float(self.get_parameter("display_vertical_scale").value),
        )
        self.publish_period_s = max(
            5.0,
            float(self.get_parameter("publish_period_s").value),
        )
        self.show_buildings = bool(self.get_parameter("show_buildings").value)
        self.show_transport = bool(self.get_parameter("show_transport").value)

        share = Path(get_package_share_directory(PACKAGE_NAME))
        data_path = share / "runtime_data" / "yuzhong_full_scene_data.npz"
        manifest_path = share / "runtime_data" / "yuzhong_full_scene_manifest.json"

        if not data_path.exists() or not manifest_path.exists():
            raise FileNotFoundError(
                f"Missing full-scene runtime data below {share}"
            )

        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        arrays = np.load(data_path, allow_pickle=False)

        self.terrain_vertices = arrays["terrain_vertices"]
        self.building_vertices = arrays["building_vertices"]
        self.water_vertices = arrays["water_vertices"]
        self.cliff_vertices = arrays["cliff_vertices"]
        self.contour_segments = arrays["contour_segments"]
        self.boundary_ring = arrays["boundary_ring"]
        self.roads_segments = arrays["roads_segments"]
        self.railways_segments = arrays["railways_segments"]
        self.bridge_vertices = arrays["bridge_vertices"]
        self.bridge_pillar_vertices = arrays["bridge_pillar_vertices"]
        self.landmark_points = arrays["landmark_points"]
        self.mission_route = arrays["mission_route"]
        self.mission_roi_rings = arrays["mission_roi_rings"]

        qos = transient_qos()
        topics = self.manifest["topics"]

        self.terrain_publisher = self.create_publisher(
            MarkerArray, topics["terrain"], qos
        )
        self.building_publisher = self.create_publisher(
            MarkerArray, topics["buildings"], qos
        )
        self.water_publisher = self.create_publisher(
            MarkerArray, topics["water_cliffs"], qos
        )
        self.topography_publisher = self.create_publisher(
            MarkerArray, topics["topography"], qos
        )
        self.transport_publisher = self.create_publisher(
            MarkerArray, topics["transport"], qos
        )
        self.mission_publisher = self.create_publisher(
            MarkerArray, topics["mission"], qos
        )
        self.metadata_publisher = self.create_publisher(
            String, topics["metadata"], qos
        )
        self.status_publisher = self.create_publisher(
            String, topics["status"], qos
        )

        self.terrain_markers = self.build_terrain_markers()
        self.building_markers = self.build_building_markers()
        self.water_markers = self.build_water_markers()
        self.topography_markers = self.build_topography_markers()
        self.transport_markers = self.build_transport_markers()
        self.mission_markers = self.build_mission_markers()
        self.metadata_message = self.build_metadata_message()
        self.status_message = self.build_status_message()

        self.publish_all()
        self.timer = self.create_timer(
            self.publish_period_s,
            self.publish_all,
        )

        dimensions = self.manifest["dimensions_m"]
        features = self.manifest["features"]
        relief = self.manifest["dem"]["relief_m"]

        self.get_logger().info(
            "Full Yuzhong scene ready: "
            f"{dimensions[0]:.0f} x {dimensions[1]:.0f} m, "
            f"terrain relief={relief:.1f} m, "
            f"buildings={features['buildings']}, "
            f"display_vertical_scale={self.display_vertical_scale:.2f}"
        )
        self.get_logger().info(
            "Physical terrain scale remains 1.0; "
            "vertical exaggeration affects RViz display only."
        )

    def scaled(self, vertices: np.ndarray) -> np.ndarray:
        result = np.asarray(vertices, dtype=np.float32).copy()
        if result.size:
            result[:, 2] *= self.display_vertical_scale
        return result

    def triangle_markers(
        self,
        vertices: np.ndarray,
        namespace: str,
        start_id: int,
        color: ColorRGBA,
        altitude_colors: bool = False,
    ) -> List[Marker]:
        scaled = self.scaled(vertices)
        markers: List[Marker] = []

        chunk_size = TRIANGLE_CHUNK_POINTS - (TRIANGLE_CHUNK_POINTS % 3)
        global_min = float(np.min(scaled[:, 2])) if len(scaled) else 0.0
        global_max = float(np.max(scaled[:, 2])) if len(scaled) else 1.0
        span = max(1.0, global_max - global_min)

        marker_id = start_id
        for offset in range(0, len(scaled), chunk_size):
            chunk = scaled[offset:offset + chunk_size]
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = namespace
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = color

            for x_value, y_value, z_value in chunk:
                marker.points.append(
                    point_message(x_value, y_value, z_value)
                )
                if altitude_colors:
                    ratio = max(
                        0.0,
                        min(1.0, (float(z_value) - global_min) / span),
                    )
                    marker.colors.append(
                        color_message(
                            0.18 + 0.30 * ratio,
                            0.28 + 0.34 * ratio,
                            0.14 + 0.18 * ratio,
                            color.a,
                        )
                    )
            markers.append(marker)

        return markers

    def line_markers(
        self,
        vertices: np.ndarray,
        namespace: str,
        start_id: int,
        width: float,
        color: ColorRGBA,
        line_strip: bool = False,
    ) -> List[Marker]:
        scaled = self.scaled(vertices)
        if not len(scaled):
            return []

        marker_type = Marker.LINE_STRIP if line_strip else Marker.LINE_LIST
        chunk_size = LINE_CHUNK_POINTS

        markers = []
        marker_id = start_id
        for offset in range(0, len(scaled), chunk_size):
            chunk = scaled[offset:offset + chunk_size]
            if not line_strip and len(chunk) % 2:
                chunk = chunk[:-1]
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = namespace
            marker.id = marker_id
            marker_id += 1
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(width)
            marker.color = color
            marker.points = [
                point_message(x_value, y_value, z_value)
                for x_value, y_value, z_value in chunk
            ]
            markers.append(marker)
        return markers

    def build_terrain_markers(self) -> MarkerArray:
        result = MarkerArray()
        result.markers = self.triangle_markers(
            self.terrain_vertices,
            "full_yuzhong_terrain",
            0,
            color_message(0.30, 0.50, 0.24, 0.96),
            altitude_colors=True,
        )
        return result

    def build_building_markers(self) -> MarkerArray:
        result = MarkerArray()
        if self.show_buildings:
            result.markers = self.triangle_markers(
                self.building_vertices,
                "full_yuzhong_buildings",
                100,
                color_message(0.72, 0.75, 0.80, 0.93),
            )
        return result

    def build_water_markers(self) -> MarkerArray:
        markers = []
        markers.extend(
            self.triangle_markers(
                self.water_vertices,
                "context_water",
                200,
                color_message(0.12, 0.42, 0.72, 0.88),
            )
        )
        markers.extend(
            self.triangle_markers(
                self.cliff_vertices,
                "peninsula_shore_cliff",
                300,
                color_message(0.43, 0.32, 0.20, 0.95),
            )
        )
        result = MarkerArray()
        result.markers = markers
        return result

    def build_topography_markers(self) -> MarkerArray:
        markers = []
        markers.extend(
            self.line_markers(
                self.contour_segments,
                "elevation_contours",
                400,
                2.0,
                color_message(0.86, 0.82, 0.60, 0.58),
            )
        )
        markers.extend(
            self.line_markers(
                self.boundary_ring,
                "peninsula_boundary",
                450,
                4.0,
                color_message(0.98, 0.94, 0.78, 0.98),
                line_strip=True,
            )
        )
        result = MarkerArray()
        result.markers = markers
        return result

    def build_transport_markers(self) -> MarkerArray:
        markers = []
        if self.show_transport:
            markers.extend(
                self.line_markers(
                    self.roads_segments,
                    "roads",
                    500,
                    2.0,
                    color_message(0.93, 0.69, 0.25, 0.82),
                )
            )
            markers.extend(
                self.line_markers(
                    self.railways_segments,
                    "railways",
                    550,
                    3.0,
                    color_message(0.66, 0.28, 0.78, 0.95),
                )
            )
            markers.extend(
                self.triangle_markers(
                    self.bridge_vertices,
                    "bridge_decks",
                    600,
                    color_message(0.20, 0.60, 0.78, 0.96),
                )
            )
            markers.extend(
                self.triangle_markers(
                    self.bridge_pillar_vertices,
                    "bridge_pillars",
                    700,
                    color_message(0.46, 0.50, 0.55, 0.98),
                )
            )
        result = MarkerArray()
        result.markers = markers
        return result

    def build_mission_markers(self) -> MarkerArray:
        markers: List[Marker] = []

        markers.extend(
            self.line_markers(
                self.mission_route,
                "mountain_city_mission_axis",
                800,
                8.0,
                color_message(0.96, 0.22, 0.12, 0.98),
                line_strip=True,
            )
        )

        for roi_index, ring in enumerate(self.mission_roi_rings):
            markers.extend(
                self.line_markers(
                    ring,
                    f"mission_roi_{roi_index + 1}",
                    820 + roi_index,
                    6.0,
                    color_message(
                        0.18 + 0.18 * roi_index,
                        0.92 - 0.14 * roi_index,
                        0.88,
                        0.98,
                    ),
                    line_strip=True,
                )
            )

        landmark_marker = Marker()
        landmark_marker.header.frame_id = self.frame_id
        landmark_marker.ns = "landmark_points"
        landmark_marker.id = 850
        landmark_marker.type = Marker.SPHERE_LIST
        landmark_marker.action = Marker.ADD
        landmark_marker.pose.orientation.w = 1.0
        landmark_marker.scale.x = 30.0
        landmark_marker.scale.y = 30.0
        landmark_marker.scale.z = 30.0
        landmark_marker.color = color_message(0.98, 0.28, 0.10, 0.98)

        for x_value, y_value, z_value in self.scaled(self.landmark_points):
            landmark_marker.points.append(
                point_message(x_value, y_value, z_value)
            )
        markers.append(landmark_marker)

        for index, record in enumerate(self.manifest["landmarks"]):
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = "landmark_labels"
            text.id = 900 + index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = point_message(
                record["x"],
                record["y"],
                (record["z"] + 22.0) * self.display_vertical_scale,
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 34.0
            text.color = color_message(1.0, 1.0, 1.0, 1.0)
            text.text = record["name"]
            markers.append(text)

        for index, roi in enumerate(self.manifest["mission_rois"]):
            ring = self.mission_roi_rings[index]
            center = np.mean(ring[:4], axis=0)
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = "mission_roi_labels"
            text.id = 950 + index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = point_message(
                float(center[0]),
                float(center[1]),
                float(center[2] * self.display_vertical_scale + 60.0),
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 30.0
            text.color = color_message(0.98, 0.96, 0.82, 1.0)
            text.text = roi["name"]
            markers.append(text)

        result = MarkerArray()
        result.markers = markers
        return result

    def build_metadata_message(self) -> String:
        payload = dict(self.manifest)
        payload["active_display_vertical_scale"] = self.display_vertical_scale
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    def build_status_message(self) -> String:
        message = String()
        message.data = json.dumps(
            {
                "state": "ready",
                "success": True,
                "dataset": self.manifest["dataset_name"],
                "frame_id": self.frame_id,
                "physical_vertical_scale": 1.0,
                "display_vertical_scale": self.display_vertical_scale,
                "display_only_vertical_exaggeration": (
                    self.display_vertical_scale != 1.0
                ),
                "terrain_relief_m": self.manifest["dem"]["relief_m"],
                "buildings": self.manifest["features"]["buildings"],
                "mission_rois": len(self.manifest["mission_rois"]),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    @staticmethod
    def stamp_marker_array(marker_array: MarkerArray, stamp) -> None:
        for marker in marker_array.markers:
            marker.header.stamp = stamp

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker_array in (
            self.terrain_markers,
            self.building_markers,
            self.water_markers,
            self.topography_markers,
            self.transport_markers,
            self.mission_markers,
        ):
            self.stamp_marker_array(marker_array, stamp)

        self.terrain_publisher.publish(self.terrain_markers)
        self.building_publisher.publish(self.building_markers)
        self.water_publisher.publish(self.water_markers)
        self.topography_publisher.publish(self.topography_markers)
        self.transport_publisher.publish(self.transport_markers)
        self.mission_publisher.publish(self.mission_markers)
        self.metadata_publisher.publish(self.metadata_message)
        self.status_publisher.publish(self.status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FullYuzhongScene()
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
