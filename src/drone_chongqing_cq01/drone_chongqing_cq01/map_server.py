#!/usr/bin/env python3
"""Publish the CQ-01 map, solid obstacle scene and planner voxel clouds."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import List

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
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def colour(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    value = ColorRGBA()
    value.r = float(r)
    value.g = float(g)
    value.b = float(b)
    value.a = float(a)
    return value


def point(x: float, y: float, z: float) -> Point:
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


class CQ01MapServer(Node):
    """Publish physical map topics and a separate presentation-quality scene."""

    TRIANGLE_CHUNK_POINTS = 15000

    def __init__(self) -> None:
        super().__init__("cq01_eling_liziba_map_server")

        share = Path(get_package_share_directory("drone_chongqing_cq01"))
        manifest_path = share / "runtime_data" / "cq01_manifest.json"
        data_path = share / "runtime_data" / "cq01_eling_liziba_data.npz"

        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        data = np.load(data_path, allow_pickle=False)

        self.raw_points = data["raw_points"]
        self.inflated_points = data["inflated_points"]
        self.terrain_points = data["terrain_surface_points"]
        self.building_points = data["building_points"]
        self.building_mesh_vertices = data["building_mesh_vertices"]
        self.safety_shell_points = data["safety_shell_points"]
        self.road_segments = data["road_segments"]
        self.rail_segments = data["rail_segments"]
        self.offline_path = data["offline_path_points"]
        self.start_goal = data["start_goal"]
        self.final_goal = data["final_goal"]

        self.resolution = float(
            self.manifest["simulation"]["resolution_m"]
        )

        qos = transient_qos()
        topics = self.manifest["topics"]

        # Physical/planning topics
        self.raw_publisher = self.create_publisher(
            PointCloud2, topics["raw_cloud"], qos
        )
        self.inflated_publisher = self.create_publisher(
            PointCloud2, topics["inflated_cloud"], qos
        )
        self.terrain_cloud_publisher = self.create_publisher(
            PointCloud2, topics["terrain_cloud"], qos
        )
        self.building_cloud_publisher = self.create_publisher(
            PointCloud2, topics["building_cloud"], qos
        )
        self.metadata_publisher = self.create_publisher(
            String, topics["metadata"], qos
        )
        self.status_publisher = self.create_publisher(
            String, topics["map_status"], qos
        )

        # Presentation topics
        self.solid_terrain_publisher = self.create_publisher(
            MarkerArray, "/chongqing/cq01/solid_terrain", qos
        )
        self.solid_buildings_publisher = self.create_publisher(
            MarkerArray, "/chongqing/cq01/solid_buildings", qos
        )
        self.building_collision_publisher = self.create_publisher(
            MarkerArray,
            "/chongqing/cq01/building_collision_solids",
            qos,
        )
        self.safety_volume_publisher = self.create_publisher(
            MarkerArray, "/chongqing/cq01/safety_volume", qos
        )
        self.context_publisher = self.create_publisher(
            MarkerArray, topics["context_markers"], qos
        )

        self.raw_cloud = self.make_cloud(self.raw_points)
        self.inflated_cloud = self.make_cloud(self.inflated_points)
        self.terrain_cloud = self.make_cloud(self.terrain_points)
        self.building_cloud = self.make_cloud(self.building_points)

        self.solid_terrain = self.make_terrain_tiles()
        self.solid_buildings = self.make_building_mesh()
        self.building_collision_solids = self.make_cube_list(
            self.building_points,
            namespace="building_collision_voxels",
            marker_id=0,
            cube_scale=self.resolution,
            marker_colour=colour(0.42, 0.45, 0.50, 0.75),
        )
        self.safety_volume = self.make_cube_list(
            self.safety_shell_points,
            namespace="inflated_safety_shell",
            marker_id=0,
            cube_scale=self.resolution,
            marker_colour=colour(0.95, 0.28, 0.10, 0.20),
        )
        self.context_markers = self.make_context_markers()
        self.metadata_message = self.make_metadata()
        self.status_message = self.make_status()

        self.publish_all()
        self.timer = self.create_timer(1.0, self.publish_all)

        simulation = self.manifest["simulation"]
        validation = self.manifest["offline_validation"]
        self.get_logger().info(
            "CQ-01 solid scene ready: "
            f"terrain_tiles={len(self.terrain_points)}, "
            f"building_mesh_vertices={len(self.building_mesh_vertices)}, "
            f"building_voxels={len(self.building_points)}"
        )
        self.get_logger().info(
            "CQ-01 physical map ready: "
            f"raw={simulation['raw_occupied_voxels']}, "
            f"inflated={simulation['inflated_occupied_voxels']}, "
            f"offline_3d_astar={validation['path_found']}"
        )

    def make_cloud(self, points_array: np.ndarray) -> PointCloud2:
        message = PointCloud2()
        message.header = Header()
        message.header.frame_id = "map"
        message.height = 1
        message.width = int(len(points_array))
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(
            struct.pack("<fff", float(x), float(y), float(z))
            for x, y, z in points_array
        )
        return message

    def make_cube_list(
        self,
        points_array: np.ndarray,
        namespace: str,
        marker_id: int,
        cube_scale: float,
        marker_colour: ColorRGBA,
        z_scale: float = None,
    ) -> MarkerArray:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(cube_scale)
        marker.scale.y = float(cube_scale)
        marker.scale.z = float(
            cube_scale if z_scale is None else z_scale
        )
        marker.color = marker_colour
        marker.points = [
            point(x, y, z)
            for x, y, z in points_array
        ]
        message = MarkerArray()
        message.markers = [marker]
        return message

    def make_terrain_tiles(self) -> MarkerArray:
        # Thin solid tiles retain the real DEM surface while avoiding a
        # diagnostic point-cloud appearance.
        return self.make_cube_list(
            self.terrain_points,
            namespace="solid_dem_tiles",
            marker_id=0,
            cube_scale=self.resolution,
            z_scale=0.12,
            marker_colour=colour(0.24, 0.48, 0.22, 0.98),
        )

    def make_building_mesh(self) -> MarkerArray:
        markers: List[Marker] = []
        chunk_size = (
            self.TRIANGLE_CHUNK_POINTS
            - self.TRIANGLE_CHUNK_POINTS % 3
        )
        marker_id = 0

        for offset in range(
            0,
            len(self.building_mesh_vertices),
            chunk_size,
        ):
            chunk = self.building_mesh_vertices[
                offset : offset + chunk_size
            ]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "solid_osm_buildings"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = colour(0.68, 0.71, 0.76, 0.98)
            marker.points = [
                point(x, y, z)
                for x, y, z in chunk
            ]
            markers.append(marker)

        message = MarkerArray()
        message.markers = markers
        return message

    def make_metadata(self) -> String:
        simulation = self.manifest["simulation"]
        message = String()
        message.data = json.dumps(
            {
                "scenario": self.manifest["scenario"],
                "frame_id": simulation["frame_id"],
                "resolution_m": simulation["resolution_m"],
                "inflation_radius_m": simulation["inflation_radius_m"],
                "bounds": simulation["bounds"],
                "dimensions": simulation["dimensions"],
                "base_occupied_voxels": simulation["raw_occupied_voxels"],
                "inflated_occupied_voxels": simulation[
                    "inflated_occupied_voxels"
                ],
                "source_crs": self.manifest["source_crs"],
                "uniform_geometry_scale": self.manifest[
                    "geometric_scale"
                ]["uniform_scale"],
                "solid_visualization": True,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    def make_status(self) -> String:
        message = String()
        message.data = json.dumps(
            {
                "state": "ready",
                "success": True,
                "scenario": self.manifest["scenario_cn"],
                "scale_ratio": self.manifest["geometric_scale"][
                    "scale_ratio"
                ],
                "buildings": self.manifest["real_world"][
                    "buildings_in_roi"
                ],
                "terrain_relief_m": self.manifest["simulation"][
                    "terrain_relief_m"
                ],
                "fixed_height_path_exists": self.manifest[
                    "offline_validation"
                ]["fixed_height_path_exists"],
                "visualization_mode": "solid_obstacles",
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        return message

    def make_context_markers(self) -> MarkerArray:
        markers = []

        def line_marker(
            marker_id: int,
            namespace: str,
            vertices: np.ndarray,
            width: float,
            marker_colour: ColorRGBA,
            marker_type: int = Marker.LINE_LIST,
        ) -> Marker:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = namespace
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.color = marker_colour
            marker.points = [point(x, y, z) for x, y, z in vertices]
            return marker

        if len(self.road_segments):
            markers.append(
                line_marker(
                    0,
                    "roads",
                    self.road_segments,
                    0.08,
                    colour(0.96, 0.68, 0.18, 0.95),
                )
            )
        if len(self.rail_segments):
            markers.append(
                line_marker(
                    1,
                    "railways",
                    self.rail_segments,
                    0.12,
                    colour(0.70, 0.18, 0.84, 0.98),
                )
            )

        # Safe reference path
        markers.append(
            line_marker(
                2,
                "offline_3d_astar_reference",
                self.offline_path,
                0.10,
                colour(0.18, 0.84, 0.96, 0.80),
                Marker.LINE_STRIP,
            )
        )

        # Direct line, shown in red for comparison with the safe 3D route.
        direct_line = np.asarray(
            [self.start_goal, self.final_goal],
            dtype=np.float32,
        )
        markers.append(
            line_marker(
                3,
                "direct_line_comparison",
                direct_line,
                0.06,
                colour(0.96, 0.18, 0.12, 0.80),
                Marker.LINE_STRIP,
            )
        )

        for marker_id, name, location, marker_colour in [
            (
                10,
                "嘉陵江滨江路山脚 START",
                self.start_goal,
                colour(0.12, 0.95, 0.28, 1.0),
            ),
            (
                11,
                "鹅岭山脊高点 GOAL",
                self.final_goal,
                colour(0.96, 0.18, 0.12, 1.0),
            ),
        ]:
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.ns = "mission_points"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = point(*location)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.50
            sphere.scale.y = 0.50
            sphere.scale.z = 0.50
            sphere.color = marker_colour
            markers.append(sphere)

            label = Marker()
            label.header.frame_id = "map"
            label.ns = "mission_labels"
            label.id = marker_id + 100
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = point(
                float(location[0]),
                float(location[1]),
                float(location[2] + 0.8),
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.45
            label.color = colour(1.0, 1.0, 1.0, 1.0)
            label.text = name
            markers.append(label)

        message = MarkerArray()
        message.markers = markers
        return message

    @staticmethod
    def stamp_markers(message: MarkerArray, stamp) -> None:
        for marker in message.markers:
            marker.header.stamp = stamp

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()

        for cloud in (
            self.raw_cloud,
            self.inflated_cloud,
            self.terrain_cloud,
            self.building_cloud,
        ):
            cloud.header.stamp = stamp

        for marker_array in (
            self.solid_terrain,
            self.solid_buildings,
            self.building_collision_solids,
            self.safety_volume,
            self.context_markers,
        ):
            self.stamp_markers(marker_array, stamp)

        self.raw_publisher.publish(self.raw_cloud)
        self.inflated_publisher.publish(self.inflated_cloud)
        self.terrain_cloud_publisher.publish(self.terrain_cloud)
        self.building_cloud_publisher.publish(self.building_cloud)
        self.solid_terrain_publisher.publish(self.solid_terrain)
        self.solid_buildings_publisher.publish(self.solid_buildings)
        self.building_collision_publisher.publish(
            self.building_collision_solids
        )
        self.safety_volume_publisher.publish(self.safety_volume)
        self.context_publisher.publish(self.context_markers)
        self.metadata_publisher.publish(self.metadata_message)
        self.status_publisher.publish(self.status_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01MapServer()
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
