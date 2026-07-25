#!/usr/bin/env python3
"""Generate a compressed but highly recognizable Yuzhong Peninsula scene."""

from __future__ import annotations

import json
import math
import random
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


Color = Tuple[float, float, float, float]
Point3 = Tuple[float, float, float]
SCENE_REVISION = "yuzhong_solid_peninsula_v3"


class SceneServer(Node):
    def __init__(self) -> None:
        super().__init__('chongqing_scene_server')
        self.declare_parameter('config_file', '')
        self.declare_parameter('marker_topic', '/chongqing/scene_markers')
        self.declare_parameter('manifest_topic', '/chongqing/scene_manifest')
        self.declare_parameter('show_labels', True)
        self.declare_parameter('building_count', -1)
        self.declare_parameter('publish_hz', -1.0)

        configuration = self._load_configuration(
            str(self.get_parameter('config_file').value)
        )
        self.scene = dict(configuration['scene'])
        self.landmarks = list(configuration.get('landmarks', []))
        self.routes = list(configuration.get('routes', []))
        self.frame_id = str(self.scene.get('frame_id', 'map'))
        self.position_scale_x = float(
            self.scene.get('position_scale_x', 4.20)
        )
        self.position_scale_y = float(
            self.scene.get('position_scale_y', 3.20)
        )
        self.object_footprint_scale = float(
            self.scene.get('object_footprint_scale', 1.90)
        )
        self.object_height_scale = float(
            self.scene.get('object_height_scale', 2.20)
        )
        self.line_width_scale = float(
            self.scene.get('line_width_scale', 1.25)
        )
        self.text_scale = float(
            self.scene.get('text_scale', 1.20)
        )
        self.show_labels = bool(self.get_parameter('show_labels').value)
        requested_count = int(self.get_parameter('building_count').value)
        self.building_target_count = (
            requested_count if requested_count > 0
            else int(self.scene.get('building_target_count', 430))
        )
        self.random = random.Random(int(self.scene.get('seed', 20260711)))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            qos,
        )
        self.manifest_publisher = self.create_publisher(
            String,
            str(self.get_parameter('manifest_topic').value),
            qos,
        )

        self.marker_array = self._build_scene()
        self.manifest = self._build_manifest()

        configured_hz = float(self.scene.get('publish_hz', 0.5))
        requested_hz = float(self.get_parameter('publish_hz').value)
        publish_hz = requested_hz if requested_hz > 0.0 else configured_hz
        self.timer = self.create_timer(1.0 / max(0.1, publish_hz), self.publish)
        self.publish()

        self.get_logger().info(
            f'Chongqing scene ready: revision={SCENE_REVISION}, '
            f'markers={len(self.marker_array.markers)}, '
            f'landmarks={len(self.landmarks)}, '
            f'generic_buildings={self.generated_building_count}'
        )

    def _load_configuration(self, config_file: str) -> Dict[str, object]:
        if config_file:
            path = Path(config_file).expanduser()
        else:
            path = (
                Path(get_package_share_directory('drone_chongqing'))
                / 'config'
                / 'yuzhong_peninsula.yaml'
            )
        if not path.exists():
            raise FileNotFoundError(f'Chongqing scene config not found: {path}')
        with path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict) or 'scene' not in data:
            raise ValueError(f'Invalid Chongqing scene config: {path}')
        self.get_logger().info(f'Loaded Chongqing configuration: {path}')
        return data

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _interpolate(points: Sequence[Tuple[float, float]], x: float) -> float:
        if x <= points[0][0]:
            return points[0][1]
        if x >= points[-1][0]:
            return points[-1][1]
        for first, second in zip(points[:-1], points[1:]):
            if first[0] <= x <= second[0]:
                ratio = (x - first[0]) / (second[0] - first[0])
                return first[1] + ratio * (second[1] - first[1])
        return points[-1][1]


    def north_edge(self, x: float) -> float:
        """Approximate the asymmetric Jialing River shoreline."""
        outline = [
            (-62.0, 4.5),
            (-58.0, 8.0),
            (-52.0, 13.5),
            (-45.0, 18.5),
            (-37.0, 22.5),
            (-28.0, 24.8),
            (-18.0, 25.6),
            (-8.0, 24.6),
            (3.0, 25.7),
            (14.0, 27.2),
            (24.0, 24.7),
            (34.0, 21.0),
            (43.0, 16.7),
            (51.0, 11.0),
            (57.0, 6.0),
            (61.0, 1.5),
            (62.0, 0.0),
        ]
        return self._interpolate(outline, x)

    def south_edge(self, x: float) -> float:
        """Approximate the broader and more concave Yangtze shoreline."""
        outline = [
            (-62.0, -5.5),
            (-58.0, -9.0),
            (-52.0, -14.0),
            (-45.0, -19.0),
            (-36.0, -24.0),
            (-25.0, -27.5),
            (-13.0, -30.2),
            (0.0, -31.6),
            (13.0, -30.5),
            (25.0, -27.5),
            (36.0, -23.0),
            (46.0, -17.2),
            (54.0, -11.0),
            (59.0, -5.2),
            (62.0, 0.0),
        ]
        return self._interpolate(outline, x)

    def center_y(self, x: float) -> float:
        north = self.north_edge(x)
        south = self.south_edge(x)
        # The urban ridge is slightly north of the geometric centre
        # through the middle and shifts south near Chaotianmen.
        width = north - south
        shift = (
            0.035 * width * math.sin(0.045 * (x + 20.0))
            - 0.55 * math.exp(-((x - 50.0) / 11.0) ** 2)
        )
        return 0.5 * (north + south) + shift

    def half_width(self, x: float) -> float:
        return 0.5 * (
            self.north_edge(x) - self.south_edge(x)
        )

    def corridor_y(
        self,
        x: float,
        fraction: float,
    ) -> float:
        """Map -1..1 from south shore through ridge to north shore."""
        centre = self.center_y(x)
        if fraction >= 0.0:
            return centre + fraction * (
                self.north_edge(x) - centre
            )
        return centre + (-fraction) * (
            self.south_edge(x) - centre
        )

    def inside_peninsula(
        self,
        x: float,
        y: float,
        margin_ratio: float = 0.0,
    ) -> bool:
        if x < -62.0 or x > 62.0:
            return False
        north = self.north_edge(x)
        south = self.south_edge(x)
        margin = max(
            0.0,
            margin_ratio * (north - south),
        )
        return (
            south + margin
            <= y
            <= north - margin
        )


    def terrain_z(self, x: float, y: float) -> float:
        """Solid mountain-city relief with an elevated longitudinal ridge."""
        north = self.north_edge(x)
        south = self.south_edge(x)
        centre = self.center_y(x)

        if y >= centre:
            denominator = max(1.0, north - centre)
            lateral = (y - centre) / denominator
        else:
            denominator = max(1.0, centre - south)
            lateral = (centre - y) / denominator
        lateral = min(1.15, max(0.0, lateral))

        shore_level = (
            4.2
            + 0.9 * math.sin(0.075 * x + 0.4)
            + 0.5 * math.sin(0.17 * x)
        )

        ridge_level = (
            58.0
            - 0.16 * (x + 8.0)
            + 17.0 * math.exp(-((x + 43.0) / 15.0) ** 2)
            + 13.0 * math.exp(-((x - 2.0) / 24.0) ** 2)
            + 6.0 * math.exp(-((x - 28.0) / 17.0) ** 2)
        )

        # A broad high ridge with steep final descent to both rivers.
        profile = max(
            0.0,
            1.0 - lateral ** 1.48,
        ) ** 0.78

        # The south bank is broader; the north bank near Hongyadong is
        # deliberately steeper to express the cliff-city relationship.
        north_cliff = (
            8.0
            * max(0.0, (lateral - 0.58) / 0.42)
            * math.exp(-((x - 25.0) / 24.0) ** 2)
            if y >= centre
            else 0.0
        )
        south_cliff = (
            5.0
            * max(0.0, (lateral - 0.66) / 0.34)
            * math.exp(-((x + 2.0) / 30.0) ** 2)
            if y < centre
            else 0.0
        )

        urban_terraces = (
            1.8 * math.sin(0.14 * x + 0.22 * y)
            + 1.0 * math.sin(0.31 * x - 0.17 * y)
        ) * max(0.0, 1.0 - lateral)

        return max(
            2.0,
            shore_level
            + (ridge_level - shore_level) * profile
            - north_cliff
            - south_cliff
            + urban_terraces,
        )

    def _point(self, x: float, y: float, z: float) -> Point:
        return Point(
            x=float(x) * self.position_scale_x,
            y=float(y) * self.position_scale_y,
            z=float(z),
        )

    def _base_marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _set_color(marker: Marker, color: Color) -> None:
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color


    def _cube(
        self,
        namespace: str,
        marker_id: int,
        center: Point3,
        size: Point3,
        color: Color,
        yaw: float = 0.0,
        world_aligned: bool = False,
        height_scale: float | None = None,
    ) -> Marker:
        marker = self._base_marker(
            namespace,
            marker_id,
            Marker.CUBE,
        )
        marker.pose.position.x = (
            center[0] * self.position_scale_x
        )
        marker.pose.position.y = (
            center[1] * self.position_scale_y
        )

        applied_height_scale = (
            self.object_height_scale
            if height_scale is None
            else float(height_scale)
        )
        base_z = center[2] - size[2] / 2.0
        rendered_height = size[2] * applied_height_scale
        marker.pose.position.z = (
            base_z + rendered_height / 2.0
        )
        marker.pose.orientation = self._quaternion_from_yaw(
            yaw
        )

        if world_aligned:
            marker.scale.x = size[0] * self.position_scale_x
            marker.scale.y = size[1] * self.position_scale_y
        else:
            marker.scale.x = (
                size[0] * self.object_footprint_scale
            )
            marker.scale.y = (
                size[1] * self.object_footprint_scale
            )
        marker.scale.z = rendered_height
        self._set_color(marker, color)
        return marker


    def _cylinder(
        self,
        namespace: str,
        marker_id: int,
        center: Point3,
        diameter: float,
        height: float,
        color: Color,
    ) -> Marker:
        marker = self._base_marker(
            namespace,
            marker_id,
            Marker.CYLINDER,
        )
        marker.pose.position.x = (
            center[0] * self.position_scale_x
        )
        marker.pose.position.y = (
            center[1] * self.position_scale_y
        )
        base_z = center[2] - height / 2.0
        rendered_height = (
            height * self.object_height_scale
        )
        marker.pose.position.z = (
            base_z + rendered_height / 2.0
        )
        marker.scale.x = (
            diameter * self.object_footprint_scale
        )
        marker.scale.y = (
            diameter * self.object_footprint_scale
        )
        marker.scale.z = rendered_height
        self._set_color(marker, color)
        return marker


    def _line(
        self,
        namespace: str,
        marker_id: int,
        points: Iterable[Point3],
        width: float,
        color: Color,
        closed: bool = False,
    ) -> Marker:
        marker = self._base_marker(
            namespace,
            marker_id,
            Marker.LINE_STRIP,
        )
        marker.scale.x = (
            width * self.line_width_scale
        )
        self._set_color(marker, color)
        values = list(points)
        if closed and values:
            values.append(values[0])
        marker.points = [
            self._point(*value)
            for value in values
        ]
        return marker


    def _text(
        self,
        namespace: str,
        marker_id: int,
        position: Point3,
        text: str,
        scale: float = 1.3,
    ) -> Marker:
        marker = self._base_marker(
            namespace,
            marker_id,
            Marker.TEXT_VIEW_FACING,
        )
        marker.pose.position.x = (
            position[0] * self.position_scale_x
        )
        marker.pose.position.y = (
            position[1] * self.position_scale_y
        )
        marker.pose.position.z = position[2]
        marker.scale.z = scale * self.text_scale
        marker.text = text
        self._set_color(
            marker,
            (0.97, 0.95, 0.86, 0.98),
        )
        return marker


    def _terrain_marker(self, marker_id: int) -> Marker:
        """Create a closed terrain solid instead of a one-sided shell."""
        marker = self._base_marker(
            'terrain_solid',
            marker_id,
            Marker.TRIANGLE_LIST,
        )
        self._set_color(
            marker,
            (0.29, 0.23, 0.16, 1.0),
        )

        nx = int(
            self.scene.get(
                'terrain_grid_x',
                91,
            )
        )
        ny = int(
            self.scene.get(
                'terrain_grid_y',
                45,
            )
        )
        minimum_x = -62.0
        maximum_x = 62.0
        bottom_z = float(
            self.scene.get(
                'terrain_bottom_z',
                -8.0,
            )
        )

        xs = [
            minimum_x
            + (maximum_x - minimum_x)
            * index
            / (nx - 1)
            for index in range(nx)
        ]
        fractions = [
            index / (ny - 1)
            for index in range(ny)
        ]

        def surface_point(
            x: float,
            fraction: float,
        ) -> Point3:
            south = self.south_edge(x)
            north = self.north_edge(x)
            y = south + fraction * (north - south)
            return (
                x,
                y,
                self.terrain_z(x, y),
            )

        # Closed top and bottom grids.
        for i in range(nx - 1):
            for j in range(ny - 1):
                top = [
                    surface_point(
                        xs[i],
                        fractions[j],
                    ),
                    surface_point(
                        xs[i + 1],
                        fractions[j],
                    ),
                    surface_point(
                        xs[i + 1],
                        fractions[j + 1],
                    ),
                    surface_point(
                        xs[i],
                        fractions[j + 1],
                    ),
                ]
                bottom = [
                    (
                        point[0],
                        point[1],
                        bottom_z,
                    )
                    for point in top
                ]

                for index in (
                    0, 1, 2,
                    0, 2, 3,
                ):
                    marker.points.append(
                        self._point(*top[index])
                    )

                for index in (
                    0, 2, 1,
                    0, 3, 2,
                ):
                    marker.points.append(
                        self._point(*bottom[index])
                    )

        # North and south side walls.
        for edge_name in (
            'north',
            'south',
        ):
            for i in range(nx - 1):
                x0 = xs[i]
                x1 = xs[i + 1]
                if edge_name == 'north':
                    y0 = self.north_edge(x0)
                    y1 = self.north_edge(x1)
                else:
                    y0 = self.south_edge(x0)
                    y1 = self.south_edge(x1)

                top0 = (
                    x0,
                    y0,
                    self.terrain_z(x0, y0),
                )
                top1 = (
                    x1,
                    y1,
                    self.terrain_z(x1, y1),
                )
                low0 = (
                    x0,
                    y0,
                    bottom_z,
                )
                low1 = (
                    x1,
                    y1,
                    bottom_z,
                )

                for point in (
                    top0,
                    low0,
                    low1,
                    top0,
                    low1,
                    top1,
                ):
                    marker.points.append(
                        self._point(*point)
                    )

        # West land connection and pointed Chaotianmen end cap.
        for x in (
            minimum_x,
            maximum_x,
        ):
            south = self.south_edge(x)
            north = self.north_edge(x)
            top_south = (
                x,
                south,
                self.terrain_z(x, south),
            )
            top_north = (
                x,
                north,
                self.terrain_z(x, north),
            )
            low_south = (
                x,
                south,
                bottom_z,
            )
            low_north = (
                x,
                north,
                bottom_z,
            )
            for point in (
                top_south,
                low_south,
                low_north,
                top_south,
                low_north,
                top_north,
            ):
                marker.points.append(
                    self._point(*point)
                )

        return marker


    def _shoreline_markers(
        self,
        start_id: int,
    ) -> List[Marker]:
        north: List[Point3] = []
        south: List[Point3] = []

        for index in range(120):
            x = -62.0 + 124.0 * index / 119.0
            north_y = self.north_edge(x)
            south_y = self.south_edge(x)
            north.append((
                x,
                north_y,
                self.terrain_z(
                    x,
                    north_y,
                ) + 0.18,
            ))
            south.append((
                x,
                south_y,
                self.terrain_z(
                    x,
                    south_y,
                ) + 0.18,
            ))

        return [
            self._line(
                'shoreline',
                start_id,
                north,
                0.48,
                (0.84, 0.72, 0.43, 0.98),
            ),
            self._line(
                'shoreline',
                start_id + 1,
                south,
                0.48,
                (0.84, 0.72, 0.43, 0.98),
            ),
        ]


    def _water_markers(
        self,
        start_id: int,
    ) -> List[Marker]:
        water = (0.035, 0.22, 0.43, 0.92)
        return [
            self._cube(
                'water',
                start_id,
                (0.0, 45.0, -0.15),
                (142.0, 31.0, 0.45),
                water,
                world_aligned=True,
                height_scale=1.0,
            ),
            self._cube(
                'water',
                start_id + 1,
                (0.0, -47.0, -0.15),
                (142.0, 33.0, 0.45),
                water,
                world_aligned=True,
                height_scale=1.0,
            ),
            self._cube(
                'water',
                start_id + 2,
                (67.0, 0.0, -0.15),
                (30.0, 96.0, 0.45),
                water,
                world_aligned=True,
                height_scale=1.0,
            ),
        ]

    def _landmark_exclusions(self) -> List[Tuple[float, float, float]]:
        result = []
        for item in self.landmarks:
            x, y = [float(value) for value in item['position']]
            result.append((x, y, float(item.get('exclusion_radius', 4.0))))
        return result



    def _generate_buildings(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        """Generate a dense stepped skyline using efficient CUBE_LISTs."""
        exclusions = self._landmark_exclusions()
        groups: Dict[Tuple[int, int], Marker] = {}

        colors = [
            (0.32, 0.37, 0.44, 0.98),
            (0.43, 0.39, 0.36, 0.98),
            (0.52, 0.52, 0.51, 0.98),
            (0.27, 0.31, 0.37, 0.98),
            (0.56, 0.47, 0.38, 0.98),
            (0.40, 0.46, 0.52, 0.98),
        ]

        # Values are rendered metres.  V2 topped out near 30 m;
        # V3 reaches 78 m for the dense commercial skyline.
        classes = [
            (4.8, 4.2, 10.0),
            (5.8, 4.8, 16.0),
            (6.8, 5.6, 24.0),
            (8.0, 6.7, 36.0),
            (9.5, 8.0, 54.0),
            (11.5, 9.5, 78.0),
        ]

        generated = 0
        x_values = [
            -59.0 + 2.15 * index
            for index in range(55)
        ]

        for x_index, base_x in enumerate(x_values):
            south = self.south_edge(base_x)
            north = self.north_edge(base_x)
            width = north - south
            row_count = max(
                5,
                int(width / 1.75),
            )

            for row in range(row_count):
                if generated >= self.building_target_count:
                    break

                fraction = (
                    (row + 0.5) / row_count
                )
                y = south + fraction * width
                x = (
                    base_x
                    + self.random.uniform(
                        -0.72,
                        0.72,
                    )
                )
                y += self.random.uniform(
                    -0.60,
                    0.60,
                )

                if not self.inside_peninsula(
                    x,
                    y,
                    margin_ratio=0.045,
                ):
                    continue

                if any(
                    math.hypot(
                        x - ex,
                        y - ey,
                    ) < radius
                    for ex, ey, radius in exclusions
                ):
                    continue

                centre = self.center_y(x)
                edge_ratio = min(
                    1.0,
                    abs(y - centre)
                    / max(1.0, self.half_width(x)),
                )

                central_core = math.exp(
                    -((x - 14.0) / 15.0) ** 2
                    - ((y - centre) / 12.0) ** 2
                )
                east_core = math.exp(
                    -((x - 46.0) / 10.0) ** 2
                    - ((y - centre) / 10.0) ** 2
                )
                west_core = math.exp(
                    -((x + 39.0) / 16.0) ** 2
                    - ((y - centre) / 14.0) ** 2
                )

                density = (
                    0.64
                    + 0.22 * central_core
                    + 0.14 * east_core
                    + 0.08 * west_core
                    - 0.10 * edge_ratio
                )
                if self.random.random() > density:
                    continue

                selector = (
                    self.random.random()
                    + 1.05 * central_core
                    + 0.72 * east_core
                    + 0.35 * west_core
                    - 0.35 * edge_ratio
                )

                if selector > 1.82:
                    class_index = 5
                elif selector > 1.45:
                    class_index = 4
                elif selector > 1.08:
                    class_index = 3
                elif selector > 0.70:
                    class_index = 2
                elif selector > 0.34:
                    class_index = 1
                else:
                    class_index = 0

                footprint_x, footprint_y, height = (
                    classes[class_index]
                )
                color_index = self.random.randrange(
                    len(colors)
                )
                key = (
                    class_index,
                    color_index,
                )

                if key not in groups:
                    marker = self._base_marker(
                        'generic_buildings',
                        start_id + len(groups),
                        Marker.CUBE_LIST,
                    )
                    marker.scale.x = footprint_x
                    marker.scale.y = footprint_y
                    marker.scale.z = height
                    self._set_color(
                        marker,
                        colors[color_index],
                    )
                    groups[key] = marker

                base = self.terrain_z(
                    x,
                    y,
                )
                groups[key].points.append(
                    self._point(
                        x,
                        y,
                        base + height / 2.0,
                    )
                )
                generated += 1

        self.generated_building_count = generated
        return (
            list(groups.values()),
            start_id + len(groups),
        )

    def _add_landmark_label(self, markers: List[Marker], marker_id: int, item: Dict[str, object], z: float) -> int:
        if self.show_labels:
            x, y = [float(value) for value in item['position']]
            markers.append(self._text('landmark_labels', marker_id, (x, y, z), str(item['label'])))
            marker_id += 1
        return marker_id

    def _landmark_markers(self, start_id: int) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id
        lookup = {str(item['id']): item for item in self.landmarks}

        def base_xy(identifier: str) -> Tuple[float, float, float]:
            item = lookup[identifier]
            x, y = [float(value) for value in item['position']]
            return x, y, self.terrain_z(x, y)

        # 鹅岭二厂：工业厂房、烟囱、观景平台。
        x, y, z = base_xy('eling_factory')
        markers += [
            self._cube('landmarks', marker_id, (x, y, z + 2.0), (6.5, 4.0, 4.0), (0.55, 0.30, 0.18, 1.0), 0.12),
            self._cube('landmarks', marker_id + 1, (x + 2.2, y - 2.1, z + 1.4), (3.2, 2.1, 2.8), (0.62, 0.37, 0.20, 1.0), 0.12),
            self._cylinder('landmarks', marker_id + 2, (x - 2.4, y + 1.2, z + 4.0), 0.8, 8.0, (0.32, 0.27, 0.25, 1.0)),
        ]
        marker_id += 3
        marker_id = self._add_landmark_label(markers, marker_id, lookup['eling_factory'], z + 9.0)

        # 李子坝：建筑分为上下两翼，轨道从中部穿过。
        x, y, z = base_xy('liziba')
        markers += [
            self._cube('landmarks', marker_id, (x, y, z + 6.0), (7.2, 4.8, 12.0), (0.58, 0.52, 0.47, 1.0)),
            self._cube('landmarks', marker_id + 1, (x, y, z + 6.2), (7.8, 1.4, 2.0), (0.10, 0.12, 0.15, 1.0)),
            self._line('rail', marker_id + 2, [(-58.0, 23.0, z + 6.2), (x, y, z + 6.2), (-37.0, 16.0, z + 7.2)], 0.65, (0.18, 0.72, 0.90, 1.0)),
        ]
        marker_id += 3
        marker_id = self._add_landmark_label(markers, marker_id, lookup['liziba'], z + 14.5)

        # 两路口与皇冠大扶梯：多层节点和斜向大扶梯。
        x, y, z = base_xy('lianglukou')
        markers += [
            self._cylinder('landmarks', marker_id, (x, y, z + 1.0), 7.0, 2.0, (0.35, 0.38, 0.42, 1.0)),
            self._line('elevated_roads', marker_id + 1, [(x - 8, y - 3, z + 3), (x, y, z + 4), (x + 8, y + 4, z + 5)], 1.1, (0.52, 0.54, 0.56, 1.0)),
            self._line('elevated_roads', marker_id + 2, [(x - 6, y + 6, z + 7), (x, y, z + 7), (x + 7, y - 6, z + 7)], 0.9, (0.68, 0.69, 0.70, 1.0)),
        ]
        marker_id += 3
        marker_id = self._add_landmark_label(markers, marker_id, lookup['lianglukou'], z + 10.0)
        x2, y2, z2 = base_xy('crown_escalator')
        markers.append(self._line('escalators', marker_id, [(x2 - 3.5, y2 - 2.0, z2 + 0.5), (x2 + 3.5, y2 + 2.0, z2 + 7.5)], 1.25, (0.95, 0.72, 0.20, 1.0)))
        marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['crown_escalator'], z2 + 9.0)

        # 人民大礼堂。
        x, y, z = base_xy('great_hall')
        markers += [
            self._cylinder('landmarks', marker_id, (x, y, z + 2.0), 7.0, 4.0, (0.15, 0.43, 0.35, 1.0)),
            self._cylinder('landmarks', marker_id + 1, (x, y, z + 5.0), 4.8, 2.0, (0.12, 0.34, 0.30, 1.0)),
            self._cylinder('landmarks', marker_id + 2, (x, y, z + 6.8), 2.6, 1.6, (0.72, 0.36, 0.20, 1.0)),
        ]
        marker_id += 3
        marker_id = self._add_landmark_label(markers, marker_id, lookup['great_hall'], z + 9.0)

        # 十八梯：阶梯折线和坡地旧街区。
        x, y, z = base_xy('shibati')
        stair_points: List[Point3] = []
        for index in range(15):
            ratio = index / 14.0
            sx = x - 4.5 + 9.0 * ratio
            sy = y - 3.2 + 6.4 * ratio
            stair_points.append((sx, sy, self.terrain_z(sx, sy) + 0.5 + 0.18 * index))
        markers.append(self._line('stairs', marker_id, stair_points, 1.0, (0.82, 0.66, 0.43, 1.0)))
        marker_id += 1
        for row in range(3):
            for col in range(4):
                bx = x - 4.5 + col * 2.8
                by = y + 3.2 - row * 2.8
                bz = self.terrain_z(bx, by)
                markers.append(self._cube('oldtown', marker_id, (bx, by, bz + 1.5), (2.2, 1.8, 3.0), (0.55, 0.34, 0.20, 1.0), 0.15))
                marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['shibati'], z + 9.0)

        # 较场口。
        x, y, z = base_xy('jiaochangkou')
        markers += [
            self._cylinder('landmarks', marker_id, (x, y, z + 0.3), 8.0, 0.6, (0.50, 0.49, 0.46, 1.0)),
            self._line('roads', marker_id + 1, [(x - 8, y - 5, z + 1.5), (x, y, z + 1.5), (x + 8, y + 4, z + 2.0)], 0.9, (0.24, 0.26, 0.29, 1.0)),
        ]
        marker_id += 2
        marker_id = self._add_landmark_label(markers, marker_id, lookup['jiaochangkou'], z + 6.5)

        # 白象居错层住宅。
        x, y, z = base_xy('baixiangju')
        for floor_group in range(4):
            markers.append(self._cube('landmarks', marker_id, (x + 0.7 * floor_group, y, z + 2.3 + 2.4 * floor_group), (7.0, 3.0, 4.2), (0.62, 0.56, 0.50, 1.0), -0.10))
            marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['baixiangju'], z + 13.5)

        # 解放碑纪念碑与商业核心高楼群。
        x, y, z = base_xy('jiefangbei')
        markers += [
            self._cube('landmarks', marker_id, (x, y, z + 0.35), (7.0, 7.0, 0.7), (0.56, 0.56, 0.53, 1.0)),
            self._cube('landmarks', marker_id + 1, (x, y, z + 3.5), (1.2, 1.2, 7.0), (0.80, 0.76, 0.66, 1.0)),
            self._cube('landmarks', marker_id + 2, (x + 4.0, y + 3.0, z + 8.0), (3.2, 3.0, 16.0), (0.30, 0.40, 0.52, 1.0)),
            self._cube('landmarks', marker_id + 3, (x - 4.0, y - 2.5, z + 7.0), (3.0, 3.2, 14.0), (0.38, 0.46, 0.56, 1.0)),
        ]
        marker_id += 4
        marker_id = self._add_landmark_label(markers, marker_id, lookup['jiefangbei'], z + 18.0)

        # 魁星楼：屋顶广场、悬空连廊、深谷立柱。
        x, y, z = base_xy('kuixinglou')
        platform_z = z + 7.5
        markers += [
            self._cube('landmarks', marker_id, (x, y, platform_z), (8.0, 6.0, 0.8), (0.62, 0.58, 0.50, 1.0)),
            self._cube('landmarks', marker_id + 1, (x - 2.7, y - 1.5, z + 3.8), (0.8, 0.8, 7.6), (0.40, 0.40, 0.42, 1.0)),
            self._cube('landmarks', marker_id + 2, (x + 2.7, y + 1.5, z + 3.8), (0.8, 0.8, 7.6), (0.40, 0.40, 0.42, 1.0)),
            self._line('skywalks', marker_id + 3, [(x - 8, y - 3, platform_z + 0.5), (x, y, platform_z + 0.5), (x + 6, y + 5, platform_z + 0.5)], 1.0, (0.82, 0.70, 0.36, 1.0)),
        ]
        marker_id += 4
        marker_id = self._add_landmark_label(markers, marker_id, lookup['kuixinglou'], platform_z + 3.0)

        # 洪崖洞：沿北坡多层堆叠吊脚楼。
        x, y, z = base_xy('hongyadong')
        for level in range(7):
            for col in range(4):
                bx = x - 4.5 + 3.0 * col
                by = y - 0.65 * level
                bz = self.terrain_z(bx, by) + 0.9 * level
                markers.append(self._cube('hongyadong', marker_id, (bx, by, bz + 1.2), (2.5, 1.8, 2.4), (0.68, 0.28 + 0.025 * level, 0.12, 1.0), 0.06))
                marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['hongyadong'], z + 14.0)

        # 千厮门大桥：桥面、桥塔、索线。
        bridge_z = 11.0
        bridge_points = [(30.0, 22.0, bridge_z), (34.0, 31.0, bridge_z + 1.0), (38.0, 43.0, bridge_z + 2.0)]
        markers.append(self._line('bridges', marker_id, bridge_points, 1.8, (0.80, 0.18, 0.12, 1.0)))
        marker_id += 1
        for bx, by, bz in [(32.4, 27.5, bridge_z + 5.0), (36.0, 38.0, bridge_z + 6.0)]:
            markers.append(self._cube('bridge_towers', marker_id, (bx, by, bz), (0.9, 0.9, 10.0), (0.85, 0.22, 0.15, 1.0)))
            marker_id += 1
        markers += [
            self._line('bridge_cables', marker_id, [(32.4, 27.5, bridge_z + 10), (30, 22, bridge_z), (34, 31, bridge_z + 1)], 0.18, (0.92, 0.72, 0.62, 1.0)),
            self._line('bridge_cables', marker_id + 1, [(36, 38, bridge_z + 11), (34, 31, bridge_z + 1), (38, 43, bridge_z + 2)], 0.18, (0.92, 0.72, 0.62, 1.0)),
        ]
        marker_id += 2
        marker_id = self._add_landmark_label(markers, marker_id, lookup['qiansimen'], bridge_z + 14.5)

        # 朝天门与来福士帆形塔楼。
        x, y, z = base_xy('raffles')
        tower_offsets = [(-3.8, -2.8), (-1.2, 2.7), (1.7, -2.5), (4.1, 2.0)]
        for index, (dx, dy) in enumerate(tower_offsets):
            height = 16.0 + 2.0 * index
            markers.append(self._cube('raffles', marker_id, (x + dx, y + dy, z + height / 2.0), (2.2, 2.5, height), (0.25, 0.42, 0.55, 1.0), 0.20))
            marker_id += 1
        markers.append(self._cube('raffles', marker_id, (x + 0.2, y + 0.2, z + 15.5), (11.5, 2.3, 2.0), (0.47, 0.65, 0.73, 1.0), 0.20))
        marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['raffles'], z + 23.0)
        x2, y2, z2 = base_xy('chaotianmen')
        markers.append(self._line('plazas', marker_id, [(x2 - 5, y2 - 5, z2 + 0.4), (x2 + 4, y2 - 2, z2 + 0.4), (x2 + 5, y2 + 3, z2 + 0.4), (x2 - 3, y2 + 5, z2 + 0.4)], 1.2, (0.82, 0.80, 0.74, 1.0), True))
        marker_id += 1
        marker_id = self._add_landmark_label(markers, marker_id, lookup['chaotianmen'], z2 + 11.0)

        # 长江索道。
        cable_z = 18.0
        markers += [
            self._line('cableway', marker_id, [(26.0, -17.0, cable_z), (33.0, -31.0, cable_z + 1.5), (39.0, -45.0, cable_z + 2.5)], 0.22, (0.92, 0.85, 0.32, 1.0)),
            self._cube('cableway', marker_id + 1, (32.0, -29.0, cable_z + 1.4), (2.0, 1.1, 1.1), (0.88, 0.25, 0.16, 1.0), -1.1),
        ]
        marker_id += 2
        marker_id = self._add_landmark_label(markers, marker_id, lookup['cableway'], cable_z + 5.0)

        return markers, marker_id



    def _transport_markers(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id

        road_color = (
            0.21,
            0.23,
            0.26,
            1.0,
        )
        elevated_color = (
            0.62,
            0.63,
            0.65,
            1.0,
        )
        rail_color = (
            0.08,
            0.69,
            0.91,
            1.0,
        )

        road_fractions = [
            0.0,
            0.43,
            -0.43,
            0.74,
            -0.74,
        ]
        road_names = [
            'ridge_road',
            'north_terrace_road',
            'south_terrace_road',
            'north_riverside_road',
            'south_riverside_road',
        ]
        road_widths = [
            1.55,
            1.25,
            1.25,
            1.00,
            1.00,
        ]

        for fraction, namespace, width in zip(
            road_fractions,
            road_names,
            road_widths,
        ):
            points: List[Point3] = []
            for index in range(72):
                x = (
                    -59.0
                    + 116.0 * index / 71.0
                )
                y = self.corridor_y(
                    x,
                    fraction,
                )
                points.append((
                    x,
                    y,
                    self.terrain_z(
                        x,
                        y,
                    ) + 1.25,
                ))
            markers.append(
                self._line(
                    namespace,
                    marker_id,
                    points,
                    width,
                    road_color,
                )
            )
            marker_id += 1

        elevated_routes = [
            [
                (-49.0, -0.58, 6.0),
                (-42.0, -0.15, 9.0),
                (-34.0, 0.35, 12.0),
            ],
            [
                (-31.0, -0.70, 5.5),
                (-21.0, -0.25, 9.0),
                (-10.0, 0.30, 12.5),
            ],
            [
                (-10.0, -0.72, 6.0),
                (1.0, -0.28, 10.5),
                (14.0, 0.24, 14.0),
            ],
            [
                (8.0, 0.70, 6.5),
                (20.0, 0.34, 11.0),
                (33.0, -0.08, 15.0),
            ],
            [
                (21.0, -0.70, 5.5),
                (34.0, -0.35, 10.5),
                (49.0, 0.02, 14.5),
            ],
            [
                (-8.0, 0.74, 6.0),
                (4.0, 0.40, 10.0),
                (17.0, 0.08, 13.0),
            ],
            [
                (29.0, 0.72, 7.0),
                (40.0, 0.36, 12.0),
                (52.0, 0.04, 16.0),
            ],
        ]

        for route in elevated_routes:
            points: List[Point3] = []
            for x, fraction, lift in route:
                y = self.corridor_y(
                    x,
                    fraction,
                )
                points.append((
                    x,
                    y,
                    self.terrain_z(
                        x,
                        y,
                    ) + lift,
                ))
            markers.append(
                self._line(
                    'elevated_roads',
                    marker_id,
                    points,
                    1.35,
                    elevated_color,
                )
            )
            marker_id += 1

        rail_points: List[Point3] = []
        for index in range(64):
            x = (
                -59.0
                + 98.0 * index / 63.0
            )
            y = self.corridor_y(
                x,
                0.68,
            )
            rail_points.append((
                x,
                y,
                self.terrain_z(
                    x,
                    y,
                ) + 7.5,
            ))

        markers.append(
            self._line(
                'rail',
                marker_id,
                rail_points,
                0.70,
                rail_color,
            )
        )
        marker_id += 1

        return markers, marker_id

    def _route_markers(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id

        for route in self.routes:
            color_values = [
                float(value)
                for value in route.get(
                    'color',
                    [1, 1, 0, 1],
                )
            ]
            color: Color = (
                color_values[0],
                color_values[1],
                color_values[2],
                color_values[3],
            )

            # The third value in route YAML is altitude above local
            # ground, not absolute map Z.  This prevents paths from
            # disappearing inside the enlarged mountain terrain.
            points: List[Point3] = []
            for raw_point in route.get('points', []):
                x, y, altitude_agl = [
                    float(value)
                    for value in raw_point
                ]
                points.append((
                    x,
                    y,
                    self.terrain_z(x, y)
                    + altitude_agl,
                ))

            markers.append(
                self._line(
                    'showcase_routes',
                    marker_id,
                    points,
                    0.62,
                    color,
                )
            )
            marker_id += 1

            for point in points:
                markers.append(
                    self._cylinder(
                        'route_waypoints',
                        marker_id,
                        point,
                        1.15,
                        1.0,
                        color,
                    )
                )
                marker_id += 1

            if self.show_labels and points:
                markers.append(
                    self._text(
                        'route_labels',
                        marker_id,
                        (
                            points[0][0],
                            points[0][1],
                            points[0][2] + 3.0,
                        ),
                        str(
                            route.get(
                                'label',
                                route.get('id', 'route'),
                            )
                        ),
                        1.0,
                    )
                )
                marker_id += 1

        return markers, marker_id



    def _contour_markers(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id

        for fraction in (
            -0.82,
            -0.65,
            -0.46,
            -0.24,
            0.0,
            0.24,
            0.46,
            0.65,
            0.82,
        ):
            points: List[Point3] = []

            for index in range(100):
                x = (
                    -60.0
                    + 120.0 * index / 99.0
                )
                y = self.corridor_y(
                    x,
                    fraction,
                )
                points.append((
                    x,
                    y,
                    self.terrain_z(
                        x,
                        y,
                    ) + 0.30,
                ))

            alpha = (
                0.82
                if abs(fraction) in (
                    0.46,
                    0.65,
                )
                else 0.54
            )
            markers.append(
                self._line(
                    'terrain_contours',
                    marker_id,
                    points,
                    0.19,
                    (
                        0.68,
                        0.51,
                        0.30,
                        alpha,
                    ),
                )
            )
            marker_id += 1

        return markers, marker_id


    def _cliff_face_markers(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id

        for upper_fraction, lower_fraction, namespace, color in (
            (
                0.58,
                0.94,
                'north_cliff_faces',
                (0.35, 0.27, 0.19, 1.0),
            ),
            (
                -0.60,
                -0.94,
                'south_cliff_faces',
                (0.31, 0.25, 0.19, 1.0),
            ),
        ):
            marker = self._base_marker(
                namespace,
                marker_id,
                Marker.TRIANGLE_LIST,
            )
            self._set_color(
                marker,
                color,
            )

            for index in range(99):
                x0 = (
                    -60.0
                    + 120.0 * index / 99.0
                )
                x1 = (
                    -60.0
                    + 120.0 * (index + 1) / 99.0
                )

                upper0_y = self.corridor_y(
                    x0,
                    upper_fraction,
                )
                upper1_y = self.corridor_y(
                    x1,
                    upper_fraction,
                )
                lower0_y = self.corridor_y(
                    x0,
                    lower_fraction,
                )
                lower1_y = self.corridor_y(
                    x1,
                    lower_fraction,
                )

                points = (
                    (
                        x0,
                        upper0_y,
                        self.terrain_z(
                            x0,
                            upper0_y,
                        ),
                    ),
                    (
                        x0,
                        lower0_y,
                        self.terrain_z(
                            x0,
                            lower0_y,
                        ),
                    ),
                    (
                        x1,
                        lower1_y,
                        self.terrain_z(
                            x1,
                            lower1_y,
                        ),
                    ),
                    (
                        x0,
                        upper0_y,
                        self.terrain_z(
                            x0,
                            upper0_y,
                        ),
                    ),
                    (
                        x1,
                        lower1_y,
                        self.terrain_z(
                            x1,
                            lower1_y,
                        ),
                    ),
                    (
                        x1,
                        upper1_y,
                        self.terrain_z(
                            x1,
                            upper1_y,
                        ),
                    ),
                )

                for point in points:
                    marker.points.append(
                        self._point(*point)
                    )

            markers.append(marker)
            marker_id += 1

        return markers, marker_id


    def _mountain_structure_markers(
        self,
        start_id: int,
    ) -> Tuple[List[Marker], int]:
        markers: List[Marker] = []
        marker_id = start_id
        connector_color = (
            0.95,
            0.68,
            0.18,
            1.0,
        )
        wall_color = (
            0.43,
            0.36,
            0.28,
            1.0,
        )

        connector_specs = [
            (-51.0, 1.0),
            (-40.0, -1.0),
            (-30.0, 1.0),
            (-18.0, -1.0),
            (-6.0, -1.0),
            (8.0, 1.0),
            (20.0, 1.0),
            (30.0, -1.0),
            (40.0, 1.0),
            (50.0, -1.0),
        ]

        for x, side in connector_specs:
            fractions = (
                (0.86, 0.65, 0.42, 0.18)
                if side > 0
                else (-0.86, -0.65, -0.42, -0.18)
            )
            points: List[Point3] = []
            for index, fraction in enumerate(
                fractions
            ):
                y = self.corridor_y(
                    x,
                    fraction,
                )
                points.append((
                    x + 0.65 * index,
                    y,
                    self.terrain_z(
                        x,
                        y,
                    ) + 0.9,
                ))
            markers.append(
                self._line(
                    'vertical_connectors',
                    marker_id,
                    points,
                    1.00,
                    connector_color,
                )
            )
            marker_id += 1

        # Retaining walls expose the stacked terrace structure.
        for fraction in (
            -0.66,
            -0.43,
            0.43,
            0.66,
        ):
            marker = self._base_marker(
                'retaining_walls',
                marker_id,
                Marker.TRIANGLE_LIST,
            )
            self._set_color(
                marker,
                wall_color,
            )
            for index in range(72):
                x0 = (
                    -58.0
                    + 114.0 * index / 72.0
                )
                x1 = (
                    -58.0
                    + 114.0 * (index + 1) / 72.0
                )
                y0 = self.corridor_y(
                    x0,
                    fraction,
                )
                y1 = self.corridor_y(
                    x1,
                    fraction,
                )
                top0 = self.terrain_z(
                    x0,
                    y0,
                ) + 0.4
                top1 = self.terrain_z(
                    x1,
                    y1,
                ) + 0.4
                low0 = top0 - 4.0
                low1 = top1 - 4.0
                for point in (
                    (x0, y0, top0),
                    (x0, y0, low0),
                    (x1, y1, low1),
                    (x0, y0, top0),
                    (x1, y1, low1),
                    (x1, y1, top1),
                ):
                    marker.points.append(
                        self._point(*point)
                    )
            markers.append(marker)
            marker_id += 1

        deck_specs = [
            (-46.0, 0.24, 13.0, 6.0),
            (-30.0, -0.18, 13.0, 5.5),
            (-12.0, -0.32, 14.0, 6.0),
            (5.0, 0.26, 15.0, 6.5),
            (20.0, 0.40, 15.0, 6.5),
            (34.0, 0.54, 15.0, 6.0),
        ]

        for x, fraction, length, width in deck_specs:
            y = self.corridor_y(
                x,
                fraction,
            )
            base = self.terrain_z(
                x,
                y,
            )
            markers.append(
                self._cube(
                    'rooftop_decks',
                    marker_id,
                    (
                        x,
                        y,
                        base + 8.0,
                    ),
                    (
                        length,
                        width,
                        0.85,
                    ),
                    (
                        0.55,
                        0.52,
                        0.47,
                        1.0,
                    ),
                    yaw=0.10,
                    height_scale=1.0,
                )
            )
            marker_id += 1

        return markers, marker_id

    def _build_scene(self) -> MarkerArray:
        markers: List[Marker] = []
        marker_id = 0

        markers.extend(
            self._water_markers(marker_id)
        )
        marker_id += 3

        markers.append(
            self._terrain_marker(marker_id)
        )
        marker_id += 1

        cliffs, marker_id = (
            self._cliff_face_markers(marker_id)
        )
        markers.extend(cliffs)

        shoreline = self._shoreline_markers(
            marker_id
        )
        markers.extend(shoreline)
        marker_id += len(shoreline)

        contours, marker_id = (
            self._contour_markers(marker_id)
        )
        markers.extend(contours)

        buildings, marker_id = (
            self._generate_buildings(marker_id)
        )
        markers.extend(buildings)

        mountain_structures, marker_id = (
            self._mountain_structure_markers(
                marker_id
            )
        )
        markers.extend(mountain_structures)

        transport, marker_id = (
            self._transport_markers(marker_id)
        )
        markers.extend(transport)

        landmarks, marker_id = (
            self._landmark_markers(marker_id)
        )
        markers.extend(landmarks)

        routes, marker_id = (
            self._route_markers(marker_id)
        )
        markers.extend(routes)

        title = self._text(
            'scene_title',
            marker_id,
            (-6.0, 0.0, 125.0),
            '重庆渝中半岛立体山城全景 V3  '
            'Solid Yuzhong Mountain City',
            2.4,
        )
        markers.append(title)

        return MarkerArray(markers=markers)



    def _build_manifest(self) -> Dict[str, object]:
        ridge_samples = [
            self.terrain_z(
                x,
                self.center_y(x),
            )
            for x in (
                -55.0,
                -42.0,
                -25.0,
                -5.0,
                15.0,
                35.0,
                52.0,
            )
        ]
        shore_samples = []
        for x in (
            -55.0,
            -35.0,
            -10.0,
            15.0,
            35.0,
            52.0,
        ):
            shore_samples.extend([
                self.terrain_z(
                    x,
                    self.north_edge(x),
                ),
                self.terrain_z(
                    x,
                    self.south_edge(x),
                ),
            ])

        return {
            'scene_revision': SCENE_REVISION,
            'scene_name': self.scene.get('name'),
            'frame_id': self.frame_id,
            'rendered_extent_m': {
                'east_west': (
                    124.0 * self.position_scale_x
                ),
                'north_south': (
                    63.5 * self.position_scale_y
                ),
            },
            'estimated_relief_m': (
                max(ridge_samples)
                - min(shore_samples)
            ),
            'terrain_is_closed_solid': True,
            'shoreline_model': (
                'asymmetric hand-shaped Yuzhong peninsula outline'
            ),
            'generic_building_count': (
                self.generated_building_count
            ),
            'generic_max_building_height_m': 78.0,
            'landmark_height_scale': (
                self.object_height_scale
            ),
            'landmark_count': len(self.landmarks),
            'route_count': len(self.routes),
            'marker_count': len(
                self.marker_array.markers
            ),
            'landmarks': [
                {
                    'id': item.get('id'),
                    'label': item.get('label'),
                    'kind': item.get('kind'),
                    'position': item.get('position'),
                }
                for item in self.landmarks
            ],
            'design_notes': [
                'The peninsula is asymmetric and tapers to the Chaotianmen confluence instead of using a symmetric half-width capsule.',
                'The terrain is a closed solid with top, bottom, north/south walls and west/east end caps.',
                'The ridge rises roughly 55–80 m above the river and supports dense stepped buildings.',
                'Generic buildings use 10–78 m height classes and efficient CUBE_LIST batching.',
                'This remains a stylized visual reconstruction; GIS-accurate OSM and DEM ingestion is the next milestone.',
            ],
        }

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.marker_publisher.publish(self.marker_array)
        message = String()
        message.data = json.dumps(self.manifest, ensure_ascii=False, sort_keys=True)
        self.manifest_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneServer()
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
