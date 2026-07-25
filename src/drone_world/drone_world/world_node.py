#!/usr/bin/env python3
from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from drone_world.geometry import (
    expand_obstacles,
    generate_random_obstacles,
    obstacle_contains_xy_at_z,
    sample_obstacle_surface,
)


class WorldServer(Node):
    def __init__(self) -> None:
        super().__init__('world_server')
        self.declare_parameter('scenario', 'warehouse')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('map_topic', '/drone/obstacle_grid')
        self.declare_parameter('marker_topic', '/drone/obstacle_markers')
        self.declare_parameter('pointcloud_topic', '/drone/obstacle_cloud')
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('point_spacing_m', 0.35)
        self.declare_parameter('show_labels', True)

        self.scenario_name = str(self.get_parameter('scenario').value)
        scenario_file = str(self.get_parameter('scenario_file').value)
        self.configuration = self._load_configuration(scenario_file)
        self.world = self.configuration['world']
        self.frame_id = str(self.world.get('frame_id', 'map'))
        raw = list(self.world.get('obstacles', []))
        raw += generate_random_obstacles(dict(self.world.get('random', {})))
        self.obstacles = expand_obstacles(raw)
        self.point_spacing = float(self.get_parameter('point_spacing_m').value)
        self.show_labels = bool(self.get_parameter('show_labels').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            qos,
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('pointcloud_topic').value),
            qos,
        )

        self.occupancy_grid = self._build_occupancy_grid()
        self.marker_array = self._build_marker_array()
        self.point_cloud = self._build_point_cloud()

        publish_hz = max(0.2, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_world)
        self._publish_world()

        self.get_logger().info(
            f'World ready: scenario={self.scenario_name}, '
            f'obstacles={len(self.obstacles)}, '
            f'cloud_points={self.point_cloud.width}'
        )

    def _load_configuration(self, scenario_file: str) -> Dict[str, object]:
        if scenario_file:
            path = Path(scenario_file).expanduser()
        else:
            path = Path(get_package_share_directory('drone_world')) / 'config' / f'{self.scenario_name}.yaml'
        if not path.exists():
            raise FileNotFoundError(f'Scenario file not found: {path}')
        with path.open('r', encoding='utf-8') as handle:
            configuration = yaml.safe_load(handle)
        if not isinstance(configuration, dict) or 'world' not in configuration:
            raise ValueError(f'Invalid scenario configuration: {path}')
        self.get_logger().info(f'Loaded scenario: {path}')
        return configuration

    def _build_occupancy_grid(self) -> OccupancyGrid:
        bounds = self.world['bounds']
        resolution = float(self.world.get('resolution', 0.2))
        planning_z = float(self.world.get('planning_z', 1.5))
        min_x = float(bounds['min_x'])
        max_x = float(bounds['max_x'])
        min_y = float(bounds['min_y'])
        max_y = float(bounds['max_y'])
        width = int(math.ceil((max_x - min_x) / resolution))
        height = int(math.ceil((max_y - min_y) / resolution))

        message = OccupancyGrid()
        message.header.frame_id = self.frame_id
        message.info.resolution = resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = min_x
        message.info.origin.position.y = min_y
        message.info.origin.orientation.w = 1.0
        data: List[int] = [0] * (width * height)
        for grid_y in range(height):
            world_y = min_y + (grid_y + 0.5) * resolution
            for grid_x in range(width):
                world_x = min_x + (grid_x + 0.5) * resolution
                if any(
                    obstacle_contains_xy_at_z(obstacle, world_x, world_y, planning_z)
                    for obstacle in self.obstacles
                ):
                    data[grid_y * width + grid_x] = 100
        message.data = data
        return message

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    @staticmethod
    def _category_color(obstacle: Dict[str, object]) -> Tuple[float, float, float, float]:
        category = str(obstacle.get('category', obstacle.get('type', 'obstacle')))
        colors = {
            'shelf': (0.55, 0.35, 0.15, 0.95),
            'wall': (0.65, 0.65, 0.70, 0.95),
            'doorway': (0.30, 0.45, 0.75, 0.95),
            'column': (0.50, 0.50, 0.55, 0.95),
            'beam': (0.75, 0.45, 0.15, 0.95),
            'tree_trunk': (0.35, 0.20, 0.08, 0.95),
            'tree_crown': (0.10, 0.55, 0.18, 0.80),
            'building': (0.40, 0.48, 0.62, 0.95),
            'bridge': (0.35, 0.35, 0.38, 0.95),
            'terrain': (0.40, 0.30, 0.20, 0.95),
            'river': (0.10, 0.35, 0.75, 0.55),
            'random': (0.65, 0.28, 0.25, 0.90),
            'tunnel': (0.48, 0.48, 0.52, 0.95),
        }
        return colors.get(category, (0.75, 0.20, 0.20, 0.90))

    @staticmethod
    def _obstacle_top_z(obstacle: Dict[str, object]) -> float:
        shape = str(obstacle.get('shape', obstacle.get('type')))
        center_z = float(obstacle['center'][2])
        if shape == 'box':
            return center_z + float(obstacle['size'][2]) / 2.0
        if shape == 'cylinder':
            return center_z + float(obstacle['height']) / 2.0
        if shape == 'sphere':
            return center_z + float(obstacle['radius'])
        return center_z

    def _build_marker_array(self) -> MarkerArray:
        array = MarkerArray()
        marker_id = 0
        for obstacle in self.obstacles:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = 'world_obstacles'
            marker.id = marker_id
            marker.action = Marker.ADD
            marker.pose.position.x = float(obstacle['center'][0])
            marker.pose.position.y = float(obstacle['center'][1])
            marker.pose.position.z = float(obstacle['center'][2])
            marker.pose.orientation = self._quaternion_from_yaw(float(obstacle.get('yaw', 0.0)))
            shape = str(obstacle.get('shape', obstacle.get('type')))
            if shape == 'box':
                marker.type = Marker.CUBE
                size = [float(v) for v in obstacle['size']]
                marker.scale.x, marker.scale.y, marker.scale.z = size
            elif shape == 'cylinder':
                marker.type = Marker.CYLINDER
                radius = float(obstacle['radius'])
                marker.scale.x = marker.scale.y = 2.0 * radius
                marker.scale.z = float(obstacle['height'])
            elif shape == 'sphere':
                marker.type = Marker.SPHERE
                diameter = 2.0 * float(obstacle['radius'])
                marker.scale.x = marker.scale.y = marker.scale.z = diameter
            else:
                raise ValueError(f'Unsupported marker shape: {shape}')
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self._category_color(obstacle)
            array.markers.append(marker)
            marker_id += 1

            if self.show_labels:
                text = Marker()
                text.header.frame_id = self.frame_id
                text.ns = 'world_labels'
                text.id = marker_id
                text.action = Marker.ADD
                text.type = Marker.TEXT_VIEW_FACING
                text.pose.position.x = float(obstacle['center'][0])
                text.pose.position.y = float(obstacle['center'][1])
                text.pose.position.z = self._obstacle_top_z(obstacle) + 0.25
                text.pose.orientation.w = 1.0
                text.scale.z = 0.24
                text.color.r = text.color.g = text.color.b = 0.95
                text.color.a = 0.95
                text.text = str(obstacle['id'])
                array.markers.append(text)
                marker_id += 1

        bounds = self.world['bounds']
        outline = Marker()
        outline.header.frame_id = self.frame_id
        outline.ns = 'world_bounds'
        outline.id = marker_id
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.05
        outline.color.r = 0.95
        outline.color.g = 0.85
        outline.color.b = 0.15
        outline.color.a = 0.90
        min_x, max_x = float(bounds['min_x']), float(bounds['max_x'])
        min_y, max_y = float(bounds['min_y']), float(bounds['max_y'])
        for x, y in [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)]:
            point = Point(x=x, y=y, z=0.02)
            outline.points.append(point)
        array.markers.append(outline)
        return array

    def _build_point_cloud(self) -> PointCloud2:
        points: List[Tuple[float, float, float]] = []
        for obstacle in self.obstacles:
            if bool(obstacle.get('publish_cloud', True)):
                points.extend(sample_obstacle_surface(obstacle, self.point_spacing))
        message = PointCloud2()
        message.header = Header(frame_id=self.frame_id)
        message.height = 1
        message.width = len(points)
        message.is_bigendian = False
        message.is_dense = True
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = b''.join(struct.pack('<fff', *point) for point in points)
        return message

    def _publish_world(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.occupancy_grid.header.stamp = stamp
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.point_cloud.header.stamp = stamp
        self.map_publisher.publish(self.occupancy_grid)
        self.marker_publisher.publish(self.marker_array)
        self.pointcloud_publisher.publish(self.point_cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
