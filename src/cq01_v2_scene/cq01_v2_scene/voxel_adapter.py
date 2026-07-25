#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import struct
from typing import Dict, Iterable, Sequence, Set, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from cq01_v2_scene.scene_publisher import ScenePublisher

Voxel = Tuple[int, int, int]
Point3 = Tuple[float, float, float]


def make_cloud_from_voxels(
    voxels: Iterable[Voxel],
    bounds: Dict[str, float],
    resolution: float,
    frame_id: str,
    stamp,
) -> PointCloud2:
    ordered = sorted(voxels)
    data = bytearray(len(ordered) * 12)
    min_x = float(bounds['min_x'])
    min_y = float(bounds['min_y'])
    min_z = float(bounds['min_z'])
    for index, (ix, iy, iz) in enumerate(ordered):
        x = min_x + (ix + 0.5) * resolution
        y = min_y + (iy + 0.5) * resolution
        z = min_z + (iz + 0.5) * resolution
        struct.pack_into('<fff', data, index * 12, x, y, z)

    message = PointCloud2()
    message.header = Header(frame_id=frame_id, stamp=stamp)
    message.height = 1
    message.width = len(ordered)
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = bytes(data)
    return message


class CQ01VoxelAdapter(Node):
    """Convert CQ-01 V3.4 geometry into the legacy voxel interface.

    V3.4C enforces visual/planning consistency: every displayed building is
    included in the inflated planning occupancy.  The coarse 3 m lattice keeps
    the 430 x 320 x 300 m scene tractable while preserving the gorge, bridge,
    terraces and all 133 visible structures.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v34c_voxel_adapter')
        share = get_package_share_directory('cq01_v2_scene')
        self.declare_parameter(
            'scene_file',
            os.path.join(share, 'config', 'cq01_v34_scene.yaml'),
        )
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution_m', 3.0)
        self.declare_parameter('inflation_radius_m', 1.4)
        self.declare_parameter('publish_period_s', 5.0)
        self.declare_parameter('raw_cloud_topic', '/drone/voxel_cloud_raw')
        self.declare_parameter('inflated_cloud_topic', '/drone/voxel_cloud')
        self.declare_parameter('metadata_topic', '/drone/voxel_metadata')
        self.declare_parameter('include_visual_only_boxes', True)

        scene_file = str(self.get_parameter('scene_file').value)
        with open(scene_file, 'r', encoding='utf-8') as handle:
            self.scene: Dict = yaml.safe_load(handle)['scene']

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.resolution = float(self.get_parameter('resolution_m').value)
        self.inflation_radius = float(
            self.get_parameter('inflation_radius_m').value
        )
        self.include_visual_only_boxes = bool(
            self.get_parameter('include_visual_only_boxes').value
        )
        if self.resolution <= 0.0:
            raise ValueError('resolution_m must be positive')
        if self.inflation_radius < 0.0:
            raise ValueError('inflation_radius_m must be non-negative')

        source_bounds = self.scene['world_bounds']
        self.bounds = {
            'min_x': float(source_bounds['x_min']),
            'max_x': float(source_bounds['x_max']),
            'min_y': float(source_bounds['y_min']),
            'max_y': float(source_bounds['y_max']),
            'min_z': float(source_bounds['z_min']),
            'max_z': float(source_bounds['z_max']),
        }
        self.dimensions = (
            int(math.ceil(
                (self.bounds['max_x'] - self.bounds['min_x'])
                / self.resolution
            )),
            int(math.ceil(
                (self.bounds['max_y'] - self.bounds['min_y'])
                / self.resolution
            )),
            int(math.ceil(
                (self.bounds['max_z'] - self.bounds['min_z'])
                / self.resolution
            )),
        )

        # Reuse the exact V3.4 terrain/box geometry without constructing the
        # visualization node or duplicating its mathematical model.
        self.geometry = ScenePublisher.__new__(ScenePublisher)
        self.geometry.scene = self.scene

        self.raw_occupied = self._build_occupancy(0.0)
        self.inflated_occupied = self._build_occupancy(
            self.inflation_radius
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.raw_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('raw_cloud_topic').value),
            qos,
        )
        self.inflated_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('inflated_cloud_topic').value),
            qos,
        )
        self.metadata_publisher = self.create_publisher(
            String,
            str(self.get_parameter('metadata_topic').value),
            qos,
        )

        self.metadata = self._build_metadata()
        initial_stamp = self.get_clock().now().to_msg()
        self.raw_cloud = make_cloud_from_voxels(
            self.raw_occupied,
            self.bounds,
            self.resolution,
            self.frame_id,
            initial_stamp,
        )
        self.inflated_cloud = make_cloud_from_voxels(
            self.inflated_occupied,
            self.bounds,
            self.resolution,
            self.frame_id,
            initial_stamp,
        )
        period = max(
            1.0,
            float(self.get_parameter('publish_period_s').value),
        )
        self.timer = self.create_timer(period, self.publish_all)
        self.publish_all()
        self.get_logger().info(
            'CQ-01 V3.4C visual-collision-consistent voxel map ready: '
            f'resolution={self.resolution:.2f} m, '
            f'dimensions={self.dimensions}, '
            f'raw={len(self.raw_occupied)}, '
            f'inflated={len(self.inflated_occupied)}, '
            f'planning_boxes='
            f'{len(self.scene.get("collision_boxes", [])) + (len(self.scene.get("visual_only_boxes", [])) if self.include_visual_only_boxes else 0)}, '
            f'visual_consistent={self.include_visual_only_boxes}'
        )

    def _voxel_center(self, voxel: Voxel) -> Point3:
        return (
            self.bounds['min_x']
            + (voxel[0] + 0.5) * self.resolution,
            self.bounds['min_y']
            + (voxel[1] + 0.5) * self.resolution,
            self.bounds['min_z']
            + (voxel[2] + 0.5) * self.resolution,
        )

    def _index_range(
        self,
        minimum: float,
        maximum: float,
        axis: int,
    ) -> range:
        key = ('min_x', 'min_y', 'min_z')[axis]
        dimension = self.dimensions[axis]
        first = max(
            0,
            int(math.floor(
                (minimum - self.bounds[key]) / self.resolution
            )),
        )
        last = min(
            dimension - 1,
            int(math.floor(
                (maximum - self.bounds[key]) / self.resolution
            )),
        )
        return range(first, last + 1)

    def _build_occupancy(self, margin: float) -> Set[Voxel]:
        occupied: Set[Voxel] = set()
        self._voxelize_filled_terrain(occupied, margin)
        planning_boxes = list(self.scene.get('collision_boxes', []))
        if self.include_visual_only_boxes:
            planning_boxes.extend(self.scene.get('visual_only_boxes', []))
        for spec in planning_boxes:
            self._voxelize_box(occupied, spec, margin)
        return occupied

    def _voxelize_filled_terrain(
        self,
        occupied: Set[Voxel],
        margin: float,
    ) -> None:
        nx, ny, nz = self.dimensions
        for ix in range(nx):
            x = self.bounds['min_x'] + (ix + 0.5) * self.resolution
            for iy in range(ny):
                y = (
                    self.bounds['min_y']
                    + (iy + 0.5) * self.resolution
                )
                terrain_top = (
                    self.geometry.terrain_height(x, y) + margin
                )
                maximum_iz = min(
                    nz - 1,
                    int(math.floor(
                        (terrain_top - self.bounds['min_z'])
                        / self.resolution
                    )),
                )
                for iz in range(maximum_iz + 1):
                    z = (
                        self.bounds['min_z']
                        + (iz + 0.5) * self.resolution
                    )
                    if z <= terrain_top:
                        occupied.add((ix, iy, iz))

    def _voxelize_box(
        self,
        occupied: Set[Voxel],
        spec: Dict,
        margin: float,
    ) -> None:
        center, size, yaw = self.geometry.resolve_box(spec)
        cx, cy, cz = center
        half_x = 0.5 * size[0] + margin
        half_y = 0.5 * size[1] + margin
        half_z = 0.5 * size[2] + margin

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        extent_x = abs(cos_yaw) * half_x + abs(sin_yaw) * half_y
        extent_y = abs(sin_yaw) * half_x + abs(cos_yaw) * half_y

        for ix in self._index_range(cx - extent_x, cx + extent_x, 0):
            x = self.bounds['min_x'] + (ix + 0.5) * self.resolution
            for iy in self._index_range(cy - extent_y, cy + extent_y, 1):
                y = self.bounds['min_y'] + (iy + 0.5) * self.resolution
                dx = x - cx
                dy = y - cy
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                if abs(local_x) > half_x or abs(local_y) > half_y:
                    continue
                for iz in self._index_range(cz - half_z, cz + half_z, 2):
                    z = (
                        self.bounds['min_z']
                        + (iz + 0.5) * self.resolution
                    )
                    if abs(z - cz) <= half_z:
                        occupied.add((ix, iy, iz))

    def _build_metadata(self) -> String:
        total_voxels = (
            self.dimensions[0]
            * self.dimensions[1]
            * self.dimensions[2]
        )
        data = {
            'scenario': 'cq01_v34c_conservative_clearance_planning',
            'frame_id': self.frame_id,
            'resolution_m': self.resolution,
            'inflation_radius_m': self.inflation_radius,
            'bounds': self.bounds,
            'dimensions': {
                'x': self.dimensions[0],
                'y': self.dimensions[1],
                'z': self.dimensions[2],
            },
            'base_occupied_voxels': len(self.raw_occupied),
            'inflated_occupied_voxels': len(self.inflated_occupied),
            'free_voxels': total_voxels - len(self.inflated_occupied),
            'slice_z_m': 30.0,
            'source_scene': str(self.scene.get('name', 'CQ-01 V3.4')),
            'terrain_model': str(
                self.scene.get('terrain', {}).get('model', '')
            ),
            'collision_box_count': len(
                self.scene.get('collision_boxes', [])
            ),
            'visual_only_box_count': len(
                self.scene.get('visual_only_boxes', [])
            ),
            'planning_box_count': (
                len(self.scene.get('collision_boxes', []))
                + (len(self.scene.get('visual_only_boxes', []))
                   if self.include_visual_only_boxes else 0)
            ),
            'include_visual_only_boxes': self.include_visual_only_boxes,
            'visual_collision_consistent': self.include_visual_only_boxes,
            'adapter_profile': '2m_filled_terrain_all_visible_boxes_3p5m_inflation',
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
        self.raw_publisher.publish(self.raw_cloud)
        self.inflated_publisher.publish(self.inflated_cloud)
        self.metadata_publisher.publish(self.metadata)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01VoxelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
