#!/usr/bin/env python3
"""CQ-01 presentation layer.

This node leaves all physical/planning topics unchanged. It publishes a
display-only Z-exaggerated scene, Z-exaggerated paths and an odometry-driven
quadrotor model using PointCloud2. PointCloud2 is used deliberately because
it renders reliably in the current VMware/RViz environment.
"""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def volatile_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class CQ01PresentationNode(Node):
    """Publish a reliable, visible quadrotor and display-scaled scene."""

    def __init__(self) -> None:
        super().__init__("cq01_presentation")

        self.declare_parameter("display_z_scale", 2.5)
        self.declare_parameter("maximum_actual_path_points", 3000)
        self.declare_parameter("actual_path_min_spacing_m", 0.025)

        self.display_z_scale = max(
            1.0,
            float(self.get_parameter("display_z_scale").value),
        )
        self.maximum_actual_path_points = max(
            100,
            int(self.get_parameter("maximum_actual_path_points").value),
        )
        self.actual_path_min_spacing = max(
            0.005,
            float(
                self.get_parameter(
                    "actual_path_min_spacing_m"
                ).value
            ),
        )

        share = Path(get_package_share_directory("drone_chongqing_cq01"))
        data_path = share / "runtime_data" / "cq01_eling_liziba_data.npz"
        data = np.load(data_path, allow_pickle=False)

        self.terrain_points = data["terrain_surface_points"].astype(
            np.float32
        )
        self.building_points = data["building_points"].astype(
            np.float32
        )
        self.inflated_points = data["inflated_points"].astype(
            np.float32
        )
        self.start_goal = data["start_goal"].astype(np.float32)
        self.final_goal = data["final_goal"].astype(np.float32)
        self.reference_path = data["offline_path_points"].astype(
            np.float32
        )
        self.direct_line = data["direct_line_points"].astype(
            np.float32
        )
        self.corridor_obstacles = data[
            "corridor_obstacle_centres"
        ].astype(np.float32)

        self.display_terrain_points = self.scale_points(
            self.terrain_points
        )
        self.display_building_points = self.scale_points(
            self.building_points
        )
        self.display_inflated_points = self.scale_points(
            self.inflated_points
        )

        self.display_start = self.scale_point(self.start_goal)
        self.display_goal = self.scale_point(self.final_goal)
        self.display_reference_path = self.scale_points(
            self.reference_path
        )
        self.display_direct_line = self.scale_points(
            self.direct_line
        )
        self.display_corridor_obstacles = self.scale_points(
            self.corridor_obstacles
        )

        self.height_column_points = self.make_height_column()
        self.height_datum_path = self.make_static_path(
            [
                self.display_start,
                (
                    float(self.display_goal[0]),
                    float(self.display_goal[1]),
                    float(self.display_start[2]),
                ),
            ]
        )
        self.reference_path_message = self.make_static_path(
            self.display_reference_path
        )
        self.direct_path_message = self.make_static_path(
            self.display_direct_line
        )

        static_qos = transient_qos()
        dynamic_qos = volatile_qos(20)

        self.terrain_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/terrain",
            static_qos,
        )
        self.building_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/buildings",
            static_qos,
        )
        self.safety_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/safety",
            static_qos,
        )
        self.start_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/start",
            static_qos,
        )
        self.goal_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/goal",
            static_qos,
        )
        self.height_column_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/height_column",
            static_qos,
        )
        self.datum_path_publisher = self.create_publisher(
            NavPath,
            "/chongqing/cq01/presentation/height_datum",
            static_qos,
        )
        self.reference_path_publisher = self.create_publisher(
            NavPath,
            "/chongqing/cq01/presentation/reference_path",
            static_qos,
        )
        self.direct_path_publisher = self.create_publisher(
            NavPath,
            "/chongqing/cq01/presentation/direct_path",
            static_qos,
        )
        self.corridor_obstacle_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/corridor_obstacles",
            static_qos,
        )
        self.planned_path_publisher = self.create_publisher(
            NavPath,
            "/chongqing/cq01/presentation/planned_path",
            static_qos,
        )
        self.actual_path_publisher = self.create_publisher(
            NavPath,
            "/chongqing/cq01/presentation/actual_path",
            dynamic_qos,
        )
        self.drone_body_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/drone_body",
            dynamic_qos,
        )
        self.drone_rotor_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/drone_rotors",
            dynamic_qos,
        )
        self.drone_nose_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/drone_nose",
            dynamic_qos,
        )
        self.drone_altitude_publisher = self.create_publisher(
            PointCloud2,
            "/chongqing/cq01/presentation/drone_altitude",
            dynamic_qos,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            dynamic_qos,
        )
        self.create_subscription(
            NavPath,
            "/drone/planned_path_3d",
            self.planned_path_callback,
            static_qos,
        )

        self.terrain_cloud = self.make_cloud(
            self.display_terrain_points
        )
        self.building_cloud = self.make_cloud(
            self.display_building_points
        )
        self.safety_cloud = self.make_cloud(
            self.display_inflated_points
        )
        self.start_cloud = self.make_cloud(
            np.asarray([self.display_start], dtype=np.float32)
        )
        self.goal_cloud = self.make_cloud(
            np.asarray([self.display_goal], dtype=np.float32)
        )
        self.height_column_cloud = self.make_cloud(
            self.height_column_points
        )
        self.corridor_obstacle_cloud = self.make_cloud(
            self.display_corridor_obstacles
        )

        self.actual_path = NavPath()
        self.actual_path.header.frame_id = "map"
        self.latest_position: Optional[np.ndarray] = None
        self.latest_ground_z = 0.0

        self.publish_static()
        self.static_timer = self.create_timer(1.0, self.publish_static)

        self.get_logger().info(
            "CQ-01 presentation ready: "
            f"display_z_scale={self.display_z_scale:.2f}, "
            "quadrotor_source=/drone/odom, "
            "renderer=PointCloud2"
        )
        self.get_logger().info(
            "Physical planning/evaluation remains at Z scale 1.0."
        )

    def scale_point(
        self,
        point_value: Sequence[float],
    ) -> Tuple[float, float, float]:
        return (
            float(point_value[0]),
            float(point_value[1]),
            float(point_value[2]) * self.display_z_scale,
        )

    def scale_points(self, points: np.ndarray) -> np.ndarray:
        result = np.asarray(points, dtype=np.float32).copy()
        if len(result):
            result[:, 2] *= self.display_z_scale
        return result

    def make_cloud(self, points: np.ndarray) -> PointCloud2:
        message = PointCloud2()
        message.header = Header()
        message.header.frame_id = "map"
        message.height = 1
        message.width = int(len(points))
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
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(
            struct.pack("<fff", float(x), float(y), float(z))
            for x, y, z in points
        )
        return message

    def make_static_path(
        self,
        points: Sequence[Sequence[float]],
    ) -> NavPath:
        message = NavPath()
        message.header.frame_id = "map"
        for x, y, z in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def make_height_column(self) -> np.ndarray:
        start_z = float(self.display_start[2])
        goal_z = float(self.display_goal[2])
        minimum = min(start_z, goal_z)
        maximum = max(start_z, goal_z)
        samples = max(2, int(math.ceil((maximum - minimum) / 0.18)))
        z_values = np.linspace(minimum, maximum, samples)
        return np.asarray(
            [
                (
                    float(self.display_goal[0]),
                    float(self.display_goal[1]),
                    float(z_value),
                )
                for z_value in z_values
            ],
            dtype=np.float32,
        )

    def publish_static(self) -> None:
        stamp = self.get_clock().now().to_msg()

        for cloud in (
            self.terrain_cloud,
            self.building_cloud,
            self.safety_cloud,
            self.start_cloud,
            self.goal_cloud,
            self.height_column_cloud,
            self.corridor_obstacle_cloud,
        ):
            cloud.header.stamp = stamp

        for path in (
            self.height_datum_path,
            self.reference_path_message,
            self.direct_path_message,
        ):
            path.header.stamp = stamp
            for pose in path.poses:
                pose.header.stamp = stamp

        self.terrain_publisher.publish(self.terrain_cloud)
        self.building_publisher.publish(self.building_cloud)
        self.safety_publisher.publish(self.safety_cloud)
        self.start_publisher.publish(self.start_cloud)
        self.goal_publisher.publish(self.goal_cloud)
        self.height_column_publisher.publish(
            self.height_column_cloud
        )
        self.datum_path_publisher.publish(self.height_datum_path)
        self.reference_path_publisher.publish(
            self.reference_path_message
        )
        self.direct_path_publisher.publish(
            self.direct_path_message
        )
        self.corridor_obstacle_publisher.publish(
            self.corridor_obstacle_cloud
        )

    def planned_path_callback(self, message: NavPath) -> None:
        output = NavPath()
        output.header.frame_id = "map"
        output.header.stamp = self.get_clock().now().to_msg()

        for input_pose in message.poses:
            pose = PoseStamped()
            pose.header = output.header
            pose.pose = input_pose.pose
            pose.pose.position.z = (
                float(input_pose.pose.position.z)
                * self.display_z_scale
            )
            output.poses.append(pose)

        self.planned_path_publisher.publish(output)

    @staticmethod
    def quaternion_rotate(
        vector: Sequence[float],
        quaternion: Sequence[float],
    ) -> np.ndarray:
        x, y, z = [float(value) for value in vector]
        qx, qy, qz, qw = [float(value) for value in quaternion]

        # v' = v + 2*qw*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
        q_vec = np.asarray([qx, qy, qz], dtype=float)
        v_vec = np.asarray([x, y, z], dtype=float)
        first_cross = np.cross(q_vec, v_vec)
        return (
            v_vec
            + 2.0 * qw * first_cross
            + 2.0 * np.cross(q_vec, first_cross)
        )

    def terrain_height_at(self, x: float, y: float) -> float:
        if not len(self.display_terrain_points):
            return 0.0
        planar_distance = np.square(
            self.display_terrain_points[:, 0] - x
        ) + np.square(
            self.display_terrain_points[:, 1] - y
        )
        index = int(np.argmin(planar_distance))
        return float(self.display_terrain_points[index, 2])

    def odom_callback(self, message: Odometry) -> None:
        position = np.asarray(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z
                * self.display_z_scale,
            ],
            dtype=float,
        )
        orientation = message.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.publish_drone(position, quaternion)
        self.append_actual_path(position, orientation)

    def append_actual_path(
        self,
        position: np.ndarray,
        orientation,
    ) -> None:
        if self.latest_position is not None:
            spacing = float(
                np.linalg.norm(position - self.latest_position)
            )
            if spacing < self.actual_path_min_spacing:
                return

        self.latest_position = position.copy()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation = orientation
        self.actual_path.poses.append(pose)

        if len(self.actual_path.poses) > self.maximum_actual_path_points:
            self.actual_path.poses = self.actual_path.poses[
                -self.maximum_actual_path_points:
            ]

        self.actual_path.header.stamp = pose.header.stamp
        self.actual_path_publisher.publish(self.actual_path)

    def publish_drone(
        self,
        position: np.ndarray,
        quaternion: Sequence[float],
    ) -> None:
        arm_half = 0.60
        diagonal = arm_half / math.sqrt(2.0)

        body_local: List[Tuple[float, float, float]] = [
            (0.0, 0.0, 0.0),
            (0.10, 0.0, 0.0),
            (-0.10, 0.0, 0.0),
        ]

        # Dense samples along both X-config arms.
        for parameter in np.linspace(-arm_half, arm_half, 9):
            body_local.append(
                (
                    float(parameter / math.sqrt(2.0)),
                    float(parameter / math.sqrt(2.0)),
                    0.0,
                )
            )
            body_local.append(
                (
                    float(parameter / math.sqrt(2.0)),
                    float(-parameter / math.sqrt(2.0)),
                    0.0,
                )
            )

        rotor_local = [
            (+diagonal, +diagonal, 0.04),
            (+diagonal, -diagonal, 0.04),
            (-diagonal, -diagonal, 0.04),
            (-diagonal, +diagonal, 0.04),
        ]
        nose_local = [(0.58, 0.0, 0.08)]

        def transform(
            local_points: Sequence[Sequence[float]],
        ) -> np.ndarray:
            output = []
            for local_point in local_points:
                rotated = self.quaternion_rotate(
                    local_point,
                    quaternion,
                )
                output.append(position + rotated)
            return np.asarray(output, dtype=np.float32)

        body_cloud = self.make_cloud(transform(body_local))
        rotor_cloud = self.make_cloud(transform(rotor_local))
        nose_cloud = self.make_cloud(transform(nose_local))

        ground_z = self.terrain_height_at(
            float(position[0]),
            float(position[1]),
        )
        altitude_values = np.linspace(
            ground_z,
            float(position[2]),
            max(
                2,
                int(
                    math.ceil(
                        max(0.0, float(position[2]) - ground_z)
                        / 0.18
                    )
                ),
            ),
        )
        altitude_points = np.asarray(
            [
                (
                    float(position[0]),
                    float(position[1]),
                    float(z_value),
                )
                for z_value in altitude_values
            ],
            dtype=np.float32,
        )
        altitude_cloud = self.make_cloud(altitude_points)

        stamp = self.get_clock().now().to_msg()
        for cloud in (
            body_cloud,
            rotor_cloud,
            nose_cloud,
            altitude_cloud,
        ):
            cloud.header.stamp = stamp

        self.drone_body_publisher.publish(body_cloud)
        self.drone_rotor_publisher.publish(rotor_cloud)
        self.drone_nose_publisher.publish(nose_cloud)
        self.drone_altitude_publisher.publish(altitude_cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01PresentationNode()
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
