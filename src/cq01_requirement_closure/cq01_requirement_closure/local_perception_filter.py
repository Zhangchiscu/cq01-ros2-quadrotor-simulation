#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import struct
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker


POINTFIELD_FORMAT: Dict[int, Tuple[str, int]] = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


def static_map_qos() -> QoSProfile:
    """QoS compatible with the CQ01 teacher-interface map publisher."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class LocalPerceptionFilter(Node):
    """12 m local PointCloud2 perception simulation.

    Global static obstacle cloud:
        /map/obstacles
        RELIABLE + TRANSIENT_LOCAL

    Local output:
        /map/local_obstacles_cloud

    The full static cloud is cached once (or whenever republished). It is
    cropped around the latest vehicle position at 5 Hz instead of on every
    high-rate odometry callback.
    """

    def __init__(self) -> None:
        super().__init__("local_perception_filter")

        defaults = {
            "odom_topic": "/drone/odom",
            "obstacle_cloud_topic": "/map/obstacles",
            "local_cloud_topic": "/map/local_obstacles_cloud",
            "perception_marker_topic": "/cq01_bonus/perception_range",
            "sensing_radius_m": 12.0,
            "filter_rate_hz": 5.0,
            "status_publish_period_s": 0.5,
        }

        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.radius = float(
            self.get_parameter("sensing_radius_m").value
        )
        self.filter_rate_hz = max(
            1.0,
            float(self.get_parameter("filter_rate_hz").value),
        )

        odom_topic = str(
            self.get_parameter("odom_topic").value
        )
        cloud_topic = str(
            self.get_parameter("obstacle_cloud_topic").value
        )
        local_cloud_topic = str(
            self.get_parameter("local_cloud_topic").value
        )
        perception_topic = str(
            self.get_parameter("perception_marker_topic").value
        )

        self.position: Optional[Tuple[float, float, float]] = None
        self.latest_cloud: Optional[PointCloud2] = None

        self.input_point_count = 0
        self.local_point_count = 0
        self.minimum_point_distance = float("nan")
        self.last_frame = "map"
        self.cloud_messages = 0
        self.odom_messages = 0
        self.filter_cycles = 0

        map_qos = static_map_qos()

        # Latched local point cloud: RViz can start later and still receive
        # the most recently cropped cloud.
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            local_cloud_topic,
            map_qos,
        )
        self.range_pub = self.create_publisher(
            Marker,
            perception_topic,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/cq01_bonus/local_perception/status",
            10,
        )

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        # Critical V1.4 fix: exact static-map-compatible QoS.
        self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            map_qos,
        )

        self.create_timer(
            1.0 / self.filter_rate_hz,
            self.filter_cloud,
        )

        status_period = max(
            0.1,
            float(
                self.get_parameter(
                    "status_publish_period_s"
                ).value
            ),
        )
        self.create_timer(
            status_period,
            self.publish_status_and_range,
        )

        self.get_logger().warning(
            "Local-perception V1.4 active: "
            f"input={cloud_topic}, radius={self.radius:.2f} m, "
            "QoS=RELIABLE+TRANSIENT_LOCAL, "
            f"filter_rate={self.filter_rate_hz:.1f} Hz, "
            f"output={local_cloud_topic}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        self.odom_messages += 1

        if msg.header.frame_id:
            self.last_frame = msg.header.frame_id

    def cloud_callback(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.cloud_messages += 1

        if msg.header.frame_id:
            self.last_frame = msg.header.frame_id

        self.get_logger().warning(
            "Global obstacle PointCloud2 received: "
            f"message={self.cloud_messages}, "
            f"width={msg.width}, height={msg.height}, "
            f"point_step={msg.point_step}, frame={msg.header.frame_id}"
        )

        # Immediately produce the first local cloud after receiving the
        # transient-local global map.
        self.filter_cloud()

    @staticmethod
    def xyz_field_info(
        cloud: PointCloud2,
    ) -> Optional[Dict[str, Tuple[int, str]]]:
        fields = {
            field.name: field
            for field in cloud.fields
        }

        endian = ">" if cloud.is_bigendian else "<"
        result: Dict[str, Tuple[int, str]] = {}

        for name in ("x", "y", "z"):
            field = fields.get(name)

            if field is None:
                return None

            info = POINTFIELD_FORMAT.get(
                field.datatype
            )
            if info is None:
                return None

            fmt_char, _size = info
            result[name] = (
                int(field.offset),
                endian + fmt_char,
            )

        return result

    def filter_cloud(self) -> None:
        if (
            self.position is None
            or self.latest_cloud is None
        ):
            return

        cloud = self.latest_cloud
        field_info = self.xyz_field_info(cloud)

        if field_info is None:
            self.get_logger().error(
                "PointCloud2 x/y/z fields are absent "
                "or use unsupported datatypes."
            )
            return

        point_step = int(cloud.point_step)
        if point_step <= 0:
            return

        px, py, pz = self.position
        radius_sq = self.radius * self.radius

        data = cloud.data
        row_step = int(cloud.row_step)
        width = int(cloud.width)
        height = max(
            1,
            int(cloud.height),
        )

        x_offset, x_fmt = field_info["x"]
        y_offset, y_fmt = field_info["y"]
        z_offset, z_fmt = field_info["z"]

        selected = bytearray()
        valid_input = 0
        local_count = 0
        minimum_sq = float("inf")

        for row in range(height):
            row_base = row * row_step

            for col in range(width):
                offset = (
                    row_base
                    + col * point_step
                )
                end = offset + point_step

                if end > len(data):
                    continue

                try:
                    x = float(
                        struct.unpack_from(
                            x_fmt,
                            data,
                            offset + x_offset,
                        )[0]
                    )
                    y = float(
                        struct.unpack_from(
                            y_fmt,
                            data,
                            offset + y_offset,
                        )[0]
                    )
                    z = float(
                        struct.unpack_from(
                            z_fmt,
                            data,
                            offset + z_offset,
                        )[0]
                    )
                except (
                    struct.error,
                    ValueError,
                    TypeError,
                ):
                    continue

                if not (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                ):
                    continue

                valid_input += 1

                d2 = (
                    (x - px) ** 2
                    + (y - py) ** 2
                    + (z - pz) ** 2
                )

                minimum_sq = min(
                    minimum_sq,
                    d2,
                )

                if d2 <= radius_sq:
                    selected.extend(
                        data[offset:end]
                    )
                    local_count += 1

        output = PointCloud2()
        output.header = cloud.header
        output.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        output.height = 1
        output.width = local_count
        output.fields = cloud.fields
        output.is_bigendian = (
            cloud.is_bigendian
        )
        output.point_step = point_step
        output.row_step = (
            local_count
            * point_step
        )
        output.data = bytes(selected)
        output.is_dense = cloud.is_dense

        self.input_point_count = valid_input
        self.local_point_count = local_count
        self.minimum_point_distance = (
            math.sqrt(minimum_sq)
            if math.isfinite(
                minimum_sq
            )
            else float("nan")
        )
        self.filter_cycles += 1

        self.cloud_pub.publish(output)

    def publish_status_and_range(self) -> None:
        status = {
            "mode":
                "LOCAL_POINTCLOUD_RANGE_SIMULATION_V14",
            "sensing_radius_m":
                self.radius,
            "filter_rate_hz":
                self.filter_rate_hz,
            "odom_messages":
                self.odom_messages,
            "pointcloud_messages":
                self.cloud_messages,
            "filter_cycles":
                self.filter_cycles,
            "pointcloud_input_count":
                self.input_point_count,
            "pointcloud_local_count":
                self.local_point_count,
            "nearest_obstacle_m": (
                round(
                    self.minimum_point_distance,
                    3,
                )
                if math.isfinite(
                    self.minimum_point_distance
                )
                else None
            ),
            "input_qos": {
                "reliability":
                    "RELIABLE",
                "durability":
                    "TRANSIENT_LOCAL",
                "history":
                    "KEEP_LAST",
                "depth":
                    1,
            },
            "input_topic":
                "/map/obstacles",
            "output_topic":
                "/map/local_obstacles_cloud",
        }

        if self.position is not None:
            px, py, pz = self.position

            status["drone_position_m"] = [
                round(px, 3),
                round(py, 3),
                round(pz, 3),
            ]

            marker = Marker()
            marker.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )
            marker.header.frame_id = (
                self.last_frame
                or "map"
            )
            marker.ns = (
                "local_perception_range"
            )
            marker.id = 0
            marker.type = (
                Marker.LINE_LIST
            )
            marker.action = (
                Marker.ADD
            )
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.06
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.65

            segments = 72

            for plane in range(3):
                for i in range(
                    segments
                ):
                    a0 = (
                        2.0
                        * math.pi
                        * i
                        / segments
                    )
                    a1 = (
                        2.0
                        * math.pi
                        * (i + 1)
                        / segments
                    )

                    p0 = Point()
                    p1 = Point()

                    if plane == 0:
                        p0.x = (
                            px
                            + self.radius
                            * math.cos(a0)
                        )
                        p0.y = (
                            py
                            + self.radius
                            * math.sin(a0)
                        )
                        p0.z = pz

                        p1.x = (
                            px
                            + self.radius
                            * math.cos(a1)
                        )
                        p1.y = (
                            py
                            + self.radius
                            * math.sin(a1)
                        )
                        p1.z = pz

                    elif plane == 1:
                        p0.x = (
                            px
                            + self.radius
                            * math.cos(a0)
                        )
                        p0.y = py
                        p0.z = (
                            pz
                            + self.radius
                            * math.sin(a0)
                        )

                        p1.x = (
                            px
                            + self.radius
                            * math.cos(a1)
                        )
                        p1.y = py
                        p1.z = (
                            pz
                            + self.radius
                            * math.sin(a1)
                        )

                    else:
                        p0.x = px
                        p0.y = (
                            py
                            + self.radius
                            * math.cos(a0)
                        )
                        p0.z = (
                            pz
                            + self.radius
                            * math.sin(a0)
                        )

                        p1.x = px
                        p1.y = (
                            py
                            + self.radius
                            * math.cos(a1)
                        )
                        p1.z = (
                            pz
                            + self.radius
                            * math.sin(a1)
                        )

                    marker.points.append(
                        p0
                    )
                    marker.points.append(
                        p1
                    )

            self.range_pub.publish(
                marker
            )

        msg = String()
        msg.data = json.dumps(
            status,
            ensure_ascii=False,
        )
        self.status_pub.publish(
            msg
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPerceptionFilter()

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
