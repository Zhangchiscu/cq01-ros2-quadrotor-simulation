#!/usr/bin/env python3

from math import ceil, floor
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

from drone_map.geometry import first_segment_collision


class StaticObstacleMap(Node):
    def __init__(self) -> None:
        super().__init__("static_obstacle_map")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("safety_margin", 0.25)
        self.declare_parameter("grid_resolution", 0.10)
        self.declare_parameter("grid_origin_x", -1.0)
        self.declare_parameter("grid_origin_y", -4.0)
        self.declare_parameter("grid_width_m", 8.0)
        self.declare_parameter("grid_height_m", 8.0)

        self.declare_parameter(
            "obstacle_names",
            [
                "center_block",
                "upper_pillar",
                "lower_pillar",
                "corridor_upper_wall",
                "corridor_lower_wall",
                "goal_block",
            ],
        )
        self.declare_parameter(
            "center_x",
            [1.0, 2.4, 2.4, 4.0, 4.0, 5.5],
        )
        self.declare_parameter(
            "center_y",
            [0.0, 1.4, -1.4, 1.45, -1.45, 0.0],
        )
        self.declare_parameter(
            "center_z",
            [1.1, 1.2, 1.2, 1.25, 1.25, 1.1],
        )
        self.declare_parameter(
            "size_x",
            [0.65, 0.70, 0.70, 0.40, 0.40, 0.75],
        )
        self.declare_parameter(
            "size_y",
            [1.70, 0.70, 0.70, 2.00, 2.00, 0.75],
        )
        self.declare_parameter(
            "size_z",
            [2.20, 2.40, 2.40, 2.50, 2.50, 2.20],
        )

        self.frame_id = str(
            self.get_parameter("frame_id").value
        )
        self.safety_margin = float(
            self.get_parameter("safety_margin").value
        )
        self.grid_resolution = float(
            self.get_parameter("grid_resolution").value
        )
        self.grid_origin_x = float(
            self.get_parameter("grid_origin_x").value
        )
        self.grid_origin_y = float(
            self.get_parameter("grid_origin_y").value
        )
        self.grid_width_m = float(
            self.get_parameter("grid_width_m").value
        )
        self.grid_height_m = float(
            self.get_parameter("grid_height_m").value
        )

        self.obstacles = self.load_obstacles()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/drone/obstacle_markers",
            qos,
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            "/drone/obstacle_grid",
            qos,
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_map,
        )

        self.publish_map()
        self.log_summary()

    def load_obstacles(self) -> List[Dict[str, float]]:
        names = list(
            self.get_parameter("obstacle_names").value
        )
        arrays = {
            "center_x": list(
                self.get_parameter("center_x").value
            ),
            "center_y": list(
                self.get_parameter("center_y").value
            ),
            "center_z": list(
                self.get_parameter("center_z").value
            ),
            "size_x": list(
                self.get_parameter("size_x").value
            ),
            "size_y": list(
                self.get_parameter("size_y").value
            ),
            "size_z": list(
                self.get_parameter("size_z").value
            ),
        }

        lengths = {len(names)}
        lengths.update(len(values) for values in arrays.values())

        if len(lengths) != 1:
            raise RuntimeError(
                "All obstacle arrays must have equal length"
            )

        obstacles: List[Dict[str, float]] = []

        for index, name in enumerate(names):
            obstacle = {"name": str(name)}
            for key, values in arrays.items():
                obstacle[key] = float(values[index])

            if (
                obstacle["size_x"] <= 0.0
                or obstacle["size_y"] <= 0.0
                or obstacle["size_z"] <= 0.0
            ):
                raise RuntimeError(
                    f"Obstacle {name} has invalid size"
                )

            obstacles.append(obstacle)

        return obstacles

    @staticmethod
    def set_color(
        marker: Marker,
        red: float,
        green: float,
        blue: float,
        alpha: float,
    ) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def cube_marker(
        self,
        marker_id: int,
        obstacle: Dict[str, float],
        safety_zone: bool,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = (
            "safety_zones"
            if safety_zone
            else "obstacles"
        )
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obstacle["center_x"]
        marker.pose.position.y = obstacle["center_y"]
        marker.pose.position.z = obstacle["center_z"]
        marker.pose.orientation.w = 1.0

        if safety_zone:
            marker.scale.x = (
                obstacle["size_x"]
                + 2.0 * self.safety_margin
            )
            marker.scale.y = (
                obstacle["size_y"]
                + 2.0 * self.safety_margin
            )
            marker.scale.z = obstacle["size_z"] + 0.10
            self.set_color(
                marker,
                1.0,
                0.65,
                0.05,
                0.16,
            )
        else:
            marker.scale.x = obstacle["size_x"]
            marker.scale.y = obstacle["size_y"]
            marker.scale.z = obstacle["size_z"]
            palette = (
                (0.80, 0.20, 0.18),
                (0.20, 0.50, 0.88),
                (0.25, 0.72, 0.35),
                (0.72, 0.30, 0.78),
                (0.90, 0.55, 0.15),
                (0.25, 0.72, 0.72),
            )
            red, green, blue = (
                palette[marker_id % len(palette)]
            )
            self.set_color(
                marker,
                red,
                green,
                blue,
                0.90,
            )

        return marker

    def label_marker(
        self,
        marker_id: int,
        obstacle: Dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacle_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = obstacle["center_x"]
        marker.pose.position.y = obstacle["center_y"]
        marker.pose.position.z = (
            obstacle["center_z"]
            + 0.5 * obstacle["size_z"]
            + 0.25
        )
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.text = obstacle["name"]
        self.set_color(
            marker,
            1.0,
            1.0,
            1.0,
            0.95,
        )
        return marker

    def boundary_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "map_boundary"
        marker.id = 500
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        self.set_color(
            marker,
            0.95,
            0.95,
            0.95,
            0.75,
        )

        x0 = self.grid_origin_x
        y0 = self.grid_origin_y
        x1 = x0 + self.grid_width_m
        y1 = y0 + self.grid_height_m

        for x_value, y_value in (
            (x0, y0),
            (x1, y0),
            (x1, y1),
            (x0, y1),
            (x0, y0),
        ):
            point = Point()
            point.x = x_value
            point.y = y_value
            point.z = 0.02
            marker.points.append(point)

        return marker

    def build_markers(self) -> MarkerArray:
        output = MarkerArray()

        for index, obstacle in enumerate(
            self.obstacles
        ):
            output.markers.append(
                self.cube_marker(
                    index,
                    obstacle,
                    False,
                )
            )
            output.markers.append(
                self.cube_marker(
                    100 + index,
                    obstacle,
                    True,
                )
            )

        output.markers.append(
            self.boundary_marker()
        )
        return output

    def build_grid(self) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()

        width = int(
            round(
                self.grid_width_m
                / self.grid_resolution
            )
        )
        height = int(
            round(
                self.grid_height_m
                / self.grid_resolution
            )
        )

        grid.info.resolution = self.grid_resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = self.grid_origin_x
        grid.info.origin.position.y = self.grid_origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (width * height)

        for obstacle in self.obstacles:
            half_x = (
                0.5 * obstacle["size_x"]
                + self.safety_margin
            )
            half_y = (
                0.5 * obstacle["size_y"]
                + self.safety_margin
            )

            min_col = max(
                0,
                floor(
                    (
                        obstacle["center_x"]
                        - half_x
                        - self.grid_origin_x
                    )
                    / self.grid_resolution
                ),
            )
            max_col = min(
                width - 1,
                ceil(
                    (
                        obstacle["center_x"]
                        + half_x
                        - self.grid_origin_x
                    )
                    / self.grid_resolution
                ),
            )
            min_row = max(
                0,
                floor(
                    (
                        obstacle["center_y"]
                        - half_y
                        - self.grid_origin_y
                    )
                    / self.grid_resolution
                ),
            )
            max_row = min(
                height - 1,
                ceil(
                    (
                        obstacle["center_y"]
                        + half_y
                        - self.grid_origin_y
                    )
                    / self.grid_resolution
                ),
            )

            for row in range(
                min_row,
                max_row + 1,
            ):
                for column in range(
                    min_col,
                    max_col + 1,
                ):
                    grid.data[
                        row * width + column
                    ] = 100

        return grid

    def publish_map(self) -> None:
        self.marker_pub.publish(
            self.build_markers()
        )
        self.grid_pub.publish(
            self.build_grid()
        )

    def log_summary(self) -> None:
        self.get_logger().info(
            "Static obstacle map started"
        )
        self.get_logger().info(
            f"Obstacles: {len(self.obstacles)}, "
            f"safety margin: {self.safety_margin:.2f} m"
        )
        self.get_logger().info(
            "Topics: /drone/obstacle_markers, "
            "/drone/obstacle_grid"
        )

        collision = first_segment_collision(
            0.0,
            0.0,
            6.0,
            0.0,
            self.obstacles,
            self.safety_margin,
        )

        if collision is None:
            self.get_logger().warn(
                "Self-test failed: direct path is clear"
            )
        else:
            self.get_logger().info(
                "Self-test: direct path "
                "(0,0)->(6,0) is blocked by "
                f"{collision['name']}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticObstacleMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
