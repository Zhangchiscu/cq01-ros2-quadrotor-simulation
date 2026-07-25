#!/usr/bin/env python3
"""Inflate a raw 2D occupancy grid for safe quadrotor planning."""

from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class OccupancyInflator(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_inflator")

        self.declare_parameter(
            "input_topic",
            "/drone/obstacle_grid_raw",
        )
        self.declare_parameter(
            "output_topic",
            "/drone/obstacle_grid",
        )
        self.declare_parameter("inflation_radius_m", 0.50)
        self.declare_parameter("occupancy_threshold", 50)

        input_topic = str(
            self.get_parameter("input_topic").value
        )
        output_topic = str(
            self.get_parameter("output_topic").value
        )
        self.inflation_radius = float(
            self.get_parameter("inflation_radius_m").value
        )
        self.occupancy_threshold = int(
            self.get_parameter("occupancy_threshold").value
        )

        if self.inflation_radius < 0.0:
            raise ValueError(
                "inflation_radius_m must be non-negative"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            output_topic,
            qos,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid,
            input_topic,
            self.grid_callback,
            qos,
        )

        self.last_signature = None
        self.offset_cache: List[Tuple[int, int]] = []

        self.get_logger().info(
            "Occupancy inflator started: "
            f"{input_topic} -> {output_topic}, "
            f"radius={self.inflation_radius:.2f} m"
        )

    def build_offsets(
        self,
        resolution: float,
    ) -> List[Tuple[int, int]]:
        if self.inflation_radius <= 0.0:
            return [(0, 0)]

        cell_radius = int(
            math.ceil(self.inflation_radius / resolution)
        )
        offsets: List[Tuple[int, int]] = []

        for delta_x in range(
            -cell_radius,
            cell_radius + 1,
        ):
            for delta_y in range(
                -cell_radius,
                cell_radius + 1,
            ):
                distance = resolution * math.hypot(
                    delta_x,
                    delta_y,
                )
                if distance <= self.inflation_radius + 1.0e-9:
                    offsets.append((delta_x, delta_y))

        return offsets

    def grid_callback(
        self,
        message: OccupancyGrid,
    ) -> None:
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)

        signature = (
            width,
            height,
            resolution,
            self.inflation_radius,
        )
        if signature != self.last_signature:
            self.offset_cache = self.build_offsets(resolution)
            self.last_signature = signature

        output = OccupancyGrid()
        output.header = message.header
        output.info = message.info
        output.data = list(message.data)

        occupied_cells: List[Tuple[int, int]] = []

        for row in range(height):
            base_index = row * width
            for column in range(width):
                value = int(message.data[base_index + column])
                if value >= self.occupancy_threshold:
                    occupied_cells.append((column, row))

        inflated = list(output.data)

        for column, row in occupied_cells:
            for delta_x, delta_y in self.offset_cache:
                target_column = column + delta_x
                target_row = row + delta_y

                if (
                    target_column < 0
                    or target_column >= width
                    or target_row < 0
                    or target_row >= height
                ):
                    continue

                index = target_row * width + target_column
                inflated[index] = 100

        output.data = inflated
        output.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyInflator()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
