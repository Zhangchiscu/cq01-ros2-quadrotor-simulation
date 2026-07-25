#!/usr/bin/env python3
"""Print the latest voxel metadata and terminate."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class MetadataInspector(Node):
    def __init__(self) -> None:
        super().__init__("voxel_metadata_inspector")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/drone/voxel_metadata",
            self.callback,
            qos,
        )

    def callback(self, message: String) -> None:
        data = json.loads(message.data)

        print("===== 3D Voxel Map Metadata =====")
        print(f"Scenario: {data['scenario']}")
        print(
            "Resolution: "
            f"{data['resolution_m']:.3f} m"
        )
        print(
            "Inflation radius: "
            f"{data['inflation_radius_m']:.3f} m"
        )
        dimensions = data["dimensions"]
        print(
            "Dimensions: "
            f"{dimensions['x']} x "
            f"{dimensions['y']} x "
            f"{dimensions['z']}"
        )
        print(
            "Base occupied voxels: "
            f"{data['base_occupied_voxels']}"
        )
        print(
            "Inflated occupied voxels: "
            f"{data['inflated_occupied_voxels']}"
        )
        print(
            "Free voxels: "
            f"{data['free_voxels']}"
        )
        print(
            "Slice height: "
            f"{data['slice_z_m']:.3f} m"
        )

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetadataInspector()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
