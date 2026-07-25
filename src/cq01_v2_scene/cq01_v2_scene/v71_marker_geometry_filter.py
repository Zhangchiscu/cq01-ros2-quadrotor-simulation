#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class V71MarkerGeometryFilter(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v71_marker_geometry_filter')
        self.pub = self.create_publisher(
            MarkerArray,
            '/cq01_v71/h10_geometry',
            10,
        )
        self.create_subscription(
            MarkerArray,
            '/cq01_v34h/disturbance_markers',
            self.callback,
            10,
        )

    def callback(self, message: MarkerArray) -> None:
        output = MarkerArray()
        output.markers = [
            marker
            for marker in message.markers
            if marker.type != Marker.TEXT_VIEW_FACING
        ]
        self.pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V71MarkerGeometryFilter()
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
