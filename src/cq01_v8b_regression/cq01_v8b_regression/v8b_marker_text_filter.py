#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class V8BMarkerTextFilter(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8b_marker_text_filter')
        self.declare_parameter('input_topic', '/markers_in')
        self.declare_parameter('output_topic', '/markers_out')
        self.input_topic = str(
            self.get_parameter('input_topic').value
        )
        self.output_topic = str(
            self.get_parameter('output_topic').value
        )
        self.publisher = self.create_publisher(
            MarkerArray,
            self.output_topic,
            10,
        )
        self.create_subscription(
            MarkerArray,
            self.input_topic,
            self.callback,
            10,
        )

    def callback(self, message: MarkerArray) -> None:
        output = MarkerArray()
        output.markers = [
            marker
            for marker in message.markers
            if (
                marker.action == Marker.DELETEALL
                or marker.type != Marker.TEXT_VIEW_FACING
            )
        ]
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8BMarkerTextFilter()
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
