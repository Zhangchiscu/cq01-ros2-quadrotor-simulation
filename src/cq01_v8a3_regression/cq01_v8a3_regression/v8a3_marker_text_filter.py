#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class V8A3MarkerTextFilter(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8a3_marker_text_filter')
        self.declare_parameter('input_topic', '/markers_in')
        self.declare_parameter('output_topic', '/markers_out')
        self.declare_parameter('drop_text', True)
        self.declare_parameter('drop_marker_ids', [])
        self.declare_parameter('drop_namespaces', [])

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.drop_text = bool(self.get_parameter('drop_text').value)
        self.drop_marker_ids = set(int(x) for x in self.get_parameter('drop_marker_ids').value)
        self.drop_namespaces = set(str(x) for x in self.get_parameter('drop_namespaces').value)

        self.pub = self.create_publisher(MarkerArray, self.output_topic, 10)
        self.sub = self.create_subscription(MarkerArray, self.input_topic, self.callback, 10)
        self.get_logger().info(
            f'V8-A3 marker text filter started: {self.input_topic} -> {self.output_topic}; '
            f'drop_text={self.drop_text}, drop_ids={sorted(self.drop_marker_ids)}, '
            f'drop_namespaces={sorted(self.drop_namespaces)}'
        )

    def keep(self, marker: Marker) -> bool:
        if marker.action == Marker.DELETEALL:
            return True
        if marker.id in self.drop_marker_ids:
            return False
        if marker.ns in self.drop_namespaces:
            return False
        if self.drop_text and marker.type == Marker.TEXT_VIEW_FACING:
            return False
        return True

    def callback(self, msg: MarkerArray) -> None:
        out = MarkerArray()
        out.markers = [m for m in msg.markers if self.keep(m)]
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A3MarkerTextFilter()
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
