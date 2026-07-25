#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

Point3 = Tuple[float, float, float]


class V71UnifiedCameraFocusManager(Node):
    """One TF broadcaster for the complete short regression."""

    def __init__(self) -> None:
        super().__init__('cq01_v71_unified_camera_focus_manager')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf = TransformBroadcaster(self)
        self.position: Point3 = (0.0, 0.0, 12.0)
        self.drone: Optional[Point3] = None
        self.route_started = False

        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            String,
            '/cq01_v34h/mission_status',
            self.route_callback,
            transient,
        )

        self.last_s = self.now_s()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.drone = (float(p.x), float(p.y), float(p.z))

    def route_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        # The dispatcher is alive during the prelude but auto_start=false.
        # Only an ACTIVE dispatcher means the original route has started.
        self.route_started = bool(payload.get('active', False))

    def timer_callback(self) -> None:
        now = self.now_s()
        dt = max(0.0, min(0.2, now - self.last_s))
        self.last_s = now

        if self.route_started:
            target = (60.70, -85.88, 24.0)
            tau = 0.9
        elif self.drone is not None:
            target = self.drone
            tau = 0.7
        else:
            target = self.position
            tau = 1.0

        alpha = 1.0 - math.exp(-dt / max(0.05, tau))
        self.position = tuple(
            current + alpha * (desired - current)
            for current, desired in zip(self.position, target)
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'cq01_camera_focus'
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V71UnifiedCameraFocusManager()
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
