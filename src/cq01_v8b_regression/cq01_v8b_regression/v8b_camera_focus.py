#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

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


class V8BCameraFocus(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8b_camera_focus')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf = TransformBroadcaster(self)
        self.drone: Optional[Tuple[float, float, float]] = None
        self.position = (315.0, -45.0, 130.0)
        self.route: Dict = {}
        self.h14: Dict = {}
        self.integrated: Dict = {}
        self.pattern: Dict = {}
        self.maximum_reached_index = -1
        self.last_s = self.now_s()

        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            String,
            '/cq01_v8b/route_status',
            lambda message: self.json_callback('route', message),
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8b/h14/status',
            lambda message: self.json_callback('h14', message),
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8b/integrated/status',
            lambda message: self.json_callback('integrated', message),
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8b/pattern/status',
            lambda message: self.json_callback('pattern', message),
            transient,
        )
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback,
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.drone = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

    def json_callback(self, target: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        setattr(self, target, payload)
        if target == 'route' and 'reached_index' in payload:
            try:
                self.maximum_reached_index = max(
                    self.maximum_reached_index,
                    int(payload.get('reached_index', -1)),
                )
            except (TypeError, ValueError):
                pass

    def desired(self) -> Tuple[Tuple[float, float, float], float]:
        integrated_state = str(self.integrated.get('state', ''))
        pattern_state = str(self.pattern.get('state', ''))
        if (
            integrated_state in (
                'summit_pattern_running',
                'summit_pattern_completed',
                'integrated_completed',
            )
            or pattern_state not in ('', 'idle', 'waiting_for_start')
        ):
            return (400.0, 45.0, 248.0), 0.45

        h14_state = str(self.h14.get('state', ''))
        h14_scene = (
            self.maximum_reached_index >= 12
            and self.maximum_reached_index < 14
        ) or h14_state.startswith('high_altitude_')
        if h14_scene:
            return (342.0, -27.0, 174.0), 0.40

        if self.drone is not None:
            return self.drone, 0.70
        return self.position, 1.0

    def timer_callback(self) -> None:
        now = self.now_s()
        dt = max(0.0, min(0.2, now - self.last_s))
        self.last_s = now
        target, tau = self.desired()
        alpha = 1.0 - math.exp(
            -dt / max(0.05, tau)
        )
        self.position = tuple(
            current + alpha * (desired - current)
            for current, desired in zip(
                self.position,
                target,
            )
        )

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'cq01_v8b_camera_focus'
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8BCameraFocus()
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
