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


class V8A3CameraFocus(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8a3_camera_focus')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf = TransformBroadcaster(self)
        self.position = (0.0, 0.0, 8.0)
        self.drone: Optional[Tuple[float, float, float]] = None
        self.mission: Dict = {}
        self.low: Dict = {}
        self.climb: Dict = {}
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
            '/cq01_v8a3/mission_status',
            lambda m: self.json_callback('mission', m),
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8a3/low/status',
            lambda m: self.json_callback('low', m),
            transient,
        )
        self.create_subscription(
            String,
            '/cq01_v8a3/climb/status',
            lambda m: self.json_callback('climb', m),
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
        if target == 'mission' and 'reached_index' in payload:
            try:
                self.maximum_reached_index = max(
                    self.maximum_reached_index,
                    int(payload.get('reached_index', -1)),
                )
            except (TypeError, ValueError):
                pass

    def desired_target(
        self,
    ) -> Tuple[Tuple[float, float, float], float]:
        try:
            current_index = int(
                self.mission.get('current_index', -1)
            )
        except (TypeError, ValueError):
            current_index = -1

        climb_started = bool(
            self.climb.get('triggered_once', False)
            or self.climb.get('scheduled', False)
            or self.maximum_reached_index >= 3
        )
        if climb_started:
            return (105.0, -30.0, 48.0), 0.45

        low_state = str(self.low.get('state', ''))
        low_scene_active = (
            current_index == 2
            and self.maximum_reached_index >= 1
            and self.maximum_reached_index < 2
        ) or low_state in (
            'active',
            'recovery',
            'recovered_waiting_for_P4',
        )

        if low_scene_active:
            # Slightly behind the trigger point in -Y so the 8-10 m
            # downwind displacement stays centered in the RViz view.
            return (50.0, -99.0, 24.0), 0.35

        if self.drone is not None:
            return self.drone, 0.75
        return self.position, 1.0

    def timer_callback(self) -> None:
        now = self.now_s()
        dt = max(0.0, min(0.2, now - self.last_s))
        self.last_s = now
        target, tau = self.desired_target()
        alpha = 1.0 - math.exp(
            -dt / max(0.05, tau)
        )
        self.position = tuple(
            current + alpha * (desired - current)
            for current, desired in zip(self.position, target)
        )

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'cq01_v8a3_camera_focus'
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A3CameraFocus()
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
