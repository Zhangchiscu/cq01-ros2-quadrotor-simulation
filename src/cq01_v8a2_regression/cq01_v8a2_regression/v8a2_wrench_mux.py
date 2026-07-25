#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class V8A2WrenchMux(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8a2_wrench_mux')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_pub = self.create_publisher(WrenchStamped, '/drone/external_wrench', 20)
        self.active_pub = self.create_publisher(Bool, '/drone/disturbance_active', 10)
        self.status_pub = self.create_publisher(String, '/cq01_v8a2/wrench_mux_status', transient)

        self.commands: Dict[str, Optional[WrenchStamped]] = {'low': None, 'climb': None}
        self.command_time: Dict[str, float] = {'low': -1e9, 'climb': -1e9}
        self.active: Dict[str, bool] = {'low': False, 'climb': False}
        self.active_time: Dict[str, float] = {'low': -1e9, 'climb': -1e9}
        self.collision_ever = False
        self.low_selected_ever = False
        self.climb_selected_ever = False
        self.current_source = 'none'
        self.previous_source = None
        self.last_status_s = -1e9

        self.create_subscription(WrenchStamped, '/cq01_v8a2/low/wrench_cmd', lambda m: self.command_cb('low', m), 20)
        self.create_subscription(WrenchStamped, '/cq01_v8a2/climb/wrench_cmd', lambda m: self.command_cb('climb', m), 20)
        self.create_subscription(Bool, '/cq01_v8a2/low/active', lambda m: self.active_cb('low', m), 10)
        self.create_subscription(Bool, '/cq01_v8a2/climb/active', lambda m: self.active_cb('climb', m), 10)
        self.timer = self.create_timer(0.005, self.timer_cb)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def command_cb(self, source: str, message: WrenchStamped) -> None:
        self.commands[source] = message
        self.command_time[source] = self.now_s()

    def active_cb(self, source: str, message: Bool) -> None:
        self.active[source] = bool(message.data)
        self.active_time[source] = self.now_s()

    @staticmethod
    def norm(message: Optional[WrenchStamped]) -> float:
        if message is None:
            return 0.0
        w = message.wrench
        return math.sqrt(
            w.force.x ** 2 + w.force.y ** 2 + w.force.z ** 2
            + w.torque.x ** 2 + w.torque.y ** 2 + w.torque.z ** 2
        )

    def effective_active(self, source: str, now: float) -> bool:
        command_fresh = now - self.command_time[source] <= 0.20
        active_fresh = now - self.active_time[source] <= 0.20
        nonzero = command_fresh and self.norm(self.commands[source]) > 1e-6
        return nonzero or (active_fresh and self.active[source])

    def zero_message(self) -> WrenchStamped:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        return message

    def timer_cb(self) -> None:
        now = self.now_s()
        low_active = self.effective_active('low', now)
        climb_active = self.effective_active('climb', now)

        if low_active and climb_active:
            self.collision_ever = True
            selected = 'collision'
            output = self.zero_message()
            output_active = False
        elif low_active:
            selected = 'low'
            self.low_selected_ever = True
            output = self.commands['low'] or self.zero_message()
            output_active = True
        elif climb_active:
            selected = 'climb'
            self.climb_selected_ever = True
            output = self.commands['climb'] or self.zero_message()
            output_active = True
        else:
            selected = 'none'
            output = self.zero_message()
            output_active = False

        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'map'
        self.command_pub.publish(output)
        active_message = Bool()
        active_message.data = output_active
        self.active_pub.publish(active_message)
        self.current_source = selected

        if selected != self.previous_source or now - self.last_status_s >= 0.5:
            self.publish_status(now, low_active, climb_active)
            self.previous_source = selected

    def publish_status(self, now: float, low_active: bool, climb_active: bool) -> None:
        payload = {
            'state': 'collision' if self.collision_ever else 'running',
            'selected_source': self.current_source,
            'low_effective_active': low_active,
            'climb_effective_active': climb_active,
            'collision_ever': self.collision_ever,
            'low_selected_ever': self.low_selected_ever,
            'climb_selected_ever': self.climb_selected_ever,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(message)
        self.last_status_s = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A2WrenchMux()
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
