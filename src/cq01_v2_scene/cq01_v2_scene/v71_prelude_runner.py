#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

Point3 = Tuple[float, float, float]


def yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class V71PreludeRunner(Node):
    """Runs the prelude, then exits completely before route nodes start."""

    def __init__(self) -> None:
        super().__init__('cq01_v71_prelude_runner')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goals: List[dict] = [
            {
                'name': 'PRE_HOVER',
                'position': (0.0, 0.0, 12.0),
                'hold_s': 4.0,
            },
            {
                'name': 'PRE_SINGLE',
                'position': (10.0, -15.0, 14.0),
                'hold_s': 2.0,
            },
            {
                'name': 'PRE_MULTI_A',
                'position': (15.0, -45.0, 15.0),
                'hold_s': 0.5,
            },
            {
                'name': 'PRE_MULTI_B',
                'position': (18.0, -80.0, 16.0),
                'hold_s': 0.5,
            },
        ]

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/drone/goal_raw',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/cq01_v71/prelude_status',
            transient,
        )

        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            String,
            '/drone/planner_3d/status',
            self.planner_callback,
            transient,
        )

        self.index = 0
        self.have_odom = False
        self.planner_ready = False
        self.goal_sent = False
        self.hold_until: Optional[float] = None
        self.start_s = self.now_s()
        self.send_after = self.start_s + 1.0
        self.completed = False

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.publish_status('initializing')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def publish_status(self, state: str, **extra) -> None:
        payload = {
            'state': state,
            'index': self.index,
            'count': len(self.goals),
            'goal_name': (
                self.goals[self.index]['name']
                if self.index < len(self.goals)
                else None
            ),
            'planner_ready': self.planner_ready,
            'have_odom': self.have_odom,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(msg)

    def odom_callback(self, _message: Odometry) -> None:
        self.have_odom = True

    def planner_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(payload.get('state', ''))
        if state == 'ready':
            self.planner_ready = True
            return

        if state == 'planning_failed' and self.goal_sent:
            self.publish_status(
                'failed',
                planner_status=payload,
            )
            self.get_logger().error('Prelude planning failed')
            os._exit(2)

        if state == 'final_waypoint_reached' and self.goal_sent:
            goal = self.goals[self.index]
            self.goal_sent = False
            self.hold_until = self.now_s() + float(goal['hold_s'])
            self.publish_status(
                'goal_reached',
                reached_name=goal['name'],
                hold_s=goal['hold_s'],
            )

    def publish_goal(self) -> None:
        goal = self.goals[self.index]
        x, y, z = goal['position']

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = yaw_quaternion(0.0)
        self.goal_pub.publish(msg)

        self.goal_sent = True
        self.publish_status(
            'goal_sent',
            goal=[x, y, z],
        )
        self.get_logger().info(
            f'PRELUDE {self.index + 1}/{len(self.goals)} '
            f'{goal["name"]}: ({x:.1f}, {y:.1f}, {z:.1f})'
        )

    def timer_callback(self) -> None:
        if self.completed:
            return

        now = self.now_s()

        if self.hold_until is not None:
            if now < self.hold_until:
                return
            self.hold_until = None
            self.index += 1
            if self.index >= len(self.goals):
                self.completed = True
                self.publish_status('prelude_completed')
                self.get_logger().warning(
                    'PRELUDE COMPLETE. Exiting process so the original '
                    'dispatcher becomes the sole mission goal owner.'
                )
                self.create_timer(0.5, self.exit_success)
                return
            self.send_after = now + 0.3

        if (
            self.planner_ready
            and self.have_odom
            and not self.goal_sent
            and self.hold_until is None
            and now >= self.send_after
        ):
            self.publish_goal()

    def exit_success(self) -> None:
        os._exit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V71PreludeRunner()
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
