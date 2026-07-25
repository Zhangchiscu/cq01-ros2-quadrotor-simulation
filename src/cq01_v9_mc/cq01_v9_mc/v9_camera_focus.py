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

Point3 = Tuple[float, float, float]


class V9CameraFocus(Node):
    """Low-rate focus target with stable bridge and dynamic presets."""

    def __init__(self) -> None:
        super().__init__('cq01_v9_camera_focus')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf = TransformBroadcaster(self)
        self.position: Point3 = (0.0, 0.0, 12.0)
        self.drone: Optional[Point3] = None
        self.route: Dict = {}
        self.low: Dict = {}
        self.h14: Dict = {}
        self.dynamic: Dict = {}
        self.pattern: Dict = {}
        self.multi_status: Dict = {}
        self.aux_uav1: Optional[Point3] = None
        self.aux_uav2: Optional[Point3] = None
        self.max_reached = -1
        self.last = self.now_s()

        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            '/uav1/drone/odom',
            lambda msg: self.aux_odom_callback('uav1', msg),
            20,
        )
        self.create_subscription(
            Odometry,
            '/uav2/drone/odom',
            lambda msg: self.aux_odom_callback('uav2', msg),
            20,
        )
        self.create_subscription(
            String,
            '/cq01_final_showcase/multi_uav_status',
            lambda msg: self.json_callback('multi_status', msg),
            transient,
        )

        for target, topic in (
            ('route', '/cq01_v9/route/status'),
            ('low', '/cq01_v9/low_strong/status'),
            ('h14', '/cq01_v9/h14/status'),
            ('dynamic', '/cq01_v9/dynamic/status'),
            ('pattern', '/cq01_v9/pattern/status'),
        ):
            self.create_subscription(
                String,
                topic,
                lambda msg, target=target:
                    self.json_callback(target, msg),
                transient,
            )

        # 10 Hz is sufficient for a camera target and avoids fighting
        # manual mouse interaction inside a virtual machine.
        self.timer = self.create_timer(
            0.10,
            self.timer_callback,
        )

        self.get_logger().warning(
            'V9-MC.5 camera focus at 10 Hz; '
            'stable bridge focus and dynamic midpoint enabled.'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds/1.0e9

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.drone = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

    def aux_odom_callback(
        self,
        target: str,
        msg: Odometry,
    ) -> None:
        p = msg.pose.pose.position
        value = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        if target == 'uav1':
            self.aux_uav1 = value
        else:
            self.aux_uav2 = value

    def json_callback(
        self,
        target: str,
        msg: String,
    ) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        setattr(self, target, data)
        if target == 'route' and 'reached_index' in data:
            self.max_reached = max(
                self.max_reached,
                int(data.get('reached_index', -1)),
            )

    @staticmethod
    def midpoint(
        first: Point3,
        second: Point3,
    ) -> Point3:
        return (
            0.5*(first[0]+second[0]),
            0.5*(first[1]+second[1]),
            0.5*(first[2]+second[2]),
        )

    def desired(self) -> Tuple[Point3, float]:
        # CQ01 V14 STABLE_STAGE_CAMERA
        #
        # 不再连续跟随无人机、动态障碍物或轨迹中点。
        # 每个大区间只切换一次固定焦点，阶段内 TF 完全静止，
        # 因而 RViz 滚轮缩放和手动旋转不会再被焦点运动抵消。
        if bool(self.multi_status.get('activated', False)):
            return (400.0, 65.0, 255.0), 1.0

        if self.max_reached < 0:
            return (35.0, -60.0, 22.0), 1.0
        if self.max_reached <= 2:
            return (55.0, -82.0, 28.0), 1.0
        if self.max_reached <= 5:
            return (128.0, 3.0, 72.0), 1.0
        if self.max_reached <= 8:
            return (215.0, 8.0, 34.0), 1.0
        if self.max_reached <= 12:
            return (312.0, -34.0, 116.0), 1.0

        # P13-P15、圆形、八字阶段均固定观察山顶区域。
        return (400.0, 45.0, 238.0), 1.0

        if (
            bool(self.multi_status.get('activated', False))
            and self.aux_uav1 is not None
            and self.aux_uav2 is not None
        ):
            # 多无人机启动后，相机自动切到两架辅助机中点。
            return self.midpoint(
                self.aux_uav1,
                self.aux_uav2,
            ), 0.20

        pattern_state = str(
            self.pattern.get('state', '')
        )
        if pattern_state not in (
            '',
            'idle',
            'waiting_for_start',
        ):
            return (400.0, 45.0, 248.0), 0.55

        if (
            bool(self.dynamic.get('active', False))
            and self.drone is not None
        ):
            obstacle = self.dynamic.get(
                'obstacle_center'
            )
            if (
                isinstance(obstacle, list)
                and len(obstacle) == 3
            ):
                target = self.midpoint(
                    self.drone,
                    (
                        float(obstacle[0]),
                        float(obstacle[1]),
                        float(obstacle[2]),
                    ),
                )
                return target, 0.40

        h14_state = str(
            self.h14.get('state', '')
        )
        if (
            (
                self.max_reached >= 12
                and self.max_reached < 14
            )
            or h14_state.startswith(
                'high_altitude'
            )
        ):
            return (342.0, -27.0, 174.0), 0.45

        current_index = int(
            self.route.get('current_index', -1)
        )
        if current_index in (7, 8):
            # A fixed bridge/gorge focus is easier to orbit manually than
            # a target that follows every odometry sample.
            return (216.0, 5.0, 24.0), 0.40

        low_state = str(
            self.low.get('state', '')
        )
        if low_state in (
            'active',
            'recovery',
            'recovered',
        ):
            return (50.0, -100.0, 24.0), 0.40

        if self.drone is not None:
            return self.drone, 0.90

        return self.position, 1.0

    def timer_callback(self) -> None:
        now = self.now_s()
        dt = max(
            0.0,
            min(0.3, now-self.last),
        )
        self.last = now

        target, tau = self.desired()
        alpha = 1.0-math.exp(
            -dt/max(0.05, tau)
        )
        self.position = tuple(
            current + alpha*(desired-current)
            for current, desired
            in zip(self.position, target)
        )

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )
        transform.header.frame_id = 'world'
        transform.child_frame_id = (
            'cq01_v9_camera_focus'
        )
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = self.position
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V9CameraFocus()
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
