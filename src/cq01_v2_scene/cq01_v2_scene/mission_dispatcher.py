#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import yaml


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


class MissionDispatcher(Node):
    """Generic PoseStamped waypoint dispatcher.

    This node is deliberately disabled by default in the launch file because the
    existing RAPTOR stack may use a custom goal interface. Enable it only when
    the planner accepts geometry_msgs/PoseStamped on the selected goal topic and
    publishes nav_msgs/Odometry on the selected odometry topic.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v2_mission_dispatcher')
        share = get_package_share_directory('cq01_v2_scene')
        default_mission = os.path.join(share, 'config', 'cq01_v2_mission.yaml')

        self.declare_parameter('mission_file', default_mission)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/quadrotor/odom')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('republish_period_s', 1.0)

        mission_file = self.get_parameter('mission_file').value
        with open(mission_file, 'r', encoding='utf-8') as handle:
            self.mission: Dict = yaml.safe_load(handle)['mission']

        self.frame_id = str(self.mission.get('frame_id', 'world'))
        self.waypoints: List[Dict] = self.mission.get('waypoints', [])
        self.goal_tolerance = float(self.mission.get('goal_tolerance', 0.8))
        self.current_index = 1 if len(self.waypoints) > 1 else 0
        self.active = bool(self.get_parameter('auto_start').value)
        self.have_odom = False
        self.position = (0.0, 0.0, 0.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter('goal_topic').value), qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.on_odom,
            qos,
        )
        self.start_srv = self.create_service(Trigger, '~/start', self.on_start)
        self.stop_srv = self.create_service(Trigger, '~/stop', self.on_stop)
        self.timer = self.create_timer(float(self.get_parameter('republish_period_s').value), self.on_timer)

        self.get_logger().info(
            f'Dispatcher loaded {len(self.waypoints)} waypoints. active={self.active}; '
            'enable only if your planner uses PoseStamped goals.'
        )

    def on_start(self, _request, response):
        self.active = True
        response.success = True
        response.message = f'Started at waypoint index {self.current_index}'
        self.publish_current_goal()
        return response

    def on_stop(self, _request, response):
        self.active = False
        response.success = True
        response.message = 'Mission dispatcher stopped'
        return response

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)
        self.have_odom = True
        if not self.active or self.current_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_index]['position']
        distance = math.dist(self.position, tuple(float(v) for v in target))
        if distance <= self.goal_tolerance:
            name = self.waypoints[self.current_index]['name']
            self.get_logger().info(f'Reached {name}; error={distance:.3f} m')
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.active = False
                self.get_logger().info('CQ-01 V2 waypoint mission completed.')
            else:
                self.publish_current_goal()

    def on_timer(self) -> None:
        if self.active and self.have_odom and self.current_index < len(self.waypoints):
            self.publish_current_goal()

    def publish_current_goal(self) -> None:
        if self.current_index >= len(self.waypoints):
            return
        waypoint = self.waypoints[self.current_index]
        x, y, z = (float(v) for v in waypoint['position'])
        yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = yaw_to_quaternion(yaw)
        self.goal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
