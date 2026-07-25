#!/usr/bin/env python3
"""Follower-status-driven high-speed CQ-01 mission dispatcher."""
from __future__ import annotations

import json
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw_rad * 0.5)
    quaternion.w = math.cos(yaw_rad * 0.5)
    return quaternion


class FastMissionDispatcher(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v34e_complete_mission_dispatcher')
        share = get_package_share_directory('cq01_v2_scene')
        self.declare_parameter(
            'mission_file',
            os.path.join(share, 'config', 'cq01_v34e_planning_mission.yaml'),
        )
        self.declare_parameter('goal_topic', '/drone/goal_raw')
        self.declare_parameter(
            'planner_status_topic',
            '/drone/planner_3d/status',
        )
        self.declare_parameter(
            'follower_status_topic',
            '/cq01_v34e/follower_status',
        )
        self.declare_parameter(
            'mission_status_topic',
            '/cq01_v34e/mission_status',
        )
        self.declare_parameter('start_index', 0)
        self.declare_parameter('end_index', 15)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('goal_delay_s', 0.10)
        self.declare_parameter('maximum_replans_per_goal', 4)
        self.declare_parameter('target_total_time_s', 900.0)

        mission_file = os.path.expanduser(
            str(self.get_parameter('mission_file').value)
        )
        with open(mission_file, 'r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        self.mission: Dict[str, object] = document['mission']
        self.waypoints: List[Dict[str, object]] = list(
            self.mission['waypoints']
        )

        self.start_index = max(
            0,
            int(self.get_parameter('start_index').value),
        )
        requested_end = int(self.get_parameter('end_index').value)
        self.end_index = min(len(self.waypoints) - 1, requested_end)
        if self.end_index < self.start_index:
            raise ValueError('end_index must not be smaller than start_index')

        self.goal_delay = float(self.get_parameter('goal_delay_s').value)
        self.maximum_replans = int(
            self.get_parameter('maximum_replans_per_goal').value
        )
        self.target_total_time = float(
            self.get_parameter('target_total_time_s').value
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('mission_status_topic').value),
            transient_qos,
        )
        self.follower_command_publisher = self.create_publisher(
            String,
            '/cq01_v34e/follower_command',
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('planner_status_topic').value),
            self.planner_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('follower_status_topic').value),
            self.follower_status_callback,
            transient_qos,
        )

        self.start_service = self.create_service(
            Trigger,
            '~/start',
            self.start_callback,
        )
        self.stop_service = self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback,
        )

        self.planner_ready = False
        self.follower_ready = False
        self.active = False
        self.current_index = self.start_index
        self.goal_pending = False
        self.goal_sent = False
        self.send_after: Optional[float] = None
        self.replan_count = 0
        self.mission_started: Optional[float] = None
        self.goal_started: Optional[float] = None
        self.total_replans = 0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.publish_status('loaded')
        self.get_logger().info(
            'CQ-01 V3.4E complete mission loaded: '
            f'range={self.start_index}..{self.end_index}, '
            f'target_total_time={self.target_total_time:.1f}s, '
            f'start_service=~/start'
        )

        if bool(self.get_parameter('auto_start').value):
            self.active = True
            self.goal_pending = True
            self.mission_started = time.monotonic()

    def start_callback(self, _request, response):
        self.active = True
        self.current_index = self.start_index
        self.goal_pending = True
        self.goal_sent = False
        self.send_after = None
        self.replan_count = 0
        self.total_replans = 0
        self.mission_started = time.monotonic()
        self.goal_started = None
        self.publish_follower_command('reset')
        response.success = True
        response.message = (
            f'Started CQ-01 V3.4E complete mission '
            f'{self.start_index}..{self.end_index}'
        )
        self.publish_status('start_requested')
        return response

    def stop_callback(self, _request, response):
        self.active = False
        self.goal_pending = False
        self.goal_sent = False
        self.publish_follower_command('stop')
        response.success = True
        response.message = 'CQ-01 V3.4E mission stopped'
        self.publish_status('stopped')
        return response

    def planner_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(status.get('state', ''))
        if state == 'ready':
            self.planner_ready = True
            self.publish_status('planner_ready')
            return
        if not self.active:
            return
        if state == 'planning_failed':
            self.handle_execution_failure(
                'planning_failed',
                planner_status=status,
            )
        elif state == 'path_ready':
            self.publish_status(
                'path_ready',
                planner_status=status,
            )

    def follower_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = str(status.get('state', ''))
        if state == 'map_ready':
            self.follower_ready = True
            self.publish_status('follower_ready')
            return
        if not self.active:
            return
        if state == 'tracking':
            # Republish useful speed/progress data at the mission level.
            self.publish_status(
                'tracking',
                follower_status=status,
            )
            return
        if state == 'stalled':
            self.handle_execution_failure(
                'follower_stalled',
                follower_status=status,
            )
            return
        if state != 'goal_reached':
            return

        reached = self.current_index
        waypoint = self.waypoints[reached]
        name = str(waypoint['name'])
        now = time.monotonic()
        segment_elapsed = (
            now - self.goal_started if self.goal_started is not None else None
        )
        self.get_logger().info(
            f'Reached complete mission waypoint {reached}: {name}; '
            f'segment_time={segment_elapsed:.2f}s'
            if segment_elapsed is not None
            else f'Reached complete mission waypoint {reached}: {name}'
        )

        if reached >= self.end_index:
            self.active = False
            self.goal_pending = False
            self.goal_sent = False
            elapsed = (
                now - self.mission_started
                if self.mission_started is not None else 0.0
            )
            self.publish_status(
                'mission_completed',
                reached_index=reached,
                reached_name=name,
                elapsed_s=elapsed,
                target_total_time_s=self.target_total_time,
                within_time_target=elapsed <= self.target_total_time,
                total_replans=self.total_replans,
            )
            self.get_logger().info(
                f'CQ-01 V3.4E mission completed in {elapsed:.2f}s '
                f'(target <= {self.target_total_time:.1f}s)'
            )
            return

        self.current_index += 1
        self.replan_count = 0
        self.goal_sent = False
        self.goal_pending = True
        self.send_after = now + self.goal_delay
        self.publish_status(
            'advancing',
            reached_index=reached,
            reached_name=name,
            segment_elapsed_s=segment_elapsed,
        )

    def handle_execution_failure(self, reason: str, **extra: object) -> None:
        if self.replan_count >= self.maximum_replans:
            self.active = False
            self.goal_pending = False
            self.goal_sent = False
            self.publish_follower_command('stop')
            self.publish_status(
                'mission_failed',
                reason=reason,
                replans=self.replan_count,
                **extra,
            )
            self.get_logger().error(
                f'Complete mission failed at waypoint {self.current_index}: '
                f'{reason}, replans={self.replan_count}'
            )
            return
        self.replan_count += 1
        self.total_replans += 1
        self.goal_sent = False
        self.goal_pending = True
        self.send_after = time.monotonic() + 0.25
        self.publish_status(
            'replan_requested',
            reason=reason,
            replan_count=self.replan_count,
            **extra,
        )
        self.get_logger().warning(
            f'Replanning waypoint {self.current_index}: '
            f'{self.replan_count}/{self.maximum_replans}, reason={reason}'
        )

    def timer_callback(self) -> None:
        if not (
            self.active
            and self.planner_ready
            and self.follower_ready
            and self.goal_pending
            and not self.goal_sent
        ):
            return
        now = time.monotonic()
        if self.send_after is not None and now < self.send_after:
            return
        self.publish_current_goal()

    def publish_current_goal(self) -> None:
        waypoint = self.waypoints[self.current_index]
        x, y, z = (float(value) for value in waypoint['position'])
        yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))
        message = PoseStamped()
        message.header.frame_id = str(self.mission.get('frame_id', 'map'))
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation = yaw_to_quaternion(yaw)
        self.goal_publisher.publish(message)
        self.goal_sent = True
        self.goal_pending = False
        self.goal_started = time.monotonic()
        self.publish_status(
            'goal_sent',
            waypoint_name=str(waypoint['name']),
            goal=[x, y, z],
            replan_count=self.replan_count,
        )
        self.get_logger().info(
            f'Published fast planning goal {self.current_index}: '
            f'{waypoint["name"]} -> ({x:.2f}, {y:.2f}, {z:.2f})'
        )


    def publish_follower_command(self, command: str) -> None:
        message = String()
        message.data = command
        self.follower_command_publisher.publish(message)

    def publish_status(self, state: str, **extra: object) -> None:
        elapsed = (
            time.monotonic() - self.mission_started
            if self.mission_started is not None else 0.0
        )
        payload: Dict[str, object] = {
            'state': state,
            'active': self.active,
            'current_index': self.current_index,
            'start_index': self.start_index,
            'end_index': self.end_index,
            'planner_ready': self.planner_ready,
            'follower_ready': self.follower_ready,
            'elapsed_s': elapsed,
            'target_total_time_s': self.target_total_time,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastMissionDispatcher()
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
