#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
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
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw_rad * 0.5)
    quaternion.w = math.cos(yaw_rad * 0.5)
    return quaternion


class PlanningMissionDispatcher(Node):
    """Planner-status-driven CQ-01 mission sequencer.

    A goal is published exactly once.  The next goal is not issued until the
    A* planner reports final_waypoint_reached, avoiding the repeated replanning
    caused by a periodic PoseStamped publisher.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v34a_planning_mission_dispatcher')
        share = get_package_share_directory('cq01_v2_scene')
        self.declare_parameter(
            'mission_file',
            os.path.join(share, 'config', 'cq01_v34a_planning_mission.yaml'),
        )
        self.declare_parameter('goal_topic', '/drone/goal_raw')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'planner_status_topic',
            '/drone/planner_3d/status',
        )
        self.declare_parameter(
            'mission_status_topic',
            '/cq01_v34a/mission_status',
        )
        self.declare_parameter('start_index', 0)
        self.declare_parameter('end_index', 0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('goal_delay_s', 0.5)
        self.declare_parameter('stall_timeout_s', 8.0)
        self.declare_parameter('minimum_progress_m', 0.50)
        self.declare_parameter('maximum_replans_per_goal', 3)

        mission_file = str(self.get_parameter('mission_file').value)
        with open(mission_file, 'r', encoding='utf-8') as handle:
            self.mission: Dict = yaml.safe_load(handle)['mission']
        self.waypoints: List[Dict] = list(
            self.mission.get('waypoints', [])
        )
        if not self.waypoints:
            raise RuntimeError('Planning mission has no waypoints')

        requested_start = int(self.get_parameter('start_index').value)
        requested_end = int(self.get_parameter('end_index').value)
        self.start_index = max(
            0,
            min(requested_start, len(self.waypoints) - 1),
        )
        self.end_index = max(
            self.start_index,
            min(requested_end, len(self.waypoints) - 1),
        )
        self.current_index = self.start_index
        self.active = bool(self.get_parameter('auto_start').value)
        self.planner_ready = False
        self.have_odom = False
        self.goal_pending = False
        self.goal_sent = False
        self.send_after: Optional[float] = None
        self.goal_delay = max(
            0.0,
            float(self.get_parameter('goal_delay_s').value),
        )
        self.stall_timeout = max(2.0, float(self.get_parameter('stall_timeout_s').value))
        self.minimum_progress = max(0.05, float(self.get_parameter('minimum_progress_m').value))
        self.maximum_replans = max(0, int(self.get_parameter('maximum_replans_per_goal').value))
        self.latest_position: Optional[tuple] = None
        self.current_goal: Optional[tuple] = None
        self.best_goal_distance: Optional[float] = None
        self.last_progress_time: Optional[float] = None
        self.replan_count = 0

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
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            50,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('planner_status_topic').value),
            self.planner_status_callback,
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
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.publish_status('loaded')
        self.get_logger().info(
            f'CQ-01 planning mission loaded: range='
            f'{self.start_index}..{self.end_index}, '
            f'active={self.active}. Start service: ~/{"start"}'
        )

    def publish_status(
        self,
        state: str,
        **extra,
    ) -> None:
        payload = {
            'state': state,
            'active': self.active,
            'current_index': self.current_index,
            'start_index': self.start_index,
            'end_index': self.end_index,
            'planner_ready': self.planner_ready,
            'have_odom': self.have_odom,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def start_callback(self, _request, response):
        self.active = True
        self.current_index = self.start_index
        self.goal_pending = True
        self.goal_sent = False
        self.send_after = None
        self.current_goal = None
        self.best_goal_distance = None
        self.last_progress_time = None
        self.replan_count = 0
        response.success = True
        response.message = (
            f'Started CQ-01 planning mission '
            f'{self.start_index}..{self.end_index}'
        )
        self.publish_status('start_requested')
        return response

    def stop_callback(self, _request, response):
        self.active = False
        self.goal_pending = False
        self.goal_sent = False
        self.send_after = None
        response.success = True
        response.message = 'CQ-01 planning mission stopped'
        self.publish_status('stopped')
        return response

    def odom_callback(self, message: Odometry) -> None:
        self.have_odom = True
        p = message.pose.pose.position
        self.latest_position = (float(p.x), float(p.y), float(p.z))
        if not (self.active and self.goal_sent and self.current_goal):
            return
        distance = math.dist(self.latest_position, self.current_goal)
        now = self.get_clock().now().nanoseconds / 1.0e9
        if (
            self.best_goal_distance is None
            or distance <= self.best_goal_distance - self.minimum_progress
        ):
            self.best_goal_distance = distance
            self.last_progress_time = now

    def planner_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f'Ignoring malformed planner status: {message.data}'
            )
            return
        state = str(status.get('state', ''))
        if state == 'ready':
            self.planner_ready = True
            if self.active and not self.goal_sent:
                self.goal_pending = True
            self.publish_status('planner_ready')
            return

        if not self.active:
            return
        if state == 'planning_failed':
            self.active = False
            self.goal_pending = False
            self.publish_status(
                'planning_failed',
                planner_status=status,
            )
            self.get_logger().error(
                f'Planning failed at index {self.current_index}: '
                f'{status.get("reason", "unknown")}'
            )
            return
        if state == 'path_ready':
            self.publish_status(
                'path_ready',
                planner_status=status,
            )
            return
        if state == 'final_waypoint_reached':
            reached = self.current_index
            name = str(self.waypoints[reached]['name'])
            self.get_logger().info(
                f'Reached planning waypoint {reached}: {name}'
            )
            if reached >= self.end_index:
                self.active = False
                self.goal_pending = False
                self.goal_sent = False
                self.publish_status(
                    'mission_completed',
                    reached_index=reached,
                    reached_name=name,
                )
                return
            self.current_index += 1
            self.replan_count = 0
            self.current_goal = None
            self.best_goal_distance = None
            self.last_progress_time = None
            self.goal_pending = True
            self.goal_sent = False
            self.send_after = (
                self.get_clock().now().nanoseconds / 1.0e9
                + self.goal_delay
            )
            self.publish_status(
                'advancing',
                reached_index=reached,
                reached_name=name,
            )

    def timer_callback(self) -> None:
        now = self.get_clock().now().nanoseconds / 1.0e9
        if (
            self.active
            and self.goal_sent
            and self.current_goal is not None
            and self.last_progress_time is not None
            and now - self.last_progress_time >= self.stall_timeout
        ):
            if self.replan_count >= self.maximum_replans:
                self.active = False
                self.goal_pending = False
                self.goal_sent = False
                self.publish_status(
                    'stalled',
                    waypoint_name=str(self.waypoints[self.current_index]['name']),
                    replans=self.replan_count,
                )
                self.get_logger().error(
                    f'Mission stalled at waypoint {self.current_index} after '
                    f'{self.replan_count} replans'
                )
                return
            self.replan_count += 1
            self.goal_pending = True
            self.goal_sent = False
            self.send_after = now + 0.2
            self.last_progress_time = now
            if self.latest_position is not None:
                self.best_goal_distance = math.dist(
                    self.latest_position, self.current_goal
                )
            self.publish_status(
                'stall_replan_requested',
                waypoint_name=str(self.waypoints[self.current_index]['name']),
                replan_count=self.replan_count,
            )
            self.get_logger().warning(
                f'No meaningful progress for {self.stall_timeout:.1f}s; '
                f'replanning waypoint {self.current_index} '
                f'({self.replan_count}/{self.maximum_replans})'
            )
            return

        if not (
            self.active
            and self.planner_ready
            and self.have_odom
            and self.goal_pending
            and not self.goal_sent
        ):
            return
        if self.send_after is not None and now < self.send_after:
            return
        self.publish_current_goal()

    def publish_current_goal(self) -> None:
        waypoint = self.waypoints[self.current_index]
        x, y, z = (float(value) for value in waypoint['position'])
        yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))
        message = PoseStamped()
        message.header.frame_id = str(
            self.mission.get('frame_id', 'map')
        )
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation = yaw_to_quaternion(yaw)
        self.goal_publisher.publish(message)
        self.goal_sent = True
        self.goal_pending = False
        self.current_goal = (x, y, z)
        self.best_goal_distance = (
            math.dist(self.latest_position, self.current_goal)
            if self.latest_position is not None else None
        )
        self.last_progress_time = self.get_clock().now().nanoseconds / 1.0e9
        self.send_after = None
        self.publish_status(
            'goal_sent',
            waypoint_name=str(waypoint['name']),
            goal=[x, y, z],
        )
        self.get_logger().info(
            f'Published planning goal {self.current_index}: '
            f'{waypoint["name"]} -> ({x:.2f}, {y:.2f}, {z:.2f})'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningMissionDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
