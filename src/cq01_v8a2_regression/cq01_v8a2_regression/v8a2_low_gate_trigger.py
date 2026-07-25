#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class V8A2LowGateTrigger(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v8a2_low_gate_trigger')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.center = (52.0, -90.0, 25.0)
        self.radius_xy = 7.0
        self.tolerance_z = 8.0
        self.mission: Dict = {}
        self.position: Optional[Tuple[float, float, float]] = None

        # The original dispatcher publishes 'advancing' only once, immediately
        # after P1 is reached. The aircraft reaches the spatial gate later,
        # after status has already changed to goal_sent/path_ready. Therefore
        # P1 completion must be latched instead of requiring the latest status
        # to remain exactly 'advancing'.
        self.maximum_reached_index = -1
        self.segment_armed = False
        self.segment_arm_logged = False
        self.minimum_gate_distance_xy_m: Optional[float] = None
        self.minimum_gate_height_error_m: Optional[float] = None

        self.request_sent = False
        self.trigger_accepted = False
        self.trigger_position: Optional[Tuple[float, float, float]] = None
        self.trigger_distance_xy: Optional[float] = None
        self.failure: Optional[str] = None

        self.client = self.create_client(Trigger, '/cq01_v8a2_low_disturbance/trigger')
        self.status_pub = self.create_publisher(String, '/cq01_v8a2/low_gate_status', transient)
        self.marker_pub = self.create_publisher(MarkerArray, '/cq01_v8a2/low_gate_markers', transient)
        self.create_subscription(String, '/cq01_v8a2/mission_status', self.mission_cb, transient)
        self.create_subscription(Odometry, '/drone/odom', self.odom_cb, 50)
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.marker_timer = self.create_timer(0.5, self.publish_marker)

    def mission_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        self.mission = payload

        if 'reached_index' in payload:
            try:
                self.maximum_reached_index = max(
                    self.maximum_reached_index,
                    int(payload.get('reached_index', -1)),
                )
            except (TypeError, ValueError):
                pass

        state = str(payload.get('state', ''))
        current_index = int(payload.get('current_index', -1) or -1)
        active = bool(payload.get('active', False))

        # Arm on the authoritative P1 advancing event, then keep the phase
        # armed through later goal_sent/path_ready messages while index 2 is
        # active. This removes the transient-status timing race.
        if (
            state == 'advancing'
            and self.maximum_reached_index >= 1
            and current_index == 2
        ):
            self.segment_armed = True

        if (
            active
            and current_index == 2
            and self.maximum_reached_index >= 1
            and self.maximum_reached_index < 2
        ):
            self.segment_armed = True

        if self.segment_armed and not self.segment_arm_logged:
            self.segment_arm_logged = True
            self.get_logger().warning(
                'LOW-CITY P1->P2 SEGMENT LATCHED. '
                'Waiting for spatial gate center=(52,-90,25).'
            )

        # If P2 has already been reached without a trigger, record an explicit
        # failure rather than silently leaving only the gate marker visible.
        if (
            not self.request_sent
            and not self.trigger_accepted
            and self.maximum_reached_index >= 2
            and self.failure is None
        ):
            self.failure = (
                'gate_missed_before_P2: '
                f'min_xy={self.minimum_gate_distance_xy_m}, '
                f'min_z_error={self.minimum_gate_height_error_m}'
            )
            self.get_logger().error(
                'LOW-CITY WIND GATE MISSED BEFORE P2. '
                f'{self.failure}'
            )

        self.publish_status()

    def odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

        if self.segment_armed and not self.request_sent:
            x, y, z = self.position
            cx, cy, cz = self.center
            distance_xy = math.hypot(x - cx, y - cy)
            height_error = abs(z - cz)

            if (
                self.minimum_gate_distance_xy_m is None
                or distance_xy < self.minimum_gate_distance_xy_m
            ):
                self.minimum_gate_distance_xy_m = distance_xy

            if (
                self.minimum_gate_height_error_m is None
                or height_error < self.minimum_gate_height_error_m
            ):
                self.minimum_gate_height_error_m = height_error

    def inside_gate(self) -> bool:
        if self.position is None:
            return False
        x, y, z = self.position
        cx, cy, cz = self.center
        distance_xy = math.hypot(x - cx, y - cy)
        return distance_xy <= self.radius_xy and abs(z - cz) <= self.tolerance_z

    def correct_segment(self) -> bool:
        try:
            current_index = int(
                self.mission.get('current_index', -1)
            )
        except (TypeError, ValueError):
            current_index = -1

        return (
            self.segment_armed
            and bool(self.mission.get('active', False))
            and current_index == 2
            and self.maximum_reached_index >= 1
            and self.maximum_reached_index < 2
        )

    def timer_cb(self) -> None:
        if self.request_sent or self.failure:
            self.publish_status()
            return
        if not self.correct_segment() or not self.inside_gate():
            self.publish_status()
            return
        if not self.client.service_is_ready():
            self.publish_status()
            return

        self.get_logger().warning(
            'LOW-CITY SPATIAL GATE ENTERED. '
            f'position={self.position}; calling disturbance trigger service.'
        )
        self.request_sent = True
        self.trigger_position = self.position
        if self.position is not None:
            self.trigger_distance_xy = math.hypot(
                self.position[0] - self.center[0],
                self.position[1] - self.center[1],
            )
        future = self.client.call_async(Trigger.Request())
        future.add_done_callback(self.trigger_done)
        self.publish_status()

    def trigger_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exception:
            self.failure = f'service_exception: {exception}'
            self.publish_status()
            return
        if not response.success:
            self.failure = f'trigger_rejected: {response.message}'
        else:
            self.trigger_accepted = True
            self.get_logger().warning(
                'LOW-CITY WIND TRIGGER ACCEPTED near original gate '
                f'{self.center}; position={self.trigger_position}'
            )
        self.publish_status()

    def publish_status(self) -> None:
        payload = {
            'state': (
                'failed' if self.failure
                else 'triggered' if self.trigger_accepted
                else 'request_sent' if self.request_sent
                else 'waiting_for_original_gate'
            ),
            'gate_center': list(self.center),
            'gate_radius_xy_m': self.radius_xy,
            'gate_tolerance_z_m': self.tolerance_z,
            'mission_state': self.mission.get('state'),
            'mission_active': bool(self.mission.get('active', False)),
            'mission_current_index':
                self.mission.get('current_index'),
            'maximum_reached_index':
                self.maximum_reached_index,
            'segment_armed': self.segment_armed,
            'correct_segment': self.correct_segment(),
            'inside_gate': self.inside_gate(),
            'service_ready': self.client.service_is_ready(),
            'minimum_gate_distance_xy_m':
                self.minimum_gate_distance_xy_m,
            'minimum_gate_height_error_m':
                self.minimum_gate_height_error_m,
            'trigger_accepted': self.trigger_accepted,
            'trigger_position': list(self.trigger_position) if self.trigger_position else None,
            'trigger_distance_xy_m': self.trigger_distance_xy,
            'failure': self.failure,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(message)

    def publish_marker(self) -> None:
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        sphere = Marker()
        sphere.header.frame_id = 'world'
        sphere.header.stamp = now
        sphere.ns = 'v8a2_low_gate'
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = self.center
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 2.0 * self.radius_xy
        sphere.scale.y = 2.0 * self.radius_xy
        sphere.scale.z = 2.0
        sphere.color.r = 0.1
        sphere.color.g = 0.85
        sphere.color.b = 1.0
        sphere.color.a = 0.18
        array.markers.append(sphere)

        text = Marker()
        text.header.frame_id = 'world'
        text.header.stamp = now
        text.ns = 'v8a2_low_gate'
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.center[0]
        text.pose.position.y = self.center[1]
        text.pose.position.z = self.center[2] + 8.0
        text.pose.orientation.w = 1.0
        text.scale.z = 1.7
        text.color.r = 0.2
        text.color.g = 0.95
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = 'LOW-CITY WIND GATE | P1→P2'
        array.markers.append(text)
        self.marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V8A2LowGateTrigger()
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
