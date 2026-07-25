#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0*(w*x+y*z)
    cosr_cosp = 1.0-2.0*(x*x+y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0*(w*y-z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0*(w*z+x*y)
    cosy_cosp = 1.0-2.0*(y*y+z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class CourseAcceptanceRunner(Node):
    """Exact implementation of the coordinates listed in the assignment.

    Sequence:
    1. Hover at (0, 0, 1.5)
    2. Fly to (2, 1, 1.5)
    3. Fly through four positive-coordinate square waypoints
    4. Return to (0, 0, 1.5) and hold
    """

    def __init__(self) -> None:
        super().__init__('cq01_course_acceptance_runner')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('goal_topic', '/drone/goal')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter('rpm_topic', '/drone/motor_rpm')
        self.declare_parameter(
            'status_topic',
            '/cq01_course/status',
        )
        self.declare_parameter(
            'waypoint_marker_topic',
            '/cq01_course/waypoints',
        )
        self.declare_parameter(
            'results_root',
            '~/drone_task/your_drone_sim_ws/results/'
            'cq01_course_acceptance',
        )
        self.declare_parameter('position_tolerance_m', 0.30)
        self.declare_parameter('hold_speed_mps', 0.40)
        self.declare_parameter('maximum_tilt_deg', 35.0)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter('stage_timeout_s', 35.0)
        self.declare_parameter('sample_rate_hz', 20.0)

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.position_tolerance = float(
            self.get_parameter(
                'position_tolerance_m'
            ).value
        )
        self.hold_speed = float(
            self.get_parameter(
                'hold_speed_mps'
            ).value
        )
        self.maximum_tilt_limit = float(
            self.get_parameter(
                'maximum_tilt_deg'
            ).value
        )
        self.maximum_rpm_limit = float(
            self.get_parameter('maximum_rpm').value
        )
        self.stage_timeout = float(
            self.get_parameter(
                'stage_timeout_s'
            ).value
        )
        self.results_root = Path(
            os.path.expanduser(
                str(
                    self.get_parameter(
                        'results_root'
                    ).value
                )
            )
        )

        self.stages: List[Dict] = [
            {
                'name': 'hover_0_0_1_5',
                'kind': 'hover',
                'goal': (0.0, 0.0, 1.5),
                'hold_s': 5.0,
            },
            {
                'name': 'single_target_2_1_1_5',
                'kind': 'single_target',
                'goal': (2.0, 1.0, 1.5),
                'hold_s': 5.0,
            },
            {
                'name': 'multi_target_1',
                'kind': 'multi_target',
                'goal': (3.0, 1.0, 1.5),
                'hold_s': 1.5,
            },
            {
                'name': 'multi_target_2',
                'kind': 'multi_target',
                'goal': (3.0, 3.0, 1.5),
                'hold_s': 1.5,
            },
            {
                'name': 'multi_target_3',
                'kind': 'multi_target',
                'goal': (1.0, 3.0, 1.5),
                'hold_s': 1.5,
            },
            {
                'name': 'multi_target_4',
                'kind': 'multi_target',
                'goal': (1.0, 1.0, 1.5),
                'hold_s': 1.5,
            },
            {
                'name': 'final_return_hover',
                'kind': 'final_hold',
                'goal': (0.0, 0.0, 1.5),
                'hold_s': 3.0,
            },
        ]

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            20,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient,
        )
        self.waypoint_pub = self.create_publisher(
            MarkerArray,
            str(
                self.get_parameter(
                    'waypoint_marker_topic'
                ).value
            ),
            transient,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('rpm_topic').value),
            self.rpm_callback,
            20,
        )

        self.position: Optional[Point3] = None
        self.velocity: Point3 = (0.0, 0.0, 0.0)
        self.rpy_deg: Point3 = (0.0, 0.0, 0.0)
        self.rpm: List[float] = [0.0, 0.0, 0.0, 0.0]

        self.started = False
        self.completed = False
        self.failed = False
        self.failure_reason = ''
        self.stage_index = 0
        self.stage_start: Optional[float] = None
        self.hold_start: Optional[float] = None
        self.mission_start: Optional[float] = None
        self.output_dir: Optional[Path] = None

        self.samples: List[Dict] = []
        self.stage_results: List[Dict] = []
        self.maximum_tilt_seen = 0.0
        self.maximum_rpm_seen = 0.0
        self.rpm_samples = 0
        self.rpm_saturated = 0
        self.saved = False

        rate = max(
            5.0,
            float(
                self.get_parameter(
                    'sample_rate_hz'
                ).value
            ),
        )
        self.timer = self.create_timer(
            1.0/rate,
            self.timer_callback,
        )

        self.publish_waypoint_markers()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds/1.0e9

    def odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        v = message.twist.twist.linear
        q = message.pose.pose.orientation

        self.position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )
        self.velocity = (
            float(v.x),
            float(v.y),
            float(v.z),
        )

        roll, pitch, yaw = quaternion_to_rpy(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        self.rpy_deg = (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )
        self.maximum_tilt_seen = max(
            self.maximum_tilt_seen,
            abs(self.rpy_deg[0]),
            abs(self.rpy_deg[1]),
        )

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        values = [float(value) for value in message.data]
        values = (values+[0.0, 0.0, 0.0, 0.0])[:4]
        self.rpm = values
        self.rpm_samples += 1

        peak = max(abs(value) for value in values)
        self.maximum_rpm_seen = max(
            self.maximum_rpm_seen,
            peak,
        )
        if peak >= 0.98*self.maximum_rpm_limit:
            self.rpm_saturated += 1

    def current_stage(self) -> Dict:
        return self.stages[
            min(self.stage_index, len(self.stages)-1)
        ]

    def current_error(self) -> Optional[float]:
        if self.position is None:
            return None
        return math.dist(
            self.position,
            self.current_stage()['goal'],
        )

    def current_speed(self) -> float:
        return math.sqrt(
            sum(value*value for value in self.velocity)
        )

    def publish_goal(self) -> None:
        stage = self.current_stage()
        x, y, z = stage['goal']

        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def start_mission(self, now: float) -> None:
        stamp = datetime.now().strftime(
            'course_acceptance_%Y%m%d_%H%M%S'
        )
        self.output_dir = self.results_root/stamp
        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )
        self.started = True
        self.mission_start = now
        self.stage_start = now
        self.get_logger().warning(
            'COURSE ACCEPTANCE START: '
            'hover (0,0,1.5), target (2,1,1.5), '
            'four positive-coordinate waypoints.'
        )

    def complete_stage(
        self,
        now: float,
        error: float,
    ) -> None:
        stage = self.current_stage()
        elapsed = now-(self.stage_start or now)
        mission_elapsed = now-(self.mission_start or now)

        result = {
            'index': self.stage_index,
            'name': stage['name'],
            'kind': stage['kind'],
            'goal': list(stage['goal']),
            'completed': True,
            'completion_error_m': error,
            'stage_time_s': elapsed,
            'mission_time_s': mission_elapsed,
        }
        self.stage_results.append(result)

        self.get_logger().warning(
            f'COURSE STAGE PASS {self.stage_index}: '
            f'{stage["name"]}; error={error:.4f} m; '
            f'time={elapsed:.2f} s.'
        )

        if self.stage_index >= len(self.stages)-1:
            self.completed = True
            self.save_result()
            return

        self.stage_index += 1
        self.stage_start = now
        self.hold_start = None

    def fail(self, reason: str) -> None:
        if self.completed or self.failed:
            return
        self.failed = True
        self.failure_reason = reason
        self.get_logger().error(
            f'COURSE ACCEPTANCE FAILED: {reason}'
        )
        self.save_result()

    def timer_callback(self) -> None:
        if self.position is None:
            return

        now = self.now_s()
        if not self.started:
            self.start_mission(now)

        self.publish_goal()
        self.publish_waypoint_markers()

        error = self.current_error()
        if error is None:
            return

        speed = self.current_speed()
        stage = self.current_stage()

        self.samples.append({
            'time_s': now,
            'mission_elapsed_s':
                now-(self.mission_start or now),
            'stage_index': self.stage_index,
            'stage_name': stage['name'],
            'stage_kind': stage['kind'],
            'goal_x_m': stage['goal'][0],
            'goal_y_m': stage['goal'][1],
            'goal_z_m': stage['goal'][2],
            'x_m': self.position[0],
            'y_m': self.position[1],
            'z_m': self.position[2],
            'vx_mps': self.velocity[0],
            'vy_mps': self.velocity[1],
            'vz_mps': self.velocity[2],
            'speed_mps': speed,
            'position_error_m': error,
            'roll_deg': self.rpy_deg[0],
            'pitch_deg': self.rpy_deg[1],
            'yaw_deg': self.rpy_deg[2],
            'rpm_1': self.rpm[0],
            'rpm_2': self.rpm[1],
            'rpm_3': self.rpm[2],
            'rpm_4': self.rpm[3],
        })

        self.publish_status()

        if self.completed or self.failed:
            return

        if (
            now-(self.stage_start or now)
            > self.stage_timeout
        ):
            self.fail(
                f'stage_timeout:{stage["name"]}'
            )
            return

        stable = (
            error <= self.position_tolerance
            and speed <= self.hold_speed
        )

        if stable:
            if self.hold_start is None:
                self.hold_start = now
            if now-self.hold_start >= stage['hold_s']:
                self.complete_stage(now, error)
        else:
            self.hold_start = None

    def publish_waypoint_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        for index, stage in enumerate(self.stages):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = 'cq01_course_waypoints'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ) = stage['goal']
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.38
            marker.scale.y = 0.38
            marker.scale.z = 0.38

            if index < self.stage_index:
                marker.color.r = 0.20
                marker.color.g = 1.00
                marker.color.b = 0.20
            elif index == self.stage_index:
                marker.color.r = 1.00
                marker.color.g = 0.20
                marker.color.b = 0.80
            else:
                marker.color.r = 0.30
                marker.color.g = 0.65
                marker.color.b = 1.00
            marker.color.a = 1.0
            array.markers.append(marker)

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = now
        line.ns = 'cq01_course_waypoints'
        line.id = 100
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r = 0.25
        line.color.g = 0.85
        line.color.b = 1.00
        line.color.a = 0.75

        for stage in self.stages:
            point = Point()
            (
                point.x,
                point.y,
                point.z,
            ) = stage['goal']
            line.points.append(point)

        array.markers.append(line)
        self.waypoint_pub.publish(array)

    def publish_status(self) -> None:
        stage = self.current_stage()
        payload = {
            'state': (
                'completed'
                if self.completed
                else (
                    'failed'
                    if self.failed
                    else 'running'
                )
            ),
            'stage_index': self.stage_index,
            'stage_name': stage['name'],
            'goal': list(stage['goal']),
            'position': (
                list(self.position)
                if self.position else None
            ),
            'position_error_m': self.current_error(),
            'speed_mps': self.current_speed(),
            'maximum_tilt_deg': self.maximum_tilt_seen,
            'maximum_rpm': self.maximum_rpm_seen,
            'output_dir': (
                str(self.output_dir)
                if self.output_dir else None
            ),
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(message)

    def save_result(self) -> None:
        if self.saved or self.output_dir is None:
            return

        self.saved = True

        telemetry_path = (
            self.output_dir/'telemetry.csv'
        )
        if self.samples:
            with telemetry_path.open(
                'w',
                encoding='utf-8',
                newline='',
            ) as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=list(self.samples[0].keys()),
                )
                writer.writeheader()
                writer.writerows(self.samples)

        completed_names = {
            item['name']
            for item in self.stage_results
            if item.get('completed')
        }

        def completed(name: str) -> bool:
            return name in completed_names

        final_error = self.current_error()
        saturation_fraction = (
            self.rpm_saturated/max(1, self.rpm_samples)
        )

        criteria = {
            'exact_hover_0_0_1_5':
                completed('hover_0_0_1_5'),
            'exact_single_target_2_1_1_5':
                completed('single_target_2_1_1_5'),
            'four_target_sequence':
                all(
                    completed(f'multi_target_{index}')
                    for index in range(1, 5)
                ),
            'final_return_hover':
                completed('final_return_hover'),
            'final_error_at_most_0_3m':
                final_error is not None
                and final_error <= 0.30,
            'tilt_pass':
                self.maximum_tilt_seen
                <= self.maximum_tilt_limit,
            'rpm_no_saturation':
                saturation_fraction <= 0.001,
            'no_stage_timeout':
                not self.failed,
        }

        summary = {
            'experiment':
                'teacher_exact_minimum_acceptance_coordinates',
            'pass': all(criteria.values()),
            'failed_criteria': [
                key
                for key, value in criteria.items()
                if value is not True
            ],
            'criteria': criteria,
            'required_coordinates': {
                'hover': [0.0, 0.0, 1.5],
                'single_target': [2.0, 1.0, 1.5],
            },
            'multi_target_waypoints': [
                [3.0, 1.0, 1.5],
                [3.0, 3.0, 1.5],
                [1.0, 3.0, 1.5],
                [1.0, 1.0, 1.5],
            ],
            'stage_results': self.stage_results,
            'final_error_m': final_error,
            'maximum_tilt_deg': self.maximum_tilt_seen,
            'maximum_rpm': self.maximum_rpm_seen,
            'rpm_saturation_fraction':
                saturation_fraction,
            'sample_count': len(self.samples),
            'failure_reason': self.failure_reason,
        }

        (self.output_dir/'summary.json').write_text(
            json.dumps(
                summary,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

        self.publish_status()
        self.get_logger().warning(
            'COURSE ACCEPTANCE RESULT '
            f'pass={summary["pass"]}; '
            f'failed={summary["failed_criteria"]}; '
            f'output={self.output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CourseAcceptanceRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.started and not node.saved:
            node.failure_reason = 'interrupted'
            node.save_result()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
