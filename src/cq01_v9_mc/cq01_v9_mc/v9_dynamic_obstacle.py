#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


def path_branch(
    first: List[Point3],
    second: List[Point3],
) -> float:
    if not first or not second:
        return 0.0
    return sum(
        min(math.dist(point, other) for other in second)
        for point in first
    ) / len(first)


class V9DynamicObstacle(Node):
    """Persistent high-altitude obstacle with proximity-based relocation.

    The visual obstacle moves continuously from P13 until P15. Its small
    dynamic voxel cloud is published separately from the 1.59-million-point
    static map, so the A* planner updates only a few hundred voxels.

    Behaviour:
    - initial patrol begins at P13->P14;
    - first A* replan waits until drone-obstacle distance <= 28 m;
    - at <= 20 m and then <= 16 m, the obstacle relocates to the next
      high-altitude patrol zone and requests another replan;
    - it never disappears on a fixed 12-second timer;
    - it is removed only after P15 is reached.
    """

    def __init__(self) -> None:
        super().__init__('cq01_v9_dynamic_obstacle')

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.dynamic_cloud_pub = self.create_publisher(
            PointCloud2,
            '/cq01_v9/dynamic/voxel_cloud',
            transient,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/cq01_v9/dynamic/markers',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/cq01_v9/dynamic/status',
            transient,
        )
        self.replan_pub = self.create_publisher(
            PoseStamped,
            '/cq01_v9/dynamic_replan_goal_raw',
            10,
        )

        self.create_subscription(
            String,
            '/cq01_v9/route/status',
            self.route_callback,
            transient,
        )
        self.create_subscription(
            PoseStamped,
            '/drone/goal_raw',
            self.goal_callback,
            10,
        )
        self.create_subscription(
            NavPath,
            '/drone/planned_path_3d',
            self.path_callback,
            transient,
        )
        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )

        self.route: Dict = {}
        self.current_goal: Optional[PoseStamped] = None
        self.current_path: List[Point3] = []
        self.old_path: List[Point3] = []
        self.position: Optional[Point3] = None

        self.active = False
        self.completed = False
        self.active_start: Optional[float] = None
        self.stage_end_time: Optional[float] = None

        # Three high-altitude patrol zones follow P13->P14->P15.
        # Each zone contains a continuously oscillating obstacle.
        self.anchors: List[Point3] = [
            (378.0, -2.0, 210.0),
            (392.0, 20.0, 216.0),
            (404.0, 36.0, 220.0),
        ]
        self.axes: List[Point3] = [
            (0.0, 1.0, 0.0),
            (-0.70710678, 0.70710678, 0.0),
            (-0.70710678, 0.70710678, 0.0),
        ]
        self.amplitudes = [10.0, 8.0, 6.0]
        self.zone_index = 0
        self.zone_start_time: Optional[float] = None
        self.center: Point3 = self.anchors[0]

        self.radius = 2.2
        self.height = 5.0
        self.planning_radius = 4.0
        self.planning_height = 7.0
        self.maximum_patrol_speed_mps = 2.0

        self.first_replan_distance_m = 28.0
        self.relocation_distances_m = [20.0, 16.0]
        self.relocation_cooldown_s = 3.0
        self.last_relocation_time = -1.0e9
        self.relocation_count = 0

        self.cloud_publish_rate_hz = 2.0
        self.marker_publish_rate_hz = 10.0
        self.dynamic_cloud_publish_count = 0
        self.marker_publish_count = 0

        self.replan_request_count = 0
        self.replan_response_count = 0
        self.awaiting_path = False
        self.replan_request_time: Optional[float] = None
        self.replan_latency_s: Optional[float] = None
        self.maximum_path_branch_m = 0.0
        self.new_path_received = False

        self.minimum_separation_m: Optional[float] = None
        self.maximum_obstacle_speed_mps = 0.0
        self.last_center: Optional[Point3] = None
        self.last_center_time: Optional[float] = None
        self.motion_distance_m = 0.0
        self.first_proximity_trigger_distance_m: Optional[float] = None

        self.output_dir: Optional[Path] = None
        self.result_saved = False

        self.motion_timer = self.create_timer(
            1.0/self.marker_publish_rate_hz,
            self.motion_callback,
        )
        self.cloud_timer = self.create_timer(
            1.0/self.cloud_publish_rate_hz,
            self.publish_dynamic_cloud,
        )
        self.status_timer = self.create_timer(
            0.50,
            self.publish_status,
        )

        self.publish_empty_cloud()
        self.get_logger().warning(
            'V9-MC.5 persistent dynamic obstacle ready: '
            'continuous patrol P13-P15; proximity replan at 28 m; '
            'relocation at 20 m and 16 m.'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def route_callback(self, msg: String) -> None:
        try:
            self.route = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        current_index = int(
            self.route.get('current_index', -1)
        )
        reached_index = int(
            self.route.get('reached_index', -1)
        )
        state = str(self.route.get('state', ''))

        if (
            not self.active
            and not self.completed
            and current_index >= 14
            and reached_index >= 13
        ):
            self.active = True
            self.active_start = self.now_s()
            self.zone_start_time = self.active_start
            self.last_relocation_time = self.active_start
            self.state = 'persistent_patrol_active'
            self.get_logger().warning(
                'V9-MC.5 DYNAMIC PATROL ACTIVE: '
                'obstacle will remain until P15.'
            )

        if (
            self.active
            and (
                reached_index >= 15
                or state == 'mission_completed'
            )
        ):
            self.complete_stage()

    def goal_callback(self, msg: PoseStamped) -> None:
        self.current_goal = msg

    def path_callback(self, msg: NavPath) -> None:
        path = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in msg.poses
        ]
        if len(path) < 2:
            return

        if self.awaiting_path:
            self.awaiting_path = False
            self.replan_response_count += 1
            self.new_path_received = True
            if self.replan_request_time is not None:
                self.replan_latency_s = (
                    self.now_s()
                    - self.replan_request_time
                )
            branch = path_branch(
                self.old_path,
                path,
            )
            self.maximum_path_branch_m = max(
                self.maximum_path_branch_m,
                branch,
            )
            self.get_logger().warning(
                'V9-MC.5 DYNAMIC REPLAN READY: '
                f'response={self.replan_response_count}, '
                f'branch={branch:.3f} m, '
                f'latency={self.replan_latency_s}.'
            )

        self.current_path = path

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        if self.active:
            separation = (
                math.dist(
                    self.position,
                    self.center,
                )
                - self.radius
            )
            if self.minimum_separation_m is None:
                self.minimum_separation_m = separation
            else:
                self.minimum_separation_m = min(
                    self.minimum_separation_m,
                    separation,
                )

    def current_distance(self) -> Optional[float]:
        if self.position is None:
            return None
        return math.dist(
            self.position,
            self.center,
        )

    def update_center(self, now: float) -> None:
        if not self.active:
            return

        anchor = self.anchors[self.zone_index]
        axis = self.axes[self.zone_index]
        amplitude = self.amplitudes[self.zone_index]
        elapsed = now - (
            self.zone_start_time
            if self.zone_start_time is not None
            else now
        )

        omega = (
            self.maximum_patrol_speed_mps
            / max(0.5, amplitude)
        )
        offset = amplitude*math.sin(omega*elapsed)
        center = (
            anchor[0] + axis[0]*offset,
            anchor[1] + axis[1]*offset,
            anchor[2] + axis[2]*offset,
        )

        if (
            self.last_center is not None
            and self.last_center_time is not None
        ):
            dt = max(
                1.0e-3,
                now - self.last_center_time,
            )
            step = math.dist(
                center,
                self.last_center,
            )
            self.motion_distance_m += step
            self.maximum_obstacle_speed_mps = max(
                self.maximum_obstacle_speed_mps,
                step/dt,
            )

        self.last_center = center
        self.last_center_time = now
        self.center = center

    def request_replan(self, reason: str) -> None:
        if self.current_goal is None:
            self.get_logger().warning(
                'Dynamic replan deferred: current route goal unavailable.'
            )
            return

        self.old_path = list(self.current_path)
        goal = PoseStamped()
        goal.header = self.current_goal.header
        goal.pose = self.current_goal.pose
        goal.header.stamp = self.get_clock().now().to_msg()
        self.replan_pub.publish(goal)

        self.replan_request_count += 1
        self.awaiting_path = True
        self.replan_request_time = self.now_s()
        self.get_logger().warning(
            'V9-MC.5 DYNAMIC A* REPLAN REQUESTED: '
            f'count={self.replan_request_count}, reason={reason}, '
            f'obstacle={self.center}.'
        )

    def relocate(self, now: float, distance: float) -> None:
        if self.zone_index >= len(self.anchors)-1:
            return

        self.zone_index += 1
        self.relocation_count += 1
        self.zone_start_time = now
        self.last_relocation_time = now
        self.center = self.anchors[self.zone_index]
        self.last_center = self.center
        self.last_center_time = now

        # Publish the new small occupancy immediately, then replan.
        self.publish_dynamic_cloud(force=True)
        self.request_replan(
            f'proximity_relocation_{self.relocation_count}'
        )
        self.get_logger().warning(
            'V9-MC.5 DYNAMIC OBSTACLE RELOCATED: '
            f'count={self.relocation_count}, '
            f'trigger_distance={distance:.3f} m, '
            f'zone={self.zone_index}, center={self.center}.'
        )

    def motion_callback(self) -> None:
        now = self.now_s()
        self.update_center(now)

        if self.active:
            distance = self.current_distance()

            if (
                distance is not None
                and self.replan_request_count == 0
                and distance
                <= self.first_replan_distance_m
            ):
                self.first_proximity_trigger_distance_m = distance
                self.request_replan(
                    'first_proximity_trigger'
                )

            if (
                distance is not None
                and self.replan_request_count >= 1
                and self.zone_index
                < len(self.anchors)-1
                and now-self.last_relocation_time
                >= self.relocation_cooldown_s
            ):
                threshold = self.relocation_distances_m[
                    self.zone_index
                ]
                if distance <= threshold:
                    self.relocate(
                        now,
                        distance,
                    )

        self.publish_markers()

    def dynamic_points(self) -> List[Point3]:
        cx, cy, cz = self.center
        points: List[Point3] = []
        step = 1.0
        z = cz-self.planning_height/2.0

        while z <= cz+self.planning_height/2.0+1.0e-6:
            radius = 0.0
            while radius <= self.planning_radius+1.0e-6:
                count = max(
                    8,
                    int(
                        math.ceil(
                            2.0*math.pi*max(radius, step)
                            / step
                        )
                    ),
                )
                for index in range(count):
                    angle = 2.0*math.pi*index/count
                    points.append((
                        cx + radius*math.cos(angle),
                        cy + radius*math.sin(angle),
                        z,
                    ))
                radius += step
            z += step

        return points

    def publish_empty_cloud(self) -> None:
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(
            header,
            [],
        )
        self.dynamic_cloud_pub.publish(cloud)

    def publish_dynamic_cloud(
        self,
        force: bool = False,
    ) -> None:
        if not self.active:
            if force:
                self.publish_empty_cloud()
            return

        from std_msgs.msg import Header
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(
            header,
            self.dynamic_points(),
        )
        self.dynamic_cloud_pub.publish(cloud)
        self.dynamic_cloud_publish_count += 1

    def publish_markers(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        if self.active:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = (
                self.get_clock().now().to_msg()
            )
            marker.ns = 'v9_dynamic'
            marker.id = 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ) = self.center
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0*self.radius
            marker.scale.y = 2.0*self.radius
            marker.scale.z = self.height
            marker.color.r = 1.0
            marker.color.g = 0.20
            marker.color.b = 0.05
            marker.color.a = 0.98
            array.markers.append(marker)

            # Patrol-line geometry, no text.
            line = Marker()
            line.header = marker.header
            line.ns = 'v9_dynamic'
            line.id = 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.45
            line.color.r = 1.0
            line.color.g = 0.65
            line.color.b = 0.05
            line.color.a = 0.85

            anchor = self.anchors[self.zone_index]
            axis = self.axes[self.zone_index]
            amplitude = self.amplitudes[self.zone_index]

            from geometry_msgs.msg import Point
            for sign in (-1.0, 1.0):
                p = Point()
                p.x = anchor[0] + sign*axis[0]*amplitude
                p.y = anchor[1] + sign*axis[1]*amplitude
                p.z = anchor[2] + sign*axis[2]*amplitude
                line.points.append(p)
            array.markers.append(line)

        self.marker_pub.publish(array)
        self.marker_publish_count += 1

    def complete_stage(self) -> None:
        if not self.active or self.completed:
            return

        self.active = False
        self.completed = True
        self.stage_end_time = self.now_s()
        self.publish_empty_cloud()
        self.publish_markers()
        self.save_result()

        self.get_logger().warning(
            'V9-MC.5 DYNAMIC PATROL COMPLETE AT P15: '
            f'duration={self.stage_duration_s()}, '
            f'relocations={self.relocation_count}, '
            f'replans={self.replan_request_count}.'
        )

    def stage_duration_s(self) -> Optional[float]:
        if self.active_start is None:
            return None
        end = (
            self.stage_end_time
            if self.stage_end_time is not None
            else self.now_s()
        )
        return end-self.active_start

    def save_result(self) -> None:
        if self.result_saved:
            return

        criteria = {
            'continuous_patrol_started':
                self.active_start is not None,
            'patrol_persisted_until_P15':
                self.completed
                and int(
                    self.route.get('reached_index', -1)
                ) >= 15,
            'obstacle_actually_moved':
                self.motion_distance_m >= 10.0,
            'proximity_triggered_replan':
                self.first_proximity_trigger_distance_m
                is not None
                and self.first_proximity_trigger_distance_m
                <= self.first_replan_distance_m,
            'proximity_relocation_occurred':
                self.relocation_count >= 1,
            'new_path_received':
                self.new_path_received,
            'safe_separation':
                self.minimum_separation_m is not None
                and self.minimum_separation_m >= 2.5,
            'small_dynamic_cloud_only':
                self.dynamic_cloud_publish_count >= 1,
        }

        output_dir = Path(
            os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v9_mc/dynamic'
            )
        ) / datetime.now().strftime(
            'dynamic_persistent_%Y%m%d_%H%M%S'
        )
        output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )
        self.output_dir = output_dir

        summary = {
            'experiment':
                'cq01_v9_persistent_proximity_dynamic_obstacle',
            'pass':
                all(criteria.values()),
            'failed_criteria': [
                key
                for key, value in criteria.items()
                if value is not True
            ],
            'criteria':
                criteria,
            'stage_duration_s':
                self.stage_duration_s(),
            'patrol_zone_count':
                len(self.anchors),
            'final_zone_index':
                self.zone_index,
            'relocation_count':
                self.relocation_count,
            'replan_request_count':
                self.replan_request_count,
            'replan_response_count':
                self.replan_response_count,
            'first_proximity_trigger_distance_m':
                self.first_proximity_trigger_distance_m,
            'relocation_thresholds_m':
                self.relocation_distances_m,
            'minimum_separation_m':
                self.minimum_separation_m,
            'maximum_new_path_branch_m':
                self.maximum_path_branch_m,
            'latest_replan_latency_s':
                self.replan_latency_s,
            'motion_distance_m':
                self.motion_distance_m,
            'maximum_obstacle_speed_mps':
                self.maximum_obstacle_speed_mps,
            'dynamic_cloud_publish_count':
                self.dynamic_cloud_publish_count,
            'static_cloud_republished_by_dynamic_node':
                False,
            'dynamic_cloud_topic':
                '/cq01_v9/dynamic/voxel_cloud',
        }

        (output_dir/'summary.json').write_text(
            json.dumps(
                summary,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )

        self.result_saved = True
        self.publish_status()
        self.get_logger().warning(
            'V9-MC.5 DYNAMIC RESULT '
            f'pass={summary["pass"]}; '
            f'failed={summary["failed_criteria"]}; '
            f'output={output_dir}'
        )

    def publish_status(self) -> None:
        payload = {
            'state': (
                'dynamic_obstacle_completed'
                if self.completed
                else (
                    'persistent_patrol_active'
                    if self.active
                    else 'waiting_for_P13'
                )
            ),
            'active':
                self.active,
            'obstacle_center':
                list(self.center),
            'zone_index':
                self.zone_index,
            'distance_to_drone_m':
                self.current_distance(),
            'first_replan_distance_m':
                self.first_replan_distance_m,
            'relocation_count':
                self.relocation_count,
            'replan_requested':
                self.replan_request_count >= 1,
            'replan_request_count':
                self.replan_request_count,
            'new_path_received':
                self.new_path_received,
            'new_path_branch_m':
                self.maximum_path_branch_m,
            'minimum_separation_m':
                self.minimum_separation_m,
            'motion_distance_m':
                self.motion_distance_m,
            'dynamic_cloud_publish_count':
                self.dynamic_cloud_publish_count,
            'dynamic_cloud_period_s':
                1.0/self.cloud_publish_rate_hz,
            'persistent_until_waypoint':
                15,
            'output_dir':
                str(self.output_dir)
                if self.output_dir else None,
            'pass_result':
                None
                if not self.result_saved
                else (
                    self.completed
                    and self.relocation_count >= 1
                    and self.new_path_received
                    and self.minimum_separation_m is not None
                    and self.minimum_separation_m >= 2.5
                ),
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V9DynamicObstacle()
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
