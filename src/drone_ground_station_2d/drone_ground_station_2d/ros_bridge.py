from __future__ import annotations

import copy
import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from .models import (
    DynamicObstacle,
    GroundSnapshot,
    OccupancyMap,
    SystemState,
    VehicleState,
    quaternion_to_euler_deg,
    yaw_deg_to_quaternion,
)


@dataclass(frozen=True)
class GoalCommand:
    vehicle_key: str
    route: str
    x: float
    y: float
    z: float
    yaw_deg: float


class GroundStationBridge(Node):
    def __init__(self) -> None:
        super().__init__('drone_ground_station_2d')
        self.declare_parameter('path_history_limit', 3000)
        self.declare_parameter('goal_publish_repetitions', 5)
        self.declare_parameter('goal_publish_interval_s', 0.08)
        self.declare_parameter('maximum_occupied_points', 6000)

        self.path_history_limit = int(self.get_parameter('path_history_limit').value)
        self.goal_publish_repetitions = int(self.get_parameter('goal_publish_repetitions').value)
        self.goal_publish_interval_s = float(self.get_parameter('goal_publish_interval_s').value)
        self.maximum_occupied_points = int(self.get_parameter('maximum_occupied_points').value)

        self.lock = threading.RLock()
        self.command_queue: queue.Queue[GoalCommand] = queue.Queue()
        self.pending_goals: List[tuple[GoalCommand, int, float]] = []
        self.event_queue: queue.Queue[str] = queue.Queue()

        self.vehicles: Dict[str, VehicleState] = {
            'single': VehicleState('single', 'Single UAV'),
            'uav1': VehicleState('uav1', 'UAV 1'),
            'uav2': VehicleState('uav2', 'UAV 2'),
        }
        self.planned_path = []
        self.planned_path_3d = []
        self.trajectory_reference = []
        self.raw_goal = None
        self.nominal_goal = None
        self.occupancy = OccupancyMap()
        self.dynamic_obstacle = DynamicObstacle()
        self.system = SystemState()

        self.goal_publishers = {
            ('single', 'direct'): self.create_publisher(PoseStamped, '/drone/goal', 10),
            ('single', 'planner'): self.create_publisher(PoseStamped, '/drone/goal_raw', 10),
            ('single', 'dynamic'): self.create_publisher(PoseStamped, '/drone/goal_nominal', 10),
            ('uav1', 'direct'): self.create_publisher(PoseStamped, '/uav1/goal', 10),
            ('uav2', 'direct'): self.create_publisher(PoseStamped, '/uav2/goal', 10),
        }
        self.subscriptions_store = []
        for key, prefix in [('single', '/drone'), ('uav1', '/uav1'), ('uav2', '/uav2')]:
            self.subscriptions_store += [
                self.create_subscription(Odometry, f'{prefix}/odom', lambda m, k=key: self._odom(k, m), 50),
                self.create_subscription(PoseStamped, f'{prefix}/goal', lambda m, k=key: self._goal(k, m), 20),
                self.create_subscription(Float32MultiArray, f'{prefix}/motor_rpm', lambda m, k=key: self._rpm(k, m), 50),
                self.create_subscription(Vector3Stamped, f'{prefix}/position_error', lambda m, k=key: self._error(k, m), 20),
            ]

        self.subscriptions_store += [
            self.create_subscription(Path, '/drone/planned_path', lambda m: self._path('planned', m), 10),
            self.create_subscription(Path, '/drone/planned_path_3d', lambda m: self._path('planned3d', m), 10),
            self.create_subscription(Path, '/drone/trajectory_reference', lambda m: self._path('trajectory', m), 10),
            self.create_subscription(PoseStamped, '/drone/goal_raw', lambda m: self._aux_goal('raw', m), 10),
            self.create_subscription(PoseStamped, '/drone/goal_nominal', lambda m: self._aux_goal('nominal', m), 10),
            self.create_subscription(PoseStamped, '/drone/dynamic_obstacle/pose', self._dynamic_pose, 20),
            self.create_subscription(String, '/drone/dynamic_obstacle/state', lambda m: self._json_status('dynamic', m), 20),
            self.create_subscription(String, '/drone/planner_3d/status', lambda m: self._json_status('planner', m), 20),
            self.create_subscription(String, '/drone/trajectory/status', lambda m: self._json_status('trajectory', m), 20),
            self.create_subscription(String, '/drone/replanning/status', lambda m: self._json_status('replanning', m), 20),
            self.create_subscription(String, '/multi_uav/coordinator_state', self._coordinator, 20),
            self.create_subscription(Bool, '/multi_uav/safety_active', self._safety, 20),
            self.create_subscription(Float32, '/multi_uav/current_distance', self._current_distance, 20),
            self.create_subscription(Float32, '/multi_uav/predicted_minimum_distance', self._predicted_distance, 20),
        ]

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.VOLATILE
        self.subscriptions_store += [
            self.create_subscription(OccupancyGrid, '/drone/obstacle_grid', self._occupancy, map_qos),
            self.create_subscription(OccupancyGrid, '/drone/obstacle_grid_safety', self._occupancy, map_qos),
        ]

        self.timer = self.create_timer(0.02, self._drain_commands)
        self.get_logger().info('Advanced 2D ground-station bridge started')

    def _odom(self, key: str, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        now = time.monotonic()
        with self.lock:
            state = self.vehicles[key]
            state.position = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(msg.pose.pose.position.z)]
            state.velocity = [float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y), float(msg.twist.twist.linear.z)]
            state.angular_velocity = [float(msg.twist.twist.angular.x), float(msg.twist.twist.angular.y), float(msg.twist.twist.angular.z)]
            state.attitude_deg = list(quaternion_to_euler_deg(q.x, q.y, q.z, q.w))
            state.last_odom_monotonic = now
            point = tuple(state.position)
            if not state.actual_path or sum((point[i] - state.actual_path[-1][i]) ** 2 for i in range(3)) >= 0.0004:
                state.actual_path.append(point)
                overflow = len(state.actual_path) - self.path_history_limit
                if overflow > 0:
                    del state.actual_path[:overflow]

    def _goal(self, key: str, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        _, _, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        with self.lock:
            state = self.vehicles[key]
            state.goal = [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)]
            state.goal_yaw_deg = yaw
            state.has_goal = True

    def _rpm(self, key: str, msg: Float32MultiArray) -> None:
        values = [float(v) for v in msg.data[:4]]
        values += [0.0] * (4 - len(values))
        with self.lock:
            self.vehicles[key].rpm = values
            self.vehicles[key].last_rpm_monotonic = time.monotonic()

    def _error(self, key: str, msg: Vector3Stamped) -> None:
        with self.lock:
            self.vehicles[key].position_error = [float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]

    @staticmethod
    def _points(msg: Path):
        return [(float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)) for p in msg.poses]

    def _path(self, which: str, msg: Path) -> None:
        points = self._points(msg)
        with self.lock:
            if which == 'planned': self.planned_path = points
            elif which == 'planned3d': self.planned_path_3d = points
            else: self.trajectory_reference = points

    def _aux_goal(self, which: str, msg: PoseStamped) -> None:
        point = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        with self.lock:
            if which == 'raw': self.raw_goal = point
            else: self.nominal_goal = point

    def _dynamic_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            self.dynamic_obstacle.position = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
            self.dynamic_obstacle.last_update_monotonic = time.monotonic()

    def _json_status(self, key: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict): data = {'value': data}
        except Exception:
            data = {'raw': msg.data}
        now = time.monotonic()
        with self.lock:
            old = None
            if key == 'planner':
                old = self.system.planner_status.get('state')
                self.system.planner_status = data
            elif key == 'trajectory':
                old = self.system.trajectory_status.get('state')
                self.system.trajectory_status = data
            elif key == 'replanning':
                old = self.system.replanning_status.get('state')
                self.system.replanning_status = data
            else:
                old = self.dynamic_obstacle.phase
                self.system.dynamic_status = data
                self.dynamic_obstacle.phase = str(data.get('phase', 'N/A'))
                self.dynamic_obstacle.radius_m = float(data.get('radius_m', self.dynamic_obstacle.radius_m))
                self.dynamic_obstacle.height_m = float(data.get('height_m', self.dynamic_obstacle.height_m))
                predictions = data.get('predicted_positions', [])
                parsed = []
                for item in predictions:
                    p = item.get('position', item) if isinstance(item, dict) else item
                    if isinstance(p, (list, tuple)) and len(p) >= 3:
                        parsed.append((float(p[0]), float(p[1]), float(p[2])))
                self.dynamic_obstacle.predictions = tuple(parsed)
                self.dynamic_obstacle.last_update_monotonic = now
            self.system.last_status_monotonic[key] = now
            new = data.get('state', data.get('phase'))
        if new is not None and new != old:
            self.event_queue.put(f'{key.upper()}: {new}')

    def _coordinator(self, msg: String) -> None:
        with self.lock:
            old = self.system.coordinator_state
            self.system.coordinator_state = str(msg.data)
            self.system.last_status_monotonic['coordinator'] = time.monotonic()
        if old != msg.data:
            self.event_queue.put(f'MULTI-UAV: {msg.data}')

    def _safety(self, msg: Bool) -> None:
        with self.lock:
            old = self.system.safety_active
            self.system.safety_active = bool(msg.data)
            self.system.last_status_monotonic['safety'] = time.monotonic()
        if old != bool(msg.data):
            self.event_queue.put('SAFETY ACTIVE' if msg.data else 'SAFETY CLEARED')

    def _current_distance(self, msg: Float32) -> None:
        with self.lock: self.system.current_inter_uav_distance_m = float(msg.data)

    def _predicted_distance(self, msg: Float32) -> None:
        with self.lock: self.system.predicted_inter_uav_distance_m = float(msg.data)

    def _occupancy(self, msg: OccupancyGrid) -> None:
        width, height = int(msg.info.width), int(msg.info.height)
        res = float(msg.info.resolution)
        ox, oy = float(msg.info.origin.position.x), float(msg.info.origin.position.y)
        indices = [i for i, v in enumerate(msg.data) if int(v) >= 50]
        if len(indices) > self.maximum_occupied_points:
            step = max(1, len(indices) // self.maximum_occupied_points)
            indices = indices[::step]
        points = tuple((ox + (i % width + 0.5) * res, oy + (i // width + 0.5) * res) for i in indices)
        with self.lock:
            self.occupancy = OccupancyMap(
                version=self.occupancy.version + 1,
                frame_id=msg.header.frame_id or 'map', resolution=res,
                width=width, height=height, origin_x=ox, origin_y=oy,
                occupied_points=points,
            )

    def enqueue_goal(self, command: GoalCommand) -> None:
        self.command_queue.put(command)

    def clear_tracks(self) -> None:
        with self.lock:
            for state in self.vehicles.values(): state.actual_path.clear()
        self.event_queue.put('TRACKS CLEARED')

    def _drain_commands(self) -> None:
        while True:
            try: command = self.command_queue.get_nowait()
            except queue.Empty: break
            route = command.route if command.vehicle_key == 'single' else 'direct'
            normalized = GoalCommand(command.vehicle_key, route, command.x, command.y, command.z, command.yaw_deg)
            self.pending_goals.append((normalized, self.goal_publish_repetitions, 0.0))
            self.event_queue.put(
                f'GOAL {command.vehicle_key}/{route}: ({command.x:.2f}, {command.y:.2f}, {command.z:.2f})'
            )
        now = time.monotonic()
        keep = []
        for command, remaining, last_sent in self.pending_goals:
            if remaining <= 0: continue
            if now - last_sent < self.goal_publish_interval_s:
                keep.append((command, remaining, last_sent)); continue
            self._publish_goal(command)
            remaining -= 1
            if remaining > 0: keep.append((command, remaining, now))
        self.pending_goals = keep

    def _publish_goal(self, command: GoalCommand) -> None:
        publisher = self.goal_publishers.get((command.vehicle_key, command.route))
        if publisher is None: publisher = self.goal_publishers[(command.vehicle_key, 'direct')]
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = command.x
        message.pose.position.y = command.y
        message.pose.position.z = max(0.0, command.z)
        qx, qy, qz, qw = yaw_deg_to_quaternion(command.yaw_deg)
        message.pose.orientation.x, message.pose.orientation.y = qx, qy
        message.pose.orientation.z, message.pose.orientation.w = qz, qw
        publisher.publish(message)

    def snapshot(self) -> GroundSnapshot:
        with self.lock:
            vehicles = copy.deepcopy(self.vehicles)
            system = copy.deepcopy(self.system)
            occupancy = self.occupancy
            dynamic = copy.deepcopy(self.dynamic_obstacle)
            return GroundSnapshot(
                monotonic_time=time.monotonic(), vehicles=vehicles,
                planned_path=tuple(self.planned_path),
                planned_path_3d=tuple(self.planned_path_3d),
                trajectory_reference=tuple(self.trajectory_reference),
                raw_goal=self.raw_goal, nominal_goal=self.nominal_goal,
                occupancy=occupancy, dynamic_obstacle=dynamic, system=system,
            )

    def drain_events(self) -> List[str]:
        items = []
        while True:
            try: items.append(self.event_queue.get_nowait())
            except queue.Empty: return items
