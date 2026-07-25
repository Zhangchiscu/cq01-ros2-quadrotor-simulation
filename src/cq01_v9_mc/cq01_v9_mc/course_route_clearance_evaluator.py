#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]


class SceneGeometry:
    """Exact terrain and oriented-box geometry used by scene_publisher."""

    def __init__(self, scene_file: str) -> None:
        with open(scene_file, 'r', encoding='utf-8') as file:
            self.scene: Dict = yaml.safe_load(file)['scene']

        self.boxes: List[Dict] = []
        for spec in self.scene.get('collision_boxes', []):
            center, size, yaw = self.resolve_box(spec)
            self.boxes.append({
                'id': str(spec.get('id', 'unknown')),
                'center': center,
                'size': size,
                'yaw': yaw,
            })

    @staticmethod
    def interp_profile(
        x: float,
        points: Sequence[Sequence[float]],
    ) -> float:
        if x <= float(points[0][0]):
            return float(points[0][1])

        for p0, p1 in zip(points[:-1], points[1:]):
            x0, z0 = float(p0[0]), float(p0[1])
            x1, z1 = float(p1[0]), float(p1[1])
            if x <= x1:
                ratio = (
                    (x-x0)/max(1.0e-9, x1-x0)
                )
                return z0+ratio*(z1-z0)

        return float(points[-1][1])

    @staticmethod
    def ellipse_weight(
        x: float,
        y: float,
        spec: Dict,
    ) -> float:
        cx, cy = (float(v) for v in spec['center'])
        rx, ry = (float(v) for v in spec['radius'])
        yaw = math.radians(
            float(spec.get('yaw_deg', 0.0))
        )
        dx, dy = x-cx, y-cy
        lx = math.cos(yaw)*dx+math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx+math.cos(yaw)*dy
        radius_squared = (
            (lx/max(rx, 1.0e-6))**2
            +(ly/max(ry, 1.0e-6))**2
        )
        if radius_squared >= 1.0:
            return 0.0
        value = 1.0-math.sqrt(
            max(0.0, radius_squared)
        )
        return value*value*(3.0-2.0*value)

    @staticmethod
    def segment_distance(
        x: float,
        y: float,
        p0,
        p1,
    ) -> Tuple[float, float]:
        ax, ay = float(p0[0]), float(p0[1])
        bx, by = float(p1[0]), float(p1[1])
        vx, vy = bx-ax, by-ay
        wx, wy = x-ax, y-ay
        denominator = max(
            1.0e-9,
            vx*vx+vy*vy,
        )
        ratio = max(
            0.0,
            min(
                1.0,
                (wx*vx+wy*vy)/denominator,
            ),
        )
        px, py = ax+ratio*vx, ay+ratio*vy
        return math.hypot(x-px, y-py), ratio

    @staticmethod
    def smoothstep(value: float) -> float:
        value = max(0.0, min(1.0, float(value)))
        return value*value*(3.0-2.0*value)

    @staticmethod
    def superellipse_radius(
        x: float,
        y: float,
        spec: Dict,
    ) -> float:
        cx, cy = (float(v) for v in spec['center'])
        rx, ry = (float(v) for v in spec['radius'])
        yaw = math.radians(
            float(spec.get('yaw_deg', 0.0))
        )
        power = max(
            1.0,
            float(spec.get('super_power', 4.0)),
        )
        dx, dy = x-cx, y-cy
        lx = math.cos(yaw)*dx+math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx+math.cos(yaw)*dy
        return (
            abs(lx/max(rx, 1.0e-6))**power
            +abs(ly/max(ry, 1.0e-6))**power
        )**(1.0/power)

    @classmethod
    def plateau_weight(
        cls,
        radius: float,
        spec: Dict,
    ) -> float:
        if radius >= 1.0:
            return 0.0
        top = max(
            0.0,
            min(
                0.90,
                float(spec.get('top_ratio', 0.60)),
            ),
        )
        if radius <= top:
            return 1.0
        value = (
            (radius-top)/max(1.0e-6, 1.0-top)
        )
        return (
            1.0-cls.smoothstep(value)
        )**float(spec.get('edge_power', 1.6))

    def canyons(self) -> List[Dict]:
        terrain = self.scene.get('terrain', {})
        items = list(terrain.get('canyons', []))
        if not items and terrain.get('canyon'):
            items = [terrain['canyon']]
        return items

    def terrain_height(
        self,
        x: float,
        y: float = 0.0,
    ) -> float:
        terrain = self.scene['terrain']
        model = str(terrain.get('model', ''))

        if model not in (
            'mountain_city_v3',
            'mountain_city_v31',
            'mountain_city_v33',
        ):
            segments = terrain['segments']
            for segment in segments:
                x0 = float(segment['x0'])
                x1 = float(segment['x1'])
                if x0 <= x <= x1:
                    z0 = float(segment['z0'])
                    z1 = float(segment['z1'])
                    if abs(x1-x0) < 1.0e-9:
                        return z1
                    return z0+(x-x0)/(x1-x0)*(z1-z0)
            if x < float(segments[0]['x0']):
                return float(segments[0]['z0'])
            return float(segments[-1]['z1'])

        z = self.interp_profile(
            x,
            terrain['base_profile'],
        )
        z += float(
            terrain.get('lateral_tilt', 0.0)
        )*y

        undulation = terrain.get('undulation', {})
        z += float(
            undulation.get('amp_x', 0.0)
        )*math.sin(
            2.0*math.pi*x/max(
                1.0e-6,
                float(
                    undulation.get(
                        'period_x',
                        50.0,
                    )
                ),
            )
        )
        z += float(
            undulation.get('amp_y', 0.0)
        )*math.sin(
            2.0*math.pi*y/max(
                1.0e-6,
                float(
                    undulation.get(
                        'period_y',
                        50.0,
                    )
                ),
            )
        )
        z += float(
            undulation.get('cross_amp', 0.0)
        )*math.sin(
            2.0*math.pi*(x+y)/max(
                1.0e-6,
                float(
                    undulation.get(
                        'cross_period',
                        75.0,
                    )
                ),
            )
        )

        if model == 'mountain_city_v33':
            for plateau in terrain.get(
                'urban_plateaus',
                [],
            ):
                radius = self.superellipse_radius(
                    x,
                    y,
                    plateau,
                )
                weight = self.plateau_weight(
                    radius,
                    plateau,
                )
                candidate = (
                    (1.0-weight)*z
                    +weight*float(plateau['target_z'])
                )
                z = max(z, candidate)

            for ridge in terrain.get('ridges', []):
                cx, cy = (
                    float(v)
                    for v in ridge['center']
                )
                sx, sy = (
                    float(v)
                    for v in ridge['sigma']
                )
                dx = (x-cx)/max(sx, 1.0e-6)
                dy = (y-cy)/max(sy, 1.0e-6)
                z += float(ridge['height'])*math.exp(
                    -0.5*(dx*dx+dy*dy)
                )

            for terrace in terrain.get(
                'terraces',
                [],
            ):
                weight = (
                    self.ellipse_weight(
                        x,
                        y,
                        terrace,
                    )
                    *float(
                        terrace.get('strength', 1.0)
                    )
                )
                z = (
                    (1.0-weight)*z
                    +weight*float(terrace['target_z'])
                )

        elif model == 'mountain_city_v31':
            for peak in terrain.get('peaks', []):
                cx, cy = (
                    float(v)
                    for v in peak['center']
                )
                rx, ry = (
                    float(v)
                    for v in peak['radius']
                )
                yaw = math.radians(
                    float(peak.get('yaw_deg', 0.0))
                )
                dx, dy = x-cx, y-cy
                lx = (
                    math.cos(yaw)*dx
                    +math.sin(yaw)*dy
                )
                ly = (
                    -math.sin(yaw)*dx
                    +math.cos(yaw)*dy
                )
                radius = math.sqrt(
                    (lx/max(rx, 1.0e-6))**2
                    +(ly/max(ry, 1.0e-6))**2
                )
                if radius < 1.0:
                    summit = max(
                        0.0,
                        min(
                            0.8,
                            float(
                                peak.get(
                                    'summit_radius',
                                    0.15,
                                )
                            ),
                        ),
                    )
                    if radius <= summit:
                        shape = 1.0
                    else:
                        value = max(
                            0.0,
                            (1.0-radius)
                            /max(1.0e-6, 1.0-summit),
                        )
                        shape = value**float(
                            peak.get('sharpness', 1.6)
                        )
                    z += float(
                        peak.get('height', 0.0)
                    )*shape

            for ridge in terrain.get('ridges', []):
                cx, cy = (
                    float(v)
                    for v in ridge['center']
                )
                sx, sy = (
                    float(v)
                    for v in ridge['sigma']
                )
                dx = (x-cx)/max(sx, 1.0e-6)
                dy = (y-cy)/max(sy, 1.0e-6)
                z += float(ridge['height'])*math.exp(
                    -0.5*(dx*dx+dy*dy)
                )

            for terrace in terrain.get(
                'terraces',
                [],
            ):
                weight = (
                    self.ellipse_weight(
                        x,
                        y,
                        terrace,
                    )
                    *float(
                        terrace.get('strength', 1.0)
                    )
                )
                z = (
                    (1.0-weight)*z
                    +weight*float(terrace['target_z'])
                )
        else:
            for terrace in terrain.get(
                'terraces',
                [],
            ):
                weight = (
                    self.ellipse_weight(
                        x,
                        y,
                        terrace,
                    )
                    *float(
                        terrace.get('strength', 1.0)
                    )
                )
                z = (
                    (1.0-weight)*z
                    +weight*float(terrace['target_z'])
                )

            for ridge in terrain.get('ridges', []):
                cx, cy = (
                    float(v)
                    for v in ridge['center']
                )
                sx, sy = (
                    float(v)
                    for v in ridge['sigma']
                )
                dx = (x-cx)/max(sx, 1.0e-6)
                dy = (y-cy)/max(sy, 1.0e-6)
                z += float(ridge['height'])*math.exp(
                    -0.5*(dx*dx+dy*dy)
                )

        for canyon in self.canyons():
            distance, ratio = self.segment_distance(
                x,
                y,
                canyon['p0'],
                canyon['p1'],
            )
            floor = (
                float(canyon['floor_z0'])
                +ratio*(
                    float(canyon['floor_z1'])
                    -float(canyon['floor_z0'])
                )
            )
            half = float(canyon['floor_half_width'])
            shoulder = float(canyon['shoulder_width'])
            if distance < shoulder:
                if distance <= half:
                    weight = 1.0
                else:
                    value = max(
                        0.0,
                        min(
                            1.0,
                            (shoulder-distance)
                            /max(1.0e-6, shoulder-half),
                        ),
                    )
                    weight = value**float(
                        canyon.get('edge_power', 1.5)
                    )
                z = (1.0-weight)*z+weight*floor

        return max(0.0, z)

    def resolve_box(
        self,
        spec: Dict,
    ) -> Tuple[Point3, Point3, float]:
        size = tuple(float(v) for v in spec['size'])
        yaw = math.radians(
            float(spec.get('yaw_deg', 0.0))
        )
        if 'center' in spec:
            center = tuple(
                float(v) for v in spec['center']
            )
            return center, size, yaw

        x, y = (
            float(v) for v in spec['center_xy']
        )
        center_z = (
            self.terrain_height(x, y)
            +size[2]*0.5
        )
        return (x, y, center_z), size, yaw

    @staticmethod
    def box_signed_distance_and_nearest(
        point: Point3,
        box: Dict,
    ) -> Tuple[float, Point3]:
        px, py, pz = point
        cx, cy, cz = box['center']
        sx, sy, sz = box['size']
        yaw = box['yaw']

        dx, dy, dz = px-cx, py-cy, pz-cz
        lx = math.cos(yaw)*dx+math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx+math.cos(yaw)*dy
        lz = dz

        hx, hy, hz = sx*0.5, sy*0.5, sz*0.5

        outside = (
            max(abs(lx)-hx, 0.0),
            max(abs(ly)-hy, 0.0),
            max(abs(lz)-hz, 0.0),
        )
        outside_distance = math.sqrt(
            sum(value*value for value in outside)
        )

        inside_margin = min(
            hx-abs(lx),
            hy-abs(ly),
            hz-abs(lz),
        )
        inside = (
            abs(lx) <= hx
            and abs(ly) <= hy
            and abs(lz) <= hz
        )

        if inside:
            signed_distance = -inside_margin
            nearest = [lx, ly, lz]
            margins = [
                hx-abs(lx),
                hy-abs(ly),
                hz-abs(lz),
            ]
            axis = margins.index(min(margins))
            half = [hx, hy, hz][axis]
            value = [lx, ly, lz][axis]
            nearest[axis] = (
                half if value >= 0.0 else -half
            )
            nx, ny, nz = nearest
        else:
            signed_distance = outside_distance
            nx = max(-hx, min(hx, lx))
            ny = max(-hy, min(hy, ly))
            nz = max(-hz, min(hz, lz))

        world_x = (
            cx+math.cos(yaw)*nx-math.sin(yaw)*ny
        )
        world_y = (
            cy+math.sin(yaw)*nx+math.cos(yaw)*ny
        )
        world_z = cz+nz
        return (
            signed_distance,
            (world_x, world_y, world_z),
        )

    def nearest_static_surface(
        self,
        point: Point3,
    ) -> Dict:
        x, y, z = point

        terrain_z = self.terrain_height(x, y)
        best = {
            'distance_m': z-terrain_z,
            'type': 'terrain',
            'id': 'terrain',
            'nearest_point': (x, y, terrain_z),
        }

        for box in self.boxes:
            distance, nearest = (
                self.box_signed_distance_and_nearest(
                    point,
                    box,
                )
            )
            if distance < best['distance_m']:
                best = {
                    'distance_m': distance,
                    'type': 'collision_box',
                    'id': box['id'],
                    'nearest_point': nearest,
                }

        return best


class RouteClearanceEvaluator(Node):
    """Read-only P0-P15 center/body clearance evidence."""

    def __init__(self) -> None:
        super().__init__('cq01_course_route_clearance_evaluator')

        self.declare_parameter('scene_file', '')
        self.declare_parameter('odom_topic', '/drone/odom')
        self.declare_parameter(
            'path_topic',
            '/drone/planned_path_3d',
        )
        self.declare_parameter(
            'route_status_topic',
            '/cq01_v9/route/status',
        )
        self.declare_parameter(
            'status_topic',
            '/cq01_v9/course/clearance_status',
        )
        self.declare_parameter(
            'clearance_topic',
            '/cq01_v9/course/body_clearance',
        )
        self.declare_parameter(
            'marker_topic',
            '/cq01_v9/course/clearance_markers',
        )
        self.declare_parameter(
            'results_root',
            '~/drone_task/your_drone_sim_ws/results/'
            'cq01_v9_mc/course_clearance',
        )
        self.declare_parameter(
            'drone_collision_radius_m',
            0.60,
        )
        self.declare_parameter(
            'safety_distance_m',
            0.50,
        )
        self.declare_parameter('sample_rate_hz', 20.0)

        scene_file = str(
            self.get_parameter('scene_file').value
        )
        if not scene_file:
            raise RuntimeError(
                'scene_file parameter is required'
            )
        self.geometry = SceneGeometry(scene_file)

        self.drone_radius = float(
            self.get_parameter(
                'drone_collision_radius_m'
            ).value
        )
        self.safety_distance = float(
            self.get_parameter(
                'safety_distance_m'
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
        self.sample_period = 1.0/max(
            1.0,
            float(
                self.get_parameter(
                    'sample_rate_hz'
                ).value
            ),
        )

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient,
        )
        self.clearance_pub = self.create_publisher(
            Float64,
            str(
                self.get_parameter(
                    'clearance_topic'
                ).value
            ),
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            50,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('path_topic').value),
            self.path_callback,
            transient,
        )
        self.create_subscription(
            String,
            str(
                self.get_parameter(
                    'route_status_topic'
                ).value
            ),
            self.route_callback,
            transient,
        )

        self.route_status: Dict = {}
        self.started = False
        self.completed = False
        self.saved = False
        self.start_time: Optional[float] = None
        self.last_sample_time = -1.0e9
        self.output_dir: Optional[Path] = None

        self.samples: List[Dict] = []
        self.path_segments: List[Dict] = []
        self.path_signature = None
        self.minimum_sample: Optional[Dict] = None
        self.latest_sample: Optional[Dict] = None

        self.get_logger().warning(
            'Course route-clearance evaluator ready: '
            f'{len(self.geometry.boxes)} collision boxes; '
            f'drone radius={self.drone_radius:.2f} m; '
            f'safety margin={self.safety_distance:.2f} m.'
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds/1.0e9

    def route_callback(self, message: String) -> None:
        try:
            self.route_status = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(
            self.route_status.get('state', '')
        )
        current_index = int(
            self.route_status.get('current_index', -1)
        )

        if (
            not self.started
            and (
                bool(self.route_status.get('active', False))
                or current_index >= 0
            )
            and state not in (
                'waiting_for_start',
                'idle',
            )
        ):
            self.start_evaluation()

        if (
            self.started
            and not self.completed
            and state == 'mission_completed'
        ):
            self.completed = True
            self.save_result('mission_completed')

    def start_evaluation(self) -> None:
        self.started = True
        self.start_time = self.now_s()
        stamp = datetime.now().strftime(
            'route_clearance_%Y%m%d_%H%M%S'
        )
        self.output_dir = self.results_root/stamp
        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )
        self.get_logger().warning(
            'P0-P15 STATIC CLEARANCE EVALUATION STARTED.'
        )

    def path_callback(self, message: NavPath) -> None:
        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]
        if len(points) < 2:
            return

        signature = (
            len(points),
            tuple(round(v, 2) for v in points[0]),
            tuple(round(v, 2) for v in points[-1]),
        )
        if signature == self.path_signature:
            return

        self.path_signature = signature
        segment_id = len(self.path_segments)
        self.path_segments.append({
            'segment_id': segment_id,
            'route_index': int(
                self.route_status.get(
                    'current_index',
                    -1,
                )
            ),
            'points': points,
        })

    def odom_callback(self, message: Odometry) -> None:
        if not self.started or self.completed:
            return

        now = self.now_s()
        if now-self.last_sample_time < self.sample_period:
            return
        self.last_sample_time = now

        p = message.pose.pose.position
        point = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        nearest = self.geometry.nearest_static_surface(
            point
        )
        center_clearance = float(nearest['distance_m'])
        body_clearance = center_clearance-self.drone_radius

        sample = {
            'time_s': now,
            'route_elapsed_s':
                now-(self.start_time or now),
            'x_m': point[0],
            'y_m': point[1],
            'z_m': point[2],
            'route_current_index': int(
                self.route_status.get(
                    'current_index',
                    -1,
                )
            ),
            'route_reached_index': int(
                self.route_status.get(
                    'reached_index',
                    -1,
                )
            ),
            'nearest_type': nearest['type'],
            'nearest_id': nearest['id'],
            'nearest_x_m': nearest['nearest_point'][0],
            'nearest_y_m': nearest['nearest_point'][1],
            'nearest_z_m': nearest['nearest_point'][2],
            'center_clearance_m': center_clearance,
            'drone_collision_radius_m':
                self.drone_radius,
            'body_surface_clearance_m':
                body_clearance,
            'safety_distance_m':
                self.safety_distance,
            'pass_at_sample':
                body_clearance >= self.safety_distance,
        }

        self.samples.append(sample)
        self.latest_sample = sample

        if (
            self.minimum_sample is None
            or body_clearance
            < self.minimum_sample[
                'body_surface_clearance_m'
            ]
        ):
            self.minimum_sample = sample

        clearance_message = Float64()
        clearance_message.data = body_clearance
        self.clearance_pub.publish(clearance_message)

        self.publish_markers(point, nearest)
        self.publish_status()

    def publish_markers(
        self,
        drone: Point3,
        nearest: Dict,
    ) -> None:
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        line = Marker()
        line.header.frame_id = 'world'
        line.header.stamp = now
        line.ns = 'cq01_course_clearance'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.18

        body_clearance = (
            float(nearest['distance_m'])
            -self.drone_radius
        )
        if body_clearance >= self.safety_distance:
            line.color.r = 0.10
            line.color.g = 1.00
            line.color.b = 0.20
        else:
            line.color.r = 1.00
            line.color.g = 0.10
            line.color.b = 0.05
        line.color.a = 0.95

        first = Point()
        first.x, first.y, first.z = drone
        second = Point()
        (
            second.x,
            second.y,
            second.z,
        ) = nearest['nearest_point']
        line.points.extend([first, second])
        array.markers.append(line)

        surface = Marker()
        surface.header = line.header
        surface.ns = line.ns
        surface.id = 1
        surface.type = Marker.SPHERE
        surface.action = Marker.ADD
        surface.pose.position = second
        surface.pose.orientation.w = 1.0
        surface.scale.x = 0.55
        surface.scale.y = 0.55
        surface.scale.z = 0.55
        surface.color.r = line.color.r
        surface.color.g = line.color.g
        surface.color.b = line.color.b
        surface.color.a = 1.0
        array.markers.append(surface)

        self.marker_pub.publish(array)

    def publish_status(self) -> None:
        payload = {
            'state': (
                'completed'
                if self.completed
                else (
                    'running'
                    if self.started
                    else 'waiting_for_route'
                )
            ),
            'sample_count': len(self.samples),
            'latest': self.latest_sample,
            'minimum': self.minimum_sample,
            'drone_collision_radius_m':
                self.drone_radius,
            'safety_distance_m':
                self.safety_distance,
            'required_center_clearance_m':
                self.drone_radius+self.safety_distance,
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

    def save_result(self, final_state: str) -> None:
        if self.saved or self.output_dir is None:
            return
        self.saved = True

        samples_path = (
            self.output_dir/'route_clearance.csv'
        )
        if self.samples:
            with samples_path.open(
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

        planned_path = (
            self.output_dir/'planned_paths.csv'
        )
        rows = []
        for segment in self.path_segments:
            for point_index, point in enumerate(
                segment['points']
            ):
                rows.append({
                    'segment_id':
                        segment['segment_id'],
                    'route_index':
                        segment['route_index'],
                    'point_index': point_index,
                    'x_m': point[0],
                    'y_m': point[1],
                    'z_m': point[2],
                })
        if rows:
            with planned_path.open(
                'w',
                encoding='utf-8',
                newline='',
            ) as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=list(rows[0].keys()),
                )
                writer.writeheader()
                writer.writerows(rows)

        minimum = self.minimum_sample
        minimum_body = (
            minimum['body_surface_clearance_m']
            if minimum else None
        )
        minimum_center = (
            minimum['center_clearance_m']
            if minimum else None
        )

        criteria = {
            'route_started': self.started,
            'route_completed':
                final_state == 'mission_completed',
            'sufficient_samples':
                len(self.samples) >= 100,
            'no_static_collision':
                minimum_center is not None
                and minimum_center > self.drone_radius,
            'minimum_body_clearance_above_safety':
                minimum_body is not None
                and minimum_body >= self.safety_distance,
            'planned_paths_recorded':
                len(self.path_segments) >= 1,
        }

        summary = {
            'experiment':
                'P0_P15_global_static_obstacle_clearance',
            'final_state': final_state,
            'pass': all(criteria.values()),
            'failed_criteria': [
                key
                for key, value in criteria.items()
                if value is not True
            ],
            'criteria': criteria,
            'map_representation':
                'exact terrain surface plus rotated collision boxes',
            'distance_definition': {
                'center_clearance_m':
                    'minimum signed distance from vehicle center '
                    'to terrain or collision-box surface',
                'body_surface_clearance_m':
                    'center_clearance minus spherical vehicle radius',
            },
            'drone_collision_radius_m':
                self.drone_radius,
            'safety_distance_m':
                self.safety_distance,
            'required_center_clearance_m':
                self.drone_radius+self.safety_distance,
            'minimum_center_clearance_m':
                minimum_center,
            'minimum_body_surface_clearance_m':
                minimum_body,
            'minimum_sample': minimum,
            'sample_count': len(self.samples),
            'planned_path_segment_count':
                len(self.path_segments),
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
            'P0-P15 STATIC CLEARANCE RESULT '
            f'pass={summary["pass"]}; '
            f'min_body={minimum_body}; '
            f'failed={summary["failed_criteria"]}; '
            f'output={self.output_dir}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteClearanceEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.started and not node.saved:
            node.save_result('interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
