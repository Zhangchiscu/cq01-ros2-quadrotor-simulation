#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import struct
from typing import Dict, Iterable, List, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

Point3 = Tuple[float, float, float]
Color4 = Tuple[float, float, float, float]


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def make_cloud(points: Sequence[Point3], frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp)
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', float(x), float(y), float(z)) for x, y, z in points)
    return msg


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    value = start
    while value <= stop + 1e-6:
        yield value
        value += step


class ScenePublisher(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v2_scene_publisher')
        share = get_package_share_directory('cq01_v2_scene')
        self.declare_parameter('scene_file', os.path.join(share, 'config', 'cq01_v34_scene.yaml'))
        self.declare_parameter('mission_file', os.path.join(share, 'config', 'cq01_v34_mission.yaml'))
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('map_topic', '/map_generator/global_cloud')
        self.declare_parameter('viz_cloud_topic', '/cq01_v2/collision_cloud')
        self.declare_parameter('marker_topic', '/cq01_v2/markers')
        self.declare_parameter('terrain_marker_topic', '/cq01_v2/terrain_markers')
        self.declare_parameter('structure_marker_topic', '/cq01_v2/structure_markers')
        self.declare_parameter('annotation_marker_topic', '/cq01_v2/annotation_markers')
        self.declare_parameter('waypoint_topic', '/cq01_v2/waypoints')
        self.declare_parameter('path_topic', '/cq01_v2/reference_path')
        self.declare_parameter('publish_period_s', 1.0)

        scene_file = self.get_parameter('scene_file').value
        mission_file = self.get_parameter('mission_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.map_topic = self.get_parameter('map_topic').value
        with open(scene_file, 'r', encoding='utf-8') as f:
            self.scene: Dict = yaml.safe_load(f)['scene']
        with open(mission_file, 'r', encoding='utf-8') as f:
            self.mission: Dict = yaml.safe_load(f)['mission']
        self.frame_id = str(self.scene.get('frame_id', self.frame_id))

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(PointCloud2, self.map_topic, qos)
        viz_topic = self.get_parameter('viz_cloud_topic').value
        self.viz_cloud_pub = None if viz_topic == self.map_topic else self.create_publisher(PointCloud2, viz_topic, qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.terrain_marker_pub = self.create_publisher(MarkerArray, self.get_parameter('terrain_marker_topic').value, qos)
        self.structure_marker_pub = self.create_publisher(MarkerArray, self.get_parameter('structure_marker_topic').value, qos)
        self.annotation_marker_pub = self.create_publisher(MarkerArray, self.get_parameter('annotation_marker_topic').value, qos)
        self.waypoint_pub = self.create_publisher(PoseArray, self.get_parameter('waypoint_topic').value, qos)
        self.path_pub = self.create_publisher(Path, self.get_parameter('path_topic').value, qos)

        self.collision_points = self._build_collision_cloud()
        self.terrain_markers, nid = self._build_terrain_markers(0)
        self.structure_markers, nid = self._build_structure_markers(nid)
        self.annotation_markers, _ = self._build_annotation_markers(nid)
        self.markers = MarkerArray(markers=list(self.terrain_markers.markers)+list(self.structure_markers.markers)+list(self.annotation_markers.markers))
        self.waypoints, self.path = self._build_waypoints()
        period = max(0.2, float(self.get_parameter('publish_period_s').value))
        self.timer = self.create_timer(period, self.publish_all)
        self.publish_all()
        self.get_logger().info(
            f'{self.scene.get("name", "CQ-01")} ready: {len(self.collision_points)} collision points, '
            f'{len(self.scene.get("collision_boxes", []))} collision boxes, '
            f'{len(self.mission.get("waypoints", []))} waypoints. map_topic={self.map_topic}')

    @staticmethod
    def _interp_profile(x: float, points: Sequence[Sequence[float]]) -> float:
        if x <= float(points[0][0]):
            return float(points[0][1])
        for p0, p1 in zip(points[:-1], points[1:]):
            x0, z0 = float(p0[0]), float(p0[1])
            x1, z1 = float(p1[0]), float(p1[1])
            if x <= x1:
                t = (x-x0)/max(1e-9, x1-x0)
                return z0+t*(z1-z0)
        return float(points[-1][1])

    @staticmethod
    def _ellipse_weight(x: float, y: float, spec: Dict) -> float:
        cx, cy = (float(v) for v in spec['center'])
        rx, ry = (float(v) for v in spec['radius'])
        yaw = math.radians(float(spec.get('yaw_deg', 0.0)))
        dx, dy = x-cx, y-cy
        lx = math.cos(yaw)*dx + math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
        r2 = (lx/max(rx,1e-6))**2 + (ly/max(ry,1e-6))**2
        if r2 >= 1.0:
            return 0.0
        t = 1.0-math.sqrt(max(0.0,r2))
        return t*t*(3.0-2.0*t)

    @staticmethod
    def _segment_distance(x: float, y: float, p0, p1) -> Tuple[float, float]:
        ax, ay = float(p0[0]), float(p0[1]); bx, by = float(p1[0]), float(p1[1])
        vx, vy = bx-ax, by-ay; wx, wy = x-ax, y-ay
        den = max(1e-9, vx*vx+vy*vy)
        t = max(0.0, min(1.0, (wx*vx+wy*vy)/den))
        px, py = ax+t*vx, ay+t*vy
        return math.hypot(x-px,y-py), t

    @staticmethod
    def _smoothstep(value: float) -> float:
        u = max(0.0, min(1.0, float(value)))
        return u*u*(3.0-2.0*u)

    @staticmethod
    def _superellipse_radius(x: float, y: float, spec: Dict) -> float:
        cx, cy = (float(v) for v in spec['center'])
        rx, ry = (float(v) for v in spec['radius'])
        yaw = math.radians(float(spec.get('yaw_deg', 0.0)))
        power = max(1.0, float(spec.get('super_power', 4.0)))
        dx, dy = x-cx, y-cy
        lx = math.cos(yaw)*dx + math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
        return ((abs(lx/max(rx,1e-6))**power)+(abs(ly/max(ry,1e-6))**power))**(1.0/power)

    @classmethod
    def _plateau_weight(cls, radius: float, spec: Dict) -> float:
        if radius >= 1.0:
            return 0.0
        top = max(0.0, min(0.90, float(spec.get('top_ratio', 0.60))))
        if radius <= top:
            return 1.0
        u = (radius-top)/max(1e-6, 1.0-top)
        return (1.0-cls._smoothstep(u))**float(spec.get('edge_power', 1.6))

    def _canyons(self) -> List[Dict]:
        terrain = self.scene.get('terrain', {})
        items = list(terrain.get('canyons', []))
        if not items and terrain.get('canyon'):
            items = [terrain['canyon']]
        return items

    def _visual_surface_enabled(self, x: float, y: float) -> bool:
        mask = self.scene.get('terrain_visual', {}).get('mask')
        if not mask:
            return True
        cx, cy = (float(v) for v in mask.get('center', [215.0, 0.0]))
        rx, ry = (float(v) for v in mask.get('radius', [225.0, 160.0]))
        yaw = math.radians(float(mask.get('yaw_deg', 0.0)))
        dx, dy = x-cx, y-cy
        lx = math.cos(yaw)*dx + math.sin(yaw)*dy
        ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
        angle = math.atan2(ly/max(ry,1e-6), lx/max(rx,1e-6))
        radius = math.sqrt((lx/max(rx,1e-6))**2+(ly/max(ry,1e-6))**2)
        rough = 1.0
        rough += float(mask.get('roughness_1', 0.07))*math.sin(float(mask.get('frequency_1', 5.0))*angle+0.4)
        rough += float(mask.get('roughness_2', 0.04))*math.sin(float(mask.get('frequency_2', 11.0))*angle-0.8)
        notch = float(mask.get('side_notch', 0.05))*math.cos(2.0*angle)
        return radius <= rough-notch

    def terrain_height(self, x: float, y: float = 0.0) -> float:
        terrain = self.scene['terrain']
        model = str(terrain.get('model', ''))
        if model not in ('mountain_city_v3', 'mountain_city_v31', 'mountain_city_v33'):
            segments = terrain['segments']
            for seg in segments:
                x0, x1 = float(seg['x0']), float(seg['x1'])
                if x0 <= x <= x1:
                    z0, z1 = float(seg['z0']), float(seg['z1'])
                    return z1 if abs(x1-x0) < 1e-9 else z0 + (x-x0)/(x1-x0)*(z1-z0)
            return float(segments[0]['z0']) if x < float(segments[0]['x0']) else float(segments[-1]['z1'])

        z = self._interp_profile(x, terrain['base_profile']) + float(terrain.get('lateral_tilt', 0.0))*y
        und = terrain.get('undulation', {})
        z += float(und.get('amp_x', 0.0))*math.sin(2*math.pi*x/max(1e-6, float(und.get('period_x', 50.0))))
        z += float(und.get('amp_y', 0.0))*math.sin(2*math.pi*y/max(1e-6, float(und.get('period_y', 50.0))))
        z += float(und.get('cross_amp', 0.0))*math.sin(2*math.pi*(x+y)/max(1e-6, float(und.get('cross_period', 75.0))))

        # V3.3 uses broad urban tablelands with steep escarpments.  It avoids
        # alpine needle peaks: buildings sit on large mountain terraces and
        # the diagonal river gorge cuts through the urban mass.
        if model == 'mountain_city_v33':
            for plateau in terrain.get('urban_plateaus', []):
                radius = self._superellipse_radius(x, y, plateau)
                weight = self._plateau_weight(radius, plateau)
                candidate = (1.0-weight)*z + weight*float(plateau['target_z'])
                z = max(z, candidate)

            for ridge in terrain.get('ridges', []):
                cx, cy = (float(v) for v in ridge['center'])
                sx, sy = (float(v) for v in ridge['sigma'])
                dx, dy = (x-cx)/max(sx, 1e-6), (y-cy)/max(sy, 1e-6)
                z += float(ridge['height'])*math.exp(-0.5*(dx*dx+dy*dy))

            for terrace in terrain.get('terraces', []):
                w = self._ellipse_weight(x, y, terrace)*float(terrace.get('strength', 1.0))
                z = (1.0-w)*z + w*float(terrace['target_z'])

        # V3.1 adds explicit mountain masses with a small summit plateau.
        elif model == 'mountain_city_v31':
            for peak in terrain.get('peaks', []):
                cx, cy = (float(v) for v in peak['center'])
                rx, ry = (float(v) for v in peak['radius'])
                yaw = math.radians(float(peak.get('yaw_deg', 0.0)))
                dx, dy = x-cx, y-cy
                lx = math.cos(yaw)*dx + math.sin(yaw)*dy
                ly = -math.sin(yaw)*dx + math.cos(yaw)*dy
                r = math.sqrt((lx/max(rx, 1e-6))**2 + (ly/max(ry, 1e-6))**2)
                if r < 1.0:
                    summit = max(0.0, min(0.8, float(peak.get('summit_radius', 0.15))))
                    if r <= summit:
                        shape = 1.0
                    else:
                        q = max(0.0, (1.0-r)/max(1e-6, 1.0-summit))
                        shape = q**float(peak.get('sharpness', 1.6))
                    z += float(peak.get('height', 0.0))*shape

            for ridge in terrain.get('ridges', []):
                cx, cy = (float(v) for v in ridge['center'])
                sx, sy = (float(v) for v in ridge['sigma'])
                dx, dy = (x-cx)/max(sx, 1e-6), (y-cy)/max(sy, 1e-6)
                z += float(ridge['height'])*math.exp(-0.5*(dx*dx+dy*dy))

            # Small, local terraces make the mountain-top building clusters
            # plausible without turning the whole world into large flat slabs.
            for terrace in terrain.get('terraces', []):
                w = self._ellipse_weight(x, y, terrace)*float(terrace.get('strength', 1.0))
                z = (1.0-w)*z + w*float(terrace['target_z'])
        else:
            for terrace in terrain.get('terraces', []):
                w = self._ellipse_weight(x, y, terrace)*float(terrace.get('strength', 1.0))
                z = (1.0-w)*z + w*float(terrace['target_z'])
            for ridge in terrain.get('ridges', []):
                cx, cy = (float(v) for v in ridge['center'])
                sx, sy = (float(v) for v in ridge['sigma'])
                dx, dy = (x-cx)/max(sx, 1e-6), (y-cy)/max(sy, 1e-6)
                z += float(ridge['height'])*math.exp(-0.5*(dx*dx+dy*dy))

        for canyon in self._canyons():
            d, t = self._segment_distance(x, y, canyon['p0'], canyon['p1'])
            floor = float(canyon['floor_z0']) + t*(float(canyon['floor_z1'])-float(canyon['floor_z0']))
            half = float(canyon['floor_half_width'])
            shoulder = float(canyon['shoulder_width'])
            if d < shoulder:
                if d <= half:
                    w = 1.0
                else:
                    q = max(0.0, min(1.0, (shoulder-d)/max(1e-6, shoulder-half)))
                    w = q**float(canyon.get('edge_power', 1.5))
                z = (1.0-w)*z + w*floor
        return max(0.0, z)

    def _canyon_weight(self, x: float, y: float) -> float:
        best = 0.0
        for canyon in self._canyons():
            d, _ = self._segment_distance(x, y, canyon['p0'], canyon['p1'])
            shoulder = float(canyon['shoulder_width'])
            half = float(canyon['floor_half_width'])
            if d >= shoulder:
                continue
            if d <= half:
                best = max(best, 1.0)
            else:
                best = max(best, max(0.0, min(1.0, (shoulder-d)/max(1e-6, shoulder-half))))
        return best

    def resolve_box(self, spec: Dict) -> Tuple[Point3, Point3, float]:
        size = tuple(float(v) for v in spec['size'])
        yaw = math.radians(float(spec.get('yaw_deg',0.0)))
        if 'center' in spec:
            center=tuple(float(v) for v in spec['center'])
            return center,size,yaw
        x,y=(float(v) for v in spec['center_xy'])
        cz=self.terrain_height(x,y)+size[2]*0.5
        return (x,y,cz),size,yaw

    @staticmethod
    def _rotate_local(x: float, y: float, yaw: float) -> Tuple[float,float]:
        return math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y

    def sample_box_surface(self, center: Point3, size: Point3, yaw: float, resolution: float) -> List[Point3]:
        cx,cy,cz=center; dx,dy,dz=size
        xs=list(frange(-dx*0.5,dx*0.5,resolution)); ys=list(frange(-dy*0.5,dy*0.5,resolution)); zs=list(frange(-dz*0.5,dz*0.5,resolution))
        pts=[]
        def add(lx,ly,lz):
            rx,ry=self._rotate_local(lx,ly,yaw); pts.append((cx+rx,cy+ry,cz+lz))
        for lx in xs:
            for ly in ys: add(lx,ly,-dz*0.5); add(lx,ly,dz*0.5)
        for lx in xs:
            for lz in zs: add(lx,-dy*0.5,lz); add(lx,dy*0.5,lz)
        for ly in ys:
            for lz in zs: add(-dx*0.5,ly,lz); add(dx*0.5,ly,lz)
        return pts

    def _build_collision_cloud(self) -> List[Point3]:
        sampling=self.scene.get('sampling',{}); tr=float(sampling.get('terrain_resolution',2.5)); orr=float(sampling.get('obstacle_resolution',1.5))
        b=self.scene['world_bounds']; pts=[]
        for x in frange(float(b['x_min']),float(b['x_max']),tr):
            for y in frange(float(b['y_min']),float(b['y_max']),tr):
                pts.append((x,y,self.terrain_height(x,y)))
        for spec in self.scene.get('collision_boxes',[]):
            c,s,yaw=self.resolve_box(spec); pts.extend(self.sample_box_surface(c,s,yaw,orr))
        return pts

    @staticmethod
    def _point(x,y,z):
        p=Point();p.x=float(x);p.y=float(y);p.z=float(z);return p

    @staticmethod
    def _rgba(c: Color4) -> ColorRGBA:
        m=ColorRGBA();m.r,m.g,m.b,m.a=(float(v) for v in c);return m

    def _terrain_color(self, x, y, z) -> Color4:
        visual = self.scene.get('terrain_visual', {})
        low = tuple(visual.get('low_color', [0.18, 0.42, 0.22, 1.0]))
        mid = tuple(visual.get('mid_color', [0.48, 0.48, 0.22, 1.0]))
        high = tuple(visual.get('high_color', [0.50, 0.31, 0.17, 1.0]))
        ridge = tuple(visual.get('ridge_color', [0.43, 0.33, 0.27, 1.0]))
        summit = tuple(visual.get('summit_color', [0.55, 0.54, 0.50, 1.0]))
        valley = tuple(visual.get('valley_color', [0.10, 0.26, 0.31, 1.0]))
        breaks = [float(v) for v in visual.get('color_breaks', [22.0, 60.0, 105.0, 145.0])]
        if self._canyon_weight(x, y) > 0.55:
            return valley
        if z < breaks[0]:
            return low
        if z < breaks[1]:
            return mid
        if z < breaks[2]:
            return high
        if z < breaks[3]:
            return ridge
        return summit

    def _build_terrain_markers(self, start_id: int):
        bounds = self.scene['world_bounds']
        visual = self.scene.get('terrain_visual', {})
        res = float(self.scene.get('sampling', {}).get('terrain_visual_resolution', 5.0))
        surface = Marker()
        surface.header.frame_id = self.frame_id
        surface.ns = 'cq01_v33_terrain'
        surface.id = start_id
        surface.type = Marker.TRIANGLE_LIST
        surface.action = Marker.ADD
        surface.pose.orientation.w = 1.0
        surface.scale.x = surface.scale.y = surface.scale.z = 1.0
        xs = list(frange(float(bounds['x_min']), float(bounds['x_max']), res))
        ys = list(frange(float(bounds['y_min']), float(bounds['y_max']), res))
        for x0, x1 in zip(xs[:-1], xs[1:]):
            for y0, y1 in zip(ys[:-1], ys[1:]):
                corners = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
                if not all(self._visual_surface_enabled(x, y) for x, y in corners):
                    continue
                vertices = [(x0,y0),(x1,y0),(x1,y1),(x0,y0),(x1,y1),(x0,y1)]
                for x, y in vertices:
                    z = self.terrain_height(x, y) + 0.03
                    surface.points.append(self._point(x, y, z))
                    surface.colors.append(self._rgba(self._terrain_color(x, y, z)))
        arr = MarkerArray(markers=[surface])
        next_id = start_id + 1

        # A true perimeter instead of the former regular yellow cross-section
        # stripes.  Removing those stripes is important: they made the terrain
        # look like a laboratory height map rather than a mountain city.
        boundary = Marker()
        boundary.header.frame_id = self.frame_id
        boundary.ns = 'cq01_v33_boundary'
        boundary.id = next_id
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = float(visual.get('edge_width', 0.35))
        ec = visual.get('edge_color', [0.82, 0.72, 0.45, 0.65])
        boundary.color.r, boundary.color.g, boundary.color.b, boundary.color.a = (float(v) for v in ec)
        xmin, xmax = float(bounds['x_min']), float(bounds['x_max'])
        ymin, ymax = float(bounds['y_min']), float(bounds['y_max'])
        for x in xs:
            boundary.points.append(self._point(x, ymin, self.terrain_height(x, ymin)+0.10))
        for y in ys:
            boundary.points.append(self._point(xmax, y, self.terrain_height(xmax, y)+0.10))
        for x in reversed(xs):
            boundary.points.append(self._point(x, ymax, self.terrain_height(x, ymax)+0.10))
        for y in reversed(ys):
            boundary.points.append(self._point(xmin, y, self.terrain_height(xmin, y)+0.10))
        boundary.points.append(boundary.points[0])
        if bool(visual.get('show_boundary', True)):
            arr.markers.append(boundary)
            next_id += 1

        # Two dark rock faces make the deep gorge visually explicit from both
        # overview and side views.  They follow the same canyon geometry used by
        # the collision terrain; they are not decorative free-floating walls.
        if bool(visual.get('show_canyon_walls', True)):
            for canyon_index, canyon in enumerate(self._canyons()):
                p0, p1 = canyon['p0'], canyon['p1']
                ax, ay = float(p0[0]), float(p0[1])
                bx, by = float(p1[0]), float(p1[1])
                vx, vy = bx-ax, by-ay
                length = max(1e-9, math.hypot(vx, vy))
                nx, ny = -vy/length, vx/length
                offset = float(canyon.get('visual_wall_offset', canyon.get('floor_half_width', 8.0)+5.0))
                steps = max(12, int(length/max(2.0, res)))
                wall_color = tuple(visual.get('cliff_color', [0.24, 0.20, 0.18, 1.0]))
                for side in (-1.0, 1.0):
                    wall = Marker()
                    wall.header.frame_id = self.frame_id
                    wall.ns = f'cq01_v33_canyon_wall_{canyon_index}'
                    wall.id = next_id
                    wall.type = Marker.TRIANGLE_LIST
                    wall.action = Marker.ADD
                    wall.pose.orientation.w = 1.0
                    for i in range(steps):
                        t0, t1 = i/steps, (i+1)/steps
                        points = []
                        for t in (t0, t1):
                            cx, cy = ax+t*vx, ay+t*vy
                            x, y = cx+side*offset*nx, cy+side*offset*ny
                            floor = float(canyon['floor_z0'])+t*(float(canyon['floor_z1'])-float(canyon['floor_z0']))
                            top = self.terrain_height(x, y)+0.02
                            points.append((x, y, floor, top))
                        x0,y0,b0,tz0 = points[0]
                        x1,y1,b1,tz1 = points[1]
                        for vertex in ((x0,y0,b0),(x1,y1,b1),(x1,y1,tz1),(x0,y0,b0),(x1,y1,tz1),(x0,y0,tz0)):
                            wall.points.append(self._point(*vertex))
                            wall.colors.append(self._rgba(wall_color))
                    arr.markers.append(wall)
                    next_id += 1
        return arr, next_id

    def _cube_marker(self,mid,ns,center,size,yaw,color):
        m=Marker();m.header.frame_id=self.frame_id;m.ns=ns;m.id=mid;m.type=Marker.CUBE;m.action=Marker.ADD;m.pose.position.x,m.pose.position.y,m.pose.position.z=center;m.pose.orientation=yaw_to_quaternion(yaw);m.scale.x,m.scale.y,m.scale.z=size;m.color.r,m.color.g,m.color.b,m.color.a=color;return m

    @staticmethod
    def zone_color(zone:str,alpha:float,key:str='') -> Color4:
        palette={'A':(0.30,0.48,0.68),'B':(0.68,0.47,0.28),'C':(0.42,0.56,0.48),'D':(0.54,0.56,0.60),'E':(0.48,0.37,0.63),'F':(0.70,0.39,0.24),'V':(0.46,0.42,0.38)}
        base=palette.get(zone,(0.55,0.56,0.58));variation=((sum(ord(ch) for ch in key)%9)-4)*0.025
        return tuple(max(0.05,min(0.95,c+variation)) for c in base)+(alpha,)

    def _build_structure_markers(self,start_id:int):
        v=self.scene.get('structure_visual',{});ca=float(v.get('collision_alpha',0.96));va=float(v.get('visual_only_alpha',0.88));arr=MarkerArray();mid=start_id
        for spec in self.scene.get('collision_boxes',[]):
            c,s,yaw=self.resolve_box(spec);arr.markers.append(self._cube_marker(mid,'cq01_v3_collision',c,s,yaw,self.zone_color(str(spec.get('zone','')),ca,str(spec.get('id','')))));mid+=1
        for spec in self.scene.get('visual_only_boxes',[]):
            c,s,yaw=self.resolve_box(spec);arr.markers.append(self._cube_marker(mid,'cq01_v3_skyline',c,s,yaw,self.zone_color(str(spec.get('zone','V')),va,str(spec.get('id','')))));mid+=1
        return arr,mid

    def _build_annotation_markers(self, start_id: int):
        arr = MarkerArray()
        marker_id = start_id
        for i, waypoint in enumerate(self.mission.get('waypoints', [])):
            x, y, z = (float(v) for v in waypoint['position'])
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.ns = 'cq01_v33_goals'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = x, y, z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.8
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.38, 0.08, 1.0
            arr.markers.append(sphere)
            marker_id += 1
            label = Marker()
            label.header.frame_id = self.frame_id
            label.ns = 'cq01_v33_goal_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, z+2.5
            label.pose.orientation.w = 1.0
            label.scale.z = 1.4
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'{i}: {waypoint["name"]}'
            arr.markers.append(label)
            marker_id += 1

        for callout in self.scene.get('terrain_visual', {}).get('callouts', []):
            x, y = (float(v) for v in callout['position_xy'])
            z = float(callout.get('z', self.terrain_height(x, y)+8.0))
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = 'cq01_v33_terrain_callouts'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, z
            text.pose.orientation.w = 1.0
            text.scale.z = float(callout.get('scale', 3.2))
            color = callout.get('color', [1.0, 0.88, 0.25, 1.0])
            text.color.r, text.color.g, text.color.b, text.color.a = (float(v) for v in color)
            text.text = str(callout['text'])
            arr.markers.append(text)
            marker_id += 1
        return arr, marker_id

    def _build_waypoints(self):
        poses=PoseArray();poses.header.frame_id=self.frame_id;path=Path();path.header.frame_id=self.frame_id
        for w in self.mission.get('waypoints',[]):
            x,y,z=(float(v) for v in w['position']);yaw=math.radians(float(w.get('yaw_deg',0)))
            p=Pose();p.position.x=x;p.position.y=y;p.position.z=z;p.orientation=yaw_to_quaternion(yaw);poses.poses.append(p)
            ps=PoseStamped();ps.header.frame_id=self.frame_id;ps.pose=p;path.poses.append(ps)
        return poses,path

    @staticmethod
    def _stamp(markers,stamp):
        for m in markers.markers:m.header.stamp=stamp

    def publish_all(self):
        now=self.get_clock().now().to_msg();cloud=make_cloud(self.collision_points,self.frame_id,now);self.map_pub.publish(cloud)
        if self.viz_cloud_pub is not None:self.viz_cloud_pub.publish(cloud)
        for arr in (self.markers,self.terrain_markers,self.structure_markers,self.annotation_markers):self._stamp(arr,now)
        self.marker_pub.publish(self.markers);self.terrain_marker_pub.publish(self.terrain_markers);self.structure_marker_pub.publish(self.structure_markers);self.annotation_marker_pub.publish(self.annotation_markers)
        self.waypoints.header.stamp=now;self.waypoint_pub.publish(self.waypoints);self.path.header.stamp=now
        for p in self.path.poses:p.header.stamp=now
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args);node=ScenePublisher()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
