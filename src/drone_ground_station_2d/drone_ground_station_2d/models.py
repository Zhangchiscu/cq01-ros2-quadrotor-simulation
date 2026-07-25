from __future__ import annotations

from dataclasses import dataclass, field
from math import asin, atan2, copysign, pi, sqrt
from typing import Dict, List, Optional, Tuple

Point3 = Tuple[float, float, float]


def quaternion_to_euler_deg(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = copysign(pi / 2.0, sinp) if abs(sinp) >= 1.0 else asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    scale = 180.0 / pi
    return roll * scale, pitch * scale, yaw * scale


def yaw_deg_to_quaternion(yaw_deg: float) -> Tuple[float, float, float, float]:
    from math import cos, sin
    half = yaw_deg * pi / 360.0
    return 0.0, 0.0, sin(half), cos(half)


def norm3(values: List[float]) -> float:
    return sqrt(sum(float(v) * float(v) for v in values[:3]))


@dataclass
class VehicleState:
    key: str
    label: str
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    attitude_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angular_velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    goal: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    goal_yaw_deg: float = 0.0
    has_goal: bool = False
    rpm: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    position_error: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    last_odom_monotonic: float = 0.0
    last_rpm_monotonic: float = 0.0
    actual_path: List[Point3] = field(default_factory=list)

    def connected(self, now: float, timeout_s: float = 1.0) -> bool:
        return self.last_odom_monotonic > 0.0 and now - self.last_odom_monotonic <= timeout_s

    def speed(self) -> float:
        return norm3(self.velocity)

    def goal_error(self) -> Optional[float]:
        if not self.has_goal:
            return None
        return sqrt(sum((self.goal[i] - self.position[i]) ** 2 for i in range(3)))


@dataclass
class OccupancyMap:
    version: int = 0
    frame_id: str = 'map'
    resolution: float = 0.0
    width: int = 0
    height: int = 0
    origin_x: float = 0.0
    origin_y: float = 0.0
    occupied_points: Tuple[Tuple[float, float], ...] = ()


@dataclass
class DynamicObstacle:
    position: Optional[Point3] = None
    radius_m: float = 0.35
    height_m: float = 1.5
    phase: str = 'N/A'
    predictions: Tuple[Point3, ...] = ()
    last_update_monotonic: float = 0.0


@dataclass
class SystemState:
    planner_status: Dict[str, object] = field(default_factory=dict)
    trajectory_status: Dict[str, object] = field(default_factory=dict)
    replanning_status: Dict[str, object] = field(default_factory=dict)
    dynamic_status: Dict[str, object] = field(default_factory=dict)
    coordinator_state: str = 'N/A'
    safety_active: bool = False
    current_inter_uav_distance_m: Optional[float] = None
    predicted_inter_uav_distance_m: Optional[float] = None
    last_status_monotonic: Dict[str, float] = field(default_factory=dict)


@dataclass
class GroundSnapshot:
    monotonic_time: float
    vehicles: Dict[str, VehicleState]
    planned_path: Tuple[Point3, ...] = ()
    planned_path_3d: Tuple[Point3, ...] = ()
    trajectory_reference: Tuple[Point3, ...] = ()
    raw_goal: Optional[Point3] = None
    nominal_goal: Optional[Point3] = None
    occupancy: OccupancyMap = field(default_factory=OccupancyMap)
    dynamic_obstacle: DynamicObstacle = field(default_factory=DynamicObstacle)
    system: SystemState = field(default_factory=SystemState)

    def scenario(self) -> str:
        uav1 = self.vehicles['uav1'].connected(self.monotonic_time)
        uav2 = self.vehicles['uav2'].connected(self.monotonic_time)
        if uav1 or uav2:
            return 'MULTI-UAV DECONFLICTION'
        last = self.system.last_status_monotonic
        if self.monotonic_time - last.get('replanning', -1e9) < 2.0:
            return 'DYNAMIC REPLANNING'
        if self.monotonic_time - last.get('planner', -1e9) < 2.0:
            return '3D A* PLANNING'
        if self.monotonic_time - last.get('trajectory', -1e9) < 2.0:
            return 'TRAJECTORY TRACKING'
        if self.vehicles['single'].connected(self.monotonic_time):
            return 'SINGLE UAV'
        return 'WAITING FOR ROS 2 DATA'
