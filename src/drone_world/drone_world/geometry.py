from __future__ import annotations

import math
import random
from typing import Dict, Iterable, List, Sequence, Tuple

Obstacle = Dict[str, object]


def rotate_xy(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def inverse_rotate_xy(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x + s * y, -s * x + c * y


def expand_doorway(doorway: Obstacle) -> List[Obstacle]:
    cx, cy, _ = [float(v) for v in doorway['center']]
    total_width = float(doorway['total_width'])
    total_height = float(doorway['total_height'])
    opening_width = float(doorway['opening_width'])
    opening_height = float(doorway['opening_height'])
    thickness = float(doorway['thickness'])
    yaw = float(doorway.get('yaw', 0.0))
    base_z = float(doorway.get('base_z', 0.0))
    side_width = (total_width - opening_width) / 2.0
    top_height = total_height - opening_height
    if side_width <= 0.0 or top_height <= 0.0:
        raise ValueError(f"Invalid doorway dimensions: {doorway.get('id')}")

    common = {
        'shape': 'box',
        'type': 'doorway_piece',
        'category': doorway.get('category', 'doorway'),
        'yaw': yaw,
        'collidable': doorway.get('collidable', True),
        'project_to_2d': doorway.get('project_to_2d', True),
        'publish_cloud': doorway.get('publish_cloud', True),
    }
    pieces: List[Obstacle] = []
    offset = opening_width / 2.0 + side_width / 2.0
    for suffix, local_x in [('left', -offset), ('right', offset)]:
        dx, dy = rotate_xy(local_x, 0.0, yaw)
        item = dict(common)
        item.update({
            'id': f"{doorway['id']}_{suffix}",
            'center': [cx + dx, cy + dy, base_z + total_height / 2.0],
            'size': [side_width, thickness, total_height],
        })
        pieces.append(item)

    top = dict(common)
    top.update({
        'id': f"{doorway['id']}_top",
        'center': [cx, cy, base_z + opening_height + top_height / 2.0],
        'size': [opening_width, thickness, top_height],
    })
    pieces.append(top)
    return pieces


def expand_obstacles(raw: Sequence[Obstacle]) -> List[Obstacle]:
    expanded: List[Obstacle] = []
    for index, source in enumerate(raw):
        item = dict(source)
        item.setdefault('id', f'obstacle_{index:03d}')
        item.setdefault('yaw', 0.0)
        item.setdefault('collidable', True)
        item.setdefault('project_to_2d', True)
        item.setdefault('publish_cloud', True)
        obstacle_type = str(item.get('type', 'box'))
        if obstacle_type == 'doorway':
            expanded.extend(expand_doorway(item))
            continue
        if obstacle_type in {'rotated_box', 'wall', 'beam', 'terrain_block', 'building', 'bridge'}:
            item['shape'] = 'box'
        else:
            item['shape'] = obstacle_type
        expanded.append(item)
    return expanded


def generate_random_obstacles(config: Dict[str, object]) -> List[Obstacle]:
    if not bool(config.get('enabled', False)):
        return []
    rng = random.Random(int(config.get('seed', 20260711)))
    count = int(config.get('count', 20))
    bounds = dict(config.get('bounds', {}))
    min_x = float(bounds.get('min_x', -8.0))
    max_x = float(bounds.get('max_x', 8.0))
    min_y = float(bounds.get('min_y', -6.0))
    max_y = float(bounds.get('max_y', 6.0))
    min_size = float(config.get('minimum_size', 0.6))
    max_size = float(config.get('maximum_size', 1.6))
    min_height = float(config.get('minimum_height', 1.0))
    max_height = float(config.get('maximum_height', 4.0))
    cylinder_probability = float(config.get('cylinder_probability', 0.35))
    keepout_radius = float(config.get('keepout_radius', 1.2))
    keepout_points = [[float(v) for v in p] for p in config.get('keepout_points', [])]
    result: List[Obstacle] = []
    attempts = 0
    while len(result) < count and attempts < max(100, count * 100):
        attempts += 1
        x = rng.uniform(min_x, max_x)
        y = rng.uniform(min_y, max_y)
        width = rng.uniform(min_size, max_size)
        depth = rng.uniform(min_size, max_size)
        height = rng.uniform(min_height, max_height)
        footprint = 0.5 * math.hypot(width, depth)
        if any(math.hypot(x - p[0], y - p[1]) < keepout_radius + footprint for p in keepout_points):
            continue
        obstacle_id = f'random_{len(result):03d}'
        if rng.random() < cylinder_probability:
            result.append({
                'id': obstacle_id,
                'type': 'cylinder',
                'shape': 'cylinder',
                'category': 'random',
                'center': [x, y, height / 2.0],
                'radius': 0.5 * min(width, depth),
                'height': height,
                'yaw': 0.0,
                'collidable': True,
                'project_to_2d': True,
                'publish_cloud': True,
            })
        else:
            result.append({
                'id': obstacle_id,
                'type': 'rotated_box',
                'shape': 'box',
                'category': 'random',
                'center': [x, y, height / 2.0],
                'size': [width, depth, height],
                'yaw': rng.uniform(-math.pi, math.pi),
                'collidable': True,
                'project_to_2d': True,
                'publish_cloud': True,
            })
    if len(result) != count:
        raise RuntimeError('Unable to place all random obstacles.')
    return result


def obstacle_contains_xy_at_z(obstacle: Obstacle, x: float, y: float, z: float) -> bool:
    if not bool(obstacle.get('collidable', True)) or not bool(obstacle.get('project_to_2d', True)):
        return False
    shape = str(obstacle.get('shape', obstacle.get('type')))
    center = [float(v) for v in obstacle['center']]
    if shape == 'box':
        size = [float(v) for v in obstacle['size']]
        if abs(z - center[2]) > size[2] / 2.0:
            return False
        lx, ly = inverse_rotate_xy(x - center[0], y - center[1], float(obstacle.get('yaw', 0.0)))
        return abs(lx) <= size[0] / 2.0 and abs(ly) <= size[1] / 2.0
    if shape == 'cylinder':
        if abs(z - center[2]) > float(obstacle['height']) / 2.0:
            return False
        return math.hypot(x - center[0], y - center[1]) <= float(obstacle['radius'])
    if shape == 'sphere':
        radius = float(obstacle['radius'])
        dz = z - center[2]
        if abs(dz) > radius:
            return False
        cross = math.sqrt(max(0.0, radius * radius - dz * dz))
        return math.hypot(x - center[0], y - center[1]) <= cross
    raise ValueError(f'Unsupported obstacle shape: {shape}')


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    value = start
    while value <= stop + 1e-9:
        yield value
        value += step


def sample_obstacle_surface(obstacle: Obstacle, spacing: float) -> List[Tuple[float, float, float]]:
    shape = str(obstacle.get('shape', obstacle.get('type')))
    center = [float(v) for v in obstacle['center']]
    points: List[Tuple[float, float, float]] = []
    if shape == 'box':
        size = [float(v) for v in obstacle['size']]
        hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
        local = set()
        for x in frange(-hx, hx, spacing):
            for y in frange(-hy, hy, spacing):
                local.add((x, y, -hz)); local.add((x, y, hz))
        for x in frange(-hx, hx, spacing):
            for z in frange(-hz, hz, spacing):
                local.add((x, -hy, z)); local.add((x, hy, z))
        for y in frange(-hy, hy, spacing):
            for z in frange(-hz, hz, spacing):
                local.add((-hx, y, z)); local.add((hx, y, z))
        yaw = float(obstacle.get('yaw', 0.0))
        for x, y, z in local:
            rx, ry = rotate_xy(x, y, yaw)
            points.append((center[0] + rx, center[1] + ry, center[2] + z))
        return points
    if shape == 'cylinder':
        radius = float(obstacle['radius'])
        height = float(obstacle['height'])
        angular_count = max(12, int(math.ceil(2.0 * math.pi * radius / spacing)))
        for i in range(angular_count):
            angle = 2.0 * math.pi * i / angular_count
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            for z in frange(-height / 2.0, height / 2.0, spacing):
                points.append((x, y, center[2] + z))
        return points
    if shape == 'sphere':
        radius = float(obstacle['radius'])
        polar_count = max(8, int(math.ceil(math.pi * radius / spacing)))
        for p in range(polar_count + 1):
            polar = math.pi * p / polar_count
            ring_radius = radius * math.sin(polar)
            z = center[2] + radius * math.cos(polar)
            ring_count = max(8, int(math.ceil(2.0 * math.pi * max(ring_radius, spacing) / spacing)))
            for a in range(ring_count):
                azimuth = 2.0 * math.pi * a / ring_count
                points.append((center[0] + ring_radius * math.cos(azimuth), center[1] + ring_radius * math.sin(azimuth), z))
        return points
    raise ValueError(f'Unsupported obstacle shape: {shape}')
