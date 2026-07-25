#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import yaml
from ament_index_python.packages import (
    get_package_share_directory,
)


def latest_summary_directory(root: Path) -> Optional[Path]:
    candidates = sorted(
        path.parent
        for path in root.glob('*/summary.json')
    )
    return candidates[-1] if candidates else None


def read_csv(path: Path) -> List[Dict]:
    with path.open('r', encoding='utf-8') as file:
        return list(csv.DictReader(file))


def floats(rows: List[Dict], key: str) -> List[float]:
    values = []
    for row in rows:
        try:
            values.append(float(row[key]))
        except (KeyError, TypeError, ValueError):
            values.append(float('nan'))
    return values


def save_figure(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, dpi=220)
    plt.close()


def rotated_box_polygon(spec: Dict):
    if 'center_xy' not in spec:
        center = spec.get('center', [0.0, 0.0, 0.0])
        cx, cy = float(center[0]), float(center[1])
    else:
        cx, cy = (float(v) for v in spec['center_xy'])

    sx, sy = (
        float(spec['size'][0]),
        float(spec['size'][1]),
    )
    yaw = math.radians(
        float(spec.get('yaw_deg', 0.0))
    )

    corners = []
    for lx, ly in (
        (-sx/2.0, -sy/2.0),
        (sx/2.0, -sy/2.0),
        (sx/2.0, sy/2.0),
        (-sx/2.0, sy/2.0),
    ):
        x = cx+math.cos(yaw)*lx-math.sin(yaw)*ly
        y = cy+math.sin(yaw)*lx+math.cos(yaw)*ly
        corners.append((x, y))
    return corners


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--workspace',
        default=os.path.expanduser(
            '~/drone_task/your_drone_sim_ws'
        ),
    )
    parser.add_argument('--course-dir')
    parser.add_argument('--route-dir')
    parser.add_argument('--scene-file')
    parser.add_argument('--output-dir')
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser()

    course_dir = (
        Path(args.course_dir).expanduser()
        if args.course_dir
        else latest_summary_directory(
            workspace/'results/cq01_course_acceptance'
        )
    )
    route_dir = (
        Path(args.route_dir).expanduser()
        if args.route_dir
        else latest_summary_directory(
            workspace
            /'results/cq01_v9_mc/course_clearance'
        )
    )

    if course_dir is None:
        raise SystemExit(
            'No course acceptance result found.'
        )
    if route_dir is None:
        raise SystemExit(
            'No P0-P15 clearance result found.'
        )

    if args.scene_file:
        scene_file = Path(args.scene_file).expanduser()
    else:
        scene_file = Path(
            get_package_share_directory(
                'cq01_v2_scene'
            )
        )/'config/cq01_v34g_scene.yaml'

    if args.output_dir:
        output_dir = Path(args.output_dir).expanduser()
    else:
        output_dir = (
            workspace/'results/cq01_course_figures'
            /datetime.now().strftime(
                'figures_%Y%m%d_%H%M%S'
            )
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    course_rows = read_csv(
        course_dir/'telemetry.csv'
    )
    route_rows = read_csv(
        route_dir/'route_clearance.csv'
    )

    with (course_dir/'summary.json').open(
        'r',
        encoding='utf-8',
    ) as file:
        course_summary = json.load(file)
    with (route_dir/'summary.json').open(
        'r',
        encoding='utf-8',
    ) as file:
        route_summary = json.load(file)
    with scene_file.open('r', encoding='utf-8') as file:
        scene = yaml.safe_load(file)['scene']

    manifest = {
        'course_result': str(course_dir),
        'route_result': str(route_dir),
        'scene_file': str(scene_file),
        'figures': [],
    }

    # 01. Position error.
    time = floats(course_rows, 'mission_elapsed_s')
    error = floats(course_rows, 'position_error_m')
    plt.figure(figsize=(10, 5.2))
    plt.plot(time, error, label='Position error')
    plt.axhline(
        0.30,
        linestyle='--',
        label='0.30 m acceptance threshold',
    )
    plt.xlabel('Mission time (s)')
    plt.ylabel('Position error (m)')
    plt.title('Hover, Single-Target and Multi-Target Position Error')
    plt.grid(True)
    plt.legend()
    path = output_dir/'01_position_error_curve.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    # 02. Four motor RPM curves.
    plt.figure(figsize=(10, 5.2))
    for index in range(1, 5):
        plt.plot(
            time,
            floats(course_rows, f'rpm_{index}'),
            label=f'Motor {index}',
        )
    plt.axhline(
        9800.0,
        linestyle='--',
        label='98% RPM saturation threshold',
    )
    plt.xlabel('Mission time (s)')
    plt.ylabel('Motor speed (RPM)')
    plt.title('Four-Motor RPM Response')
    plt.grid(True)
    plt.legend()
    path = output_dir/'02_four_motor_rpm_curves.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    # 03. Exact acceptance trajectory.
    plt.figure(figsize=(7.5, 7.0))
    x = floats(course_rows, 'x_m')
    y = floats(course_rows, 'y_m')
    plt.plot(x, y, label='Actual trajectory')
    stage_goals = [
        item['goal']
        for item in course_summary.get(
            'stage_results',
            [],
        )
    ]
    if stage_goals:
        gx = [goal[0] for goal in stage_goals]
        gy = [goal[1] for goal in stage_goals]
        plt.scatter(gx, gy, marker='x', label='Accepted goals')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Exact Course Acceptance Target Sequence')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    path = output_dir/'03_multi_target_actual_trajectory.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    # 04. Planned versus actual P0-P15 top view.
    plt.figure(figsize=(11, 7.5))
    axes = plt.gca()
    for spec in scene.get('collision_boxes', []):
        axes.add_patch(
            Polygon(
                rotated_box_polygon(spec),
                closed=True,
                alpha=0.18,
            )
        )

    planned_path = route_dir/'planned_paths.csv'
    if planned_path.is_file():
        planned_rows = read_csv(planned_path)
        segments = sorted(
            {
                int(float(row['segment_id']))
                for row in planned_rows
            }
        )
        first = True
        for segment in segments:
            rows = [
                row
                for row in planned_rows
                if int(float(row['segment_id'])) == segment
            ]
            plt.plot(
                floats(rows, 'x_m'),
                floats(rows, 'y_m'),
                linewidth=1.0,
                label='A* planned segments' if first else None,
            )
            first = False

    plt.plot(
        floats(route_rows, 'x_m'),
        floats(route_rows, 'y_m'),
        linewidth=1.8,
        label='Actual P0-P15 trajectory',
    )

    minimum = route_summary.get('minimum_sample')
    if minimum:
        plt.scatter(
            [float(minimum['x_m'])],
            [float(minimum['y_m'])],
            marker='x',
            s=90,
            label='Minimum-clearance position',
        )

    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('P0-P15 A* Planned Paths and Actual Trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    path = output_dir/'04_route_planned_vs_actual.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    # 05. Minimum static-obstacle clearance.
    route_time = floats(route_rows, 'route_elapsed_s')
    center_clearance = floats(
        route_rows,
        'center_clearance_m',
    )
    body_clearance = floats(
        route_rows,
        'body_surface_clearance_m',
    )
    safety = float(
        route_summary.get('safety_distance_m', 0.5)
    )

    plt.figure(figsize=(10, 5.2))
    plt.plot(
        route_time,
        center_clearance,
        label='Vehicle-center clearance',
    )
    plt.plot(
        route_time,
        body_clearance,
        label='Vehicle-body surface clearance',
    )
    plt.axhline(
        safety,
        linestyle='--',
        label=f'Safety distance {safety:.2f} m',
    )
    plt.xlabel('P0-P15 route time (s)')
    plt.ylabel('Static-obstacle clearance (m)')
    plt.title('Minimum Static-Obstacle Distance Along P0-P15')
    plt.grid(True)
    plt.legend()
    path = output_dir/'05_minimum_obstacle_distance_curve.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    # 06. Altitude profile.
    plt.figure(figsize=(10, 5.2))
    plt.plot(
        route_time,
        floats(route_rows, 'z_m'),
        label='Vehicle altitude',
    )
    nearest_terrain = [
        float(row['nearest_z_m'])
        if row.get('nearest_type') == 'terrain'
        else float('nan')
        for row in route_rows
    ]
    plt.plot(
        route_time,
        nearest_terrain,
        label='Nearest terrain height when terrain-limited',
    )
    plt.xlabel('P0-P15 route time (s)')
    plt.ylabel('Z (m)')
    plt.title('P0-P15 Altitude Profile')
    plt.grid(True)
    plt.legend()
    path = output_dir/'06_route_altitude_profile.png'
    save_figure(path)
    manifest['figures'].append(str(path))

    (output_dir/'figure_manifest.json').write_text(
        json.dumps(
            manifest,
            ensure_ascii=False,
            indent=2,
        ),
        encoding='utf-8',
    )

    print('COURSE FIGURES GENERATED')
    print('course:', course_dir)
    print('route:', route_dir)
    print('output:', output_dir)
    for item in manifest['figures']:
        print(' -', item)


if __name__ == '__main__':
    main()
