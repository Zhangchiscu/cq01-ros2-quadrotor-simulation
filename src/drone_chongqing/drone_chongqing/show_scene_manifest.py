#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def main() -> None:
    path = Path(get_package_share_directory('drone_chongqing')) / 'config' / 'yuzhong_peninsula.yaml'
    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    scene = data['scene']
    landmarks = data.get('landmarks', [])
    routes = data.get('routes', [])
    print('===== Chongqing Yuzhong Scene Manifest =====')
    print(f"Scene: {scene.get('name')}")
    print(f"Target generic buildings: {scene.get('building_target_count')}")
    print(f"Landmarks: {len(landmarks)}")
    for index, item in enumerate(landmarks, 1):
        print(f"{index:02d}. {item.get('label')} [{item.get('kind')}]")
    print(f"Routes: {len(routes)}")
    for item in routes:
        print(f"- {item.get('label')}: {len(item.get('points', []))} waypoints")


if __name__ == '__main__':
    main()
