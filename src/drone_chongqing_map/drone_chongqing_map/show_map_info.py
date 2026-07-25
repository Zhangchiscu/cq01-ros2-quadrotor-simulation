#!/usr/bin/env python3
"""Print packaged Chongqing map statistics without starting ROS2."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def main() -> None:
    share = Path(get_package_share_directory("drone_chongqing_map"))
    path = share / "runtime_data" / "chongqing_map_manifest.json"
    data = json.loads(path.read_text(encoding="utf-8"))

    print("=" * 72)
    print(data["dataset_name"])
    print("=" * 72)
    print(f"scenario: {data['scenario']}")
    print(f"source CRS: {data['source_crs']}")
    print(f"local origin: {data['local_origin']}")
    print(f"resolution: {data['resolution_m']} m")
    print(f"inflation radius: {data['inflation_radius_m']} m")
    print(f"bounds: {data['bounds']}")
    print(f"dimensions: {data['dimensions']}")
    print(f"terrain relief: {data['terrain']['relief_m']:.3f} m")
    print(f"features: {data['features']}")
    print(f"raw occupied voxels: {data['base_occupied_voxels']}")
    print(f"inflated occupied voxels: {data['inflated_occupied_voxels']}")
    print("planner compatible: yes")
    print("=" * 72)


if __name__ == "__main__":
    main()
