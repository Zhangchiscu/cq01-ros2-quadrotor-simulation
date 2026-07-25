#!/usr/bin/env python3
"""Validate the three dedicated 3D scenarios from their YAML geometry."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from ament_index_python.packages import get_package_share_directory

from drone_world.geometry import (
    expand_obstacles,
    generate_random_obstacles,
)


SCENARIOS = [
    "overflight_3d",
    "underpass_3d",
    "chongqing_multilevel",
]


def load_world(name: str) -> Dict[str, object]:
    path = (
        Path(get_package_share_directory("drone_world"))
        / "config"
        / f"{name}.yaml"
    )
    with path.open("r", encoding="utf-8") as handle:
        configuration = yaml.safe_load(handle)
    return configuration["world"]


def obstacle_top(obstacle: Dict[str, object]) -> float:
    shape = obstacle.get("shape", obstacle.get("type"))
    center_z = float(obstacle["center"][2])

    if shape == "box":
        return center_z + float(obstacle["size"][2]) / 2.0
    if shape == "cylinder":
        return center_z + float(obstacle["height"]) / 2.0
    if shape == "sphere":
        return center_z + float(obstacle["radius"])
    return center_z


def main() -> None:
    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    output = (
        workspace
        / "results"
        / f"voxel_geometry_validation_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    output.mkdir(parents=True, exist_ok=False)

    records: List[Dict[str, object]] = []

    for scenario in SCENARIOS:
        world = load_world(scenario)
        obstacles = expand_obstacles(
            list(world.get("obstacles", []))
            + generate_random_obstacles(
                dict(world.get("random", {}))
            )
        )
        collidable = [
            obstacle
            for obstacle in obstacles
            if bool(obstacle.get("collidable", True))
        ]

        record = {
            "scenario": scenario,
            "obstacle_count": len(collidable),
            "maximum_obstacle_top_m": max(
                obstacle_top(obstacle)
                for obstacle in collidable
            ),
            "bounds": world["bounds"],
            "planning_z_m": float(
                world.get("planning_z", 1.5)
            ),
        }
        records.append(record)

    summary = {
        "overall_pass": all(
            record["obstacle_count"] >= 1
            and record["maximum_obstacle_top_m"] > 2.0
            for record in records
        ),
        "records": records,
    }

    (output / "voxel_geometry_summary.json").write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    labels = [
        record["scenario"]
        for record in records
    ]
    counts = [
        record["obstacle_count"]
        for record in records
    ]
    heights = [
        record["maximum_obstacle_top_m"]
        for record in records
    ]

    plt.figure(figsize=(8.0, 4.8))
    plt.bar(labels, counts)
    plt.ylabel("collidable obstacle count")
    plt.title("Dedicated 3D Scenario Complexity")
    plt.xticks(rotation=12)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(
        output / "obstacle_count.png",
        dpi=180,
    )
    plt.close()

    plt.figure(figsize=(8.0, 4.8))
    plt.bar(labels, heights)
    plt.ylabel("maximum obstacle top (m)")
    plt.title("Vertical Structure of 3D Scenarios")
    plt.xticks(rotation=12)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(
        output / "maximum_height.png",
        dpi=180,
    )
    plt.close()

    lines = [
        "===== Dedicated 3D Scenario Validation =====",
        f"Overall pass: {summary['overall_pass']}",
        "",
    ]
    for record in records:
        lines.extend([
            f"[{record['scenario']}]",
            (
                "Obstacle count: "
                f"{record['obstacle_count']}"
            ),
            (
                "Maximum obstacle top: "
                f"{record['maximum_obstacle_top_m']:.2f} m"
            ),
            (
                "Nominal planning height: "
                f"{record['planning_z_m']:.2f} m"
            ),
            "",
        ])

    text = "\n".join(lines)
    (output / "voxel_geometry_summary.txt").write_text(
        text + "\n",
        encoding="utf-8",
    )
    print(text)
    print(f"Results directory: {output}")


if __name__ == "__main__":
    main()
