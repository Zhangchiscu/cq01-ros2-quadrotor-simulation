#!/usr/bin/env python3
"""Run repeatable navigation regression across drone_world scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ALL_SCENARIOS = [
    {
        "scenario": "warehouse",
        "goal": (6.8, 4.5, 1.5),
    },
    {
        "scenario": "forest",
        "goal": (8.0, 0.0, 1.5),
    },
    {
        "scenario": "urban_canyon",
        "goal": (11.0, 0.0, 1.5),
    },
    {
        "scenario": "tunnel",
        "goal": (8.5, 0.0, 1.5),
    },
    {
        "scenario": "random_clutter",
        "goal": (8.0, 0.0, 1.5),
    },
]

QUICK_NAMES = {
    "warehouse",
    "tunnel",
    "random_clutter",
}


def result_directories(
    root: Path,
) -> List[Path]:
    if not root.exists():
        return []
    return [
        path
        for path in root.iterdir()
        if path.is_dir()
    ]


def find_new_result(
    root: Path,
    known: List[Path],
    start_time: float,
    scenario: str,
) -> Optional[Path]:
    known_resolved = {
        path.resolve()
        for path in known
    }
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_resolved:
            continue
        if not path.name.startswith(f"{scenario}_"):
            continue
        if path.stat().st_mtime < start_time - 2.0:
            continue
        if not (path / "metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        return None

    return max(
        candidates,
        key=lambda path: path.stat().st_mtime,
    )


def terminate_group(
    process: subprocess.Popen,
) -> None:
    if process.poll() is not None:
        return

    try:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGINT,
        )
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(
            os.getpgid(process.pid),
            signal.SIGTERM,
        )
        process.wait(timeout=5.0)


def run_one(
    specification: Dict[str, object],
    raw_root: Path,
    use_rviz: bool,
    timeout_seconds: float,
) -> Path:
    scenario = str(specification["scenario"])
    goal = specification["goal"]

    known = result_directories(raw_root)
    start_time = time.time()

    command = [
        "ros2",
        "launch",
        "drone_world_validation",
        "world_mission.launch.py",
        f"scenario:={scenario}",
        f"goal_x:={goal[0]}",
        f"goal_y:={goal[1]}",
        f"goal_z:={goal[2]}",
        f"use_rviz:={'true' if use_rviz else 'false'}",
    ]

    print()
    print(f"===== Running {scenario} =====")
    print(
        f"Goal: ({goal[0]}, {goal[1]}, {goal[2]})"
    )

    process = subprocess.Popen(
        command,
        start_new_session=True,
    )
    result = None
    deadline = time.monotonic() + timeout_seconds

    try:
        while time.monotonic() < deadline:
            result = find_new_result(
                raw_root,
                known,
                start_time,
                scenario,
            )
            if result is not None:
                break

            if process.poll() is not None:
                break

            time.sleep(0.5)
    finally:
        terminate_group(process)

    if result is None:
        raise RuntimeError(
            f"Scenario {scenario} did not produce a result."
        )

    return result


def copy_result(
    source: Path,
    destination: Path,
) -> None:
    destination.mkdir(
        parents=True,
        exist_ok=True,
    )

    for filename in [
        "flight_data.csv",
        "metrics.json",
        "summary.txt",
        "trajectory_xy.png",
        "position_error.png",
        "obstacle_clearance.png",
        "motor_rpm.png",
    ]:
        path = source / filename
        if path.exists():
            shutil.copy2(
                path,
                destination / filename,
            )


def create_comparison_plots(
    output_directory: Path,
    records: List[Dict[str, object]],
) -> None:
    labels = [
        str(record["scenario"])
        for record in records
    ]

    plots = [
        (
            "arrival_time_comparison.png",
            "arrival_time_s",
            "Arrival Time across 3D World Scenarios",
            "arrival time (s)",
        ),
        (
            "final_error_comparison.png",
            "final_error_m",
            "Final Error across 3D World Scenarios",
            "final error (m)",
        ),
        (
            "clearance_comparison.png",
            "minimum_pointcloud_clearance_m",
            "Minimum Physical Clearance",
            "clearance (m)",
        ),
        (
            "path_ratio_comparison.png",
            "actual_over_planned_path_ratio",
            "Path Tracking Efficiency",
            "actual/planned path ratio",
        ),
    ]

    for filename, key, title, ylabel in plots:
        values = [
            float(record[key])
            if record.get(key) is not None
            else float("nan")
            for record in records
        ]

        plt.figure(figsize=(9.0, 5.0))
        plt.bar(labels, values)
        plt.title(title)
        plt.xlabel("scenario")
        plt.ylabel(ylabel)
        plt.grid(True, axis="y")
        plt.xticks(rotation=15)
        plt.tight_layout()
        plt.savefig(
            output_directory / filename,
            dpi=180,
        )
        plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--quick",
        action="store_true",
        help=(
            "Run warehouse, tunnel and random_clutter only"
        ),
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=75.0,
    )
    arguments = parser.parse_args()

    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    raw_root = (
        workspace
        / "results"
        / "world_validation_raw"
    )
    raw_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_directory = (
        workspace
        / "results"
        / f"world_regression_{timestamp}"
    )
    output_directory.mkdir(
        parents=True,
        exist_ok=False,
    )

    specifications = [
        specification
        for specification in ALL_SCENARIOS
        if (
            not arguments.quick
            or specification["scenario"] in QUICK_NAMES
        )
    ]

    records: List[Dict[str, object]] = []

    for specification in specifications:
        result_directory = run_one(
            specification,
            raw_root,
            arguments.rviz,
            arguments.timeout,
        )
        metrics = json.loads(
            (result_directory / "metrics.json").read_text(
                encoding="utf-8"
            )
        )
        metrics["source_result_directory"] = str(
            result_directory
        )
        records.append(metrics)

        copy_result(
            result_directory,
            output_directory / str(
                specification["scenario"]
            ),
        )

        print(
            f"{specification['scenario']}: "
            f"overall_pass={metrics['overall_pass']}, "
            f"arrival={metrics['arrival_time_s']}, "
            f"clearance="
            f"{metrics['minimum_pointcloud_clearance_m']}"
        )

    criteria = {
        "all_scenarios_passed": all(
            bool(record.get("overall_pass"))
            for record in records
        ),
        "all_scenarios_have_plans": all(
            bool(
                record.get("criteria", {}).get(
                    "planned_path_received"
                )
            )
            for record in records
        ),
        "all_scenarios_collision_free": all(
            bool(
                record.get("criteria", {}).get(
                    "safety_grid_collision_free"
                )
            )
            for record in records
        ),
    }
    overall_pass = all(criteria.values())

    flat_records: List[Dict[str, object]] = []
    for record in records:
        flat_records.append({
            key: value
            for key, value in record.items()
            if key != "criteria"
        })

    fieldnames: List[str] = []
    for record in flat_records:
        for key in record:
            if key not in fieldnames:
                fieldnames.append(key)

    with (output_directory / "world_regression.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(flat_records)

    summary = {
        "overall_pass": overall_pass,
        "quick_mode": bool(arguments.quick),
        "criteria": criteria,
        "records": records,
    }
    (
        output_directory
        / "world_regression_summary.json"
    ).write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    lines = [
        "===== Multi-Scenario World Regression =====",
        f"Overall pass: {overall_pass}",
        f"Scenario count: {len(records)}",
        "",
    ]

    for record in records:
        lines.extend([
            f"[{record['scenario']}]",
            f"Overall pass: {record['overall_pass']}",
            (
                "Arrival time: "
                f"{record['arrival_time_s']} s"
            ),
            (
                "Final error: "
                f"{float(record['final_error_m']):.4f} m"
            ),
            (
                "Minimum point-cloud clearance: "
                f"{float(record['minimum_pointcloud_clearance_m']):.3f} m"
            ),
            (
                "Actual/planned ratio: "
                f"{float(record['actual_over_planned_path_ratio']):.3f}"
            ),
            "",
        ])

    lines.append("Regression criteria:")
    for key, passed in criteria.items():
        lines.append(
            f"- {key}: {'PASS' if passed else 'FAIL'}"
        )

    (
        output_directory
        / "world_regression_summary.txt"
    ).write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    create_comparison_plots(
        output_directory,
        records,
    )

    print()
    print("\n".join(lines))
    print(f"Results directory: {output_directory}")

    if not overall_pass:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
