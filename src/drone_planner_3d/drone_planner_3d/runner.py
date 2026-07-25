#!/usr/bin/env python3
"""Run overflight and underpass 3D A* benchmarks."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional

SCENARIOS = [
    {
        "scenario": "overflight_3d",
        "mode": "overflight",
        "goal": (9.0, 0.0, 1.5),
    },
    {
        "scenario": "underpass_3d",
        "mode": "underpass",
        "goal": (9.0, 0.0, 1.5),
    },
]


def result_directories(root: Path) -> List[Path]:
    if not root.exists():
        return []
    return [path for path in root.iterdir() if path.is_dir()]


def find_new_result(
    root: Path,
    known: List[Path],
    scenario: str,
    started: float,
) -> Optional[Path]:
    known_set = {path.resolve() for path in known}
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_set:
            continue
        if not path.name.startswith(f"{scenario}_"):
            continue
        if path.stat().st_mtime < started - 2.0:
            continue
        if not (path / "metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        return None
    return max(
        candidates,
        key=lambda item: item.stat().st_mtime,
    )


def stop_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def run_one(
    specification: Dict[str, object],
    raw_root: Path,
    use_rviz: bool,
    timeout_seconds: float,
) -> Path:
    scenario = str(specification["scenario"])
    mode = str(specification["mode"])
    goal = specification["goal"]

    known = result_directories(raw_root)
    started = time.time()

    command = [
        "ros2",
        "launch",
        "drone_planner_3d",
        "mission_3d.launch.py",
        f"scenario:={scenario}",
        f"mission_mode:={mode}",
        f"goal_x:={goal[0]}",
        f"goal_y:={goal[1]}",
        f"goal_z:={goal[2]}",
        f"use_rviz:={'true' if use_rviz else 'false'}",
    ]

    print()
    print(f"===== Running {scenario} =====")

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
                scenario,
                started,
            )
            if result is not None:
                break
            if process.poll() is not None:
                break
            time.sleep(0.5)
    finally:
        stop_process_group(process)

    if result is None:
        raise RuntimeError(
            f"Scenario {scenario} produced no result."
        )
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--timeout", type=float, default=95.0)
    arguments = parser.parse_args()

    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    raw_root = workspace / "results" / "planner_3d_raw"
    raw_root.mkdir(parents=True, exist_ok=True)

    output = (
        workspace
        / "results"
        / f"planner_3d_benchmark_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    output.mkdir(parents=True, exist_ok=False)

    records = []

    for specification in SCENARIOS:
        result = run_one(
            specification,
            raw_root,
            arguments.rviz,
            arguments.timeout,
        )
        metrics = json.loads(
            (result / "metrics.json").read_text(
                encoding="utf-8"
            )
        )
        records.append(metrics)
        shutil.copytree(
            result,
            output / str(specification["scenario"]),
        )

        print(
            f"{specification['scenario']}: "
            f"pass={metrics['overall_pass']}, "
            f"arrival={metrics['arrival_time_s']}, "
            f"max_z={metrics['maximum_altitude_m']}, "
            f"clearance="
            f"{metrics['minimum_voxel_clearance_m']}"
        )

    criteria = {
        "all_scenarios_passed": all(
            bool(record["overall_pass"])
            for record in records
        ),
        "overflight_requires_3d": (
            records[0]["planner_status"][
                "fixed_height_path_exists"
            ] is False
        ),
        "underpass_fixed_height_is_feasible": (
            records[1]["planner_status"][
                "fixed_height_path_exists"
            ] is True
        ),
    }
    overall_pass = all(criteria.values())

    rows = []
    for record in records:
        rows.append({
            "scenario": record["scenario"],
            "overall_pass": record["overall_pass"],
            "arrival_time_s": record["arrival_time_s"],
            "final_error_m": record["final_error_m"],
            "planned_path_length_m": (
                record["planned_path_length_m"]
            ),
            "actual_path_length_m": (
                record["actual_path_length_m"]
            ),
            "path_ratio": (
                record["actual_over_planned_path_ratio"]
            ),
            "minimum_clearance_m": (
                record["minimum_voxel_clearance_m"]
            ),
            "maximum_altitude_m": (
                record["maximum_altitude_m"]
            ),
            "planning_time_ms": (
                record["planner_status"]["planning_time_ms"]
            ),
            "fixed_height_path_exists": (
                record["planner_status"][
                    "fixed_height_path_exists"
                ]
            ),
        })

    with (output / "planner_3d_benchmark.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=list(rows[0].keys()),
        )
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "overall_pass": overall_pass,
        "criteria": criteria,
        "records": records,
    }
    (output / "planner_3d_benchmark.json").write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    lines = [
        "===== 3D A* Benchmark Summary =====",
        f"Overall pass: {overall_pass}",
        "",
    ]
    for record in records:
        lines.extend([
            f"[{record['scenario']}]",
            f"Overall pass: {record['overall_pass']}",
            f"Arrival time: {record['arrival_time_s']} s",
            f"Final error: {record['final_error_m']:.4f} m",
            (
                "Minimum clearance: "
                f"{record['minimum_voxel_clearance_m']:.3f} m"
            ),
            (
                "Maximum altitude: "
                f"{record['maximum_altitude_m']:.3f} m"
            ),
            (
                "Fixed-height path exists: "
                f"{record['planner_status']['fixed_height_path_exists']}"
            ),
            (
                "Planning time: "
                f"{record['planner_status']['planning_time_ms']:.2f} ms"
            ),
            "",
        ])

    lines.append("Benchmark criteria:")
    for key, passed in criteria.items():
        lines.append(
            f"- {key}: {'PASS' if passed else 'FAIL'}"
        )

    text = "\n".join(lines)
    (output / "planner_3d_benchmark.txt").write_text(
        text + "\n",
        encoding="utf-8",
    )
    print()
    print(text)
    print(f"Results directory: {output}")

    if not overall_pass:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
