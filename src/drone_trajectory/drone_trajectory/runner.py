#!/usr/bin/env python3
"""Run raw and smoothed trajectory comparison."""

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

MODES = ["raw", "smooth"]


def result_directories(root: Path) -> List[Path]:
    if not root.exists():
        return []
    return [path for path in root.iterdir() if path.is_dir()]


def find_new_result(
    root: Path,
    known: List[Path],
    mode: str,
    started: float,
) -> Optional[Path]:
    known_set = {path.resolve() for path in known}
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_set:
            continue
        if not path.name.startswith(f"{mode}_"):
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
    mode: str,
    raw_root: Path,
    use_rviz: bool,
    timeout_seconds: float,
) -> Path:
    known = result_directories(raw_root)
    started = time.time()

    command = [
        "ros2",
        "launch",
        "drone_trajectory",
        "trajectory_mission.launch.py",
        f"mode:={mode}",
        f"use_rviz:={'true' if use_rviz else 'false'}",
    ]

    print()
    print(f"===== Running trajectory mode: {mode} =====")

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
                mode,
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
            f"Trajectory mode {mode} produced no result."
        )
    return result


def create_plot(
    output: Path,
    records: List[Dict[str, object]],
    key: str,
    filename: str,
    title: str,
    ylabel: str,
) -> None:
    labels = [str(record["mode"]) for record in records]
    values = [float(record[key]) for record in records]

    plt.figure(figsize=(7.2, 4.8))
    plt.bar(labels, values)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(output / filename, dpi=180)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--timeout", type=float, default=105.0)
    arguments = parser.parse_args()

    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    raw_root = workspace / "results" / "trajectory_raw"
    raw_root.mkdir(parents=True, exist_ok=True)

    output = (
        workspace
        / "results"
        / f"trajectory_comparison_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    output.mkdir(parents=True, exist_ok=False)

    records: List[Dict[str, object]] = []

    for mode in MODES:
        result = run_one(
            mode,
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
        shutil.copytree(result, output / mode)

        print(
            f"{mode}: pass={metrics['overall_pass']}, "
            f"arrival={metrics['arrival_time_s']}, "
            f"rms_acc={metrics['rms_acceleration_mps2']:.3f}, "
            f"rms_jerk={metrics['rms_jerk_mps3']:.3f}, "
            f"clearance={metrics['minimum_voxel_clearance_m']:.3f}"
        )

    raw_record = records[0]
    smooth_record = records[1]

    raw_reference_jerk = float(
        raw_record["trajectory_status"][
            "rms_reference_jerk_mps3"
        ]
    )
    smooth_reference_jerk = float(
        smooth_record["trajectory_status"][
            "rms_reference_jerk_mps3"
        ]
    )
    raw_reference_acceleration = float(
        raw_record["trajectory_status"][
            "rms_reference_acceleration_mps2"
        ]
    )
    smooth_reference_acceleration = float(
        smooth_record["trajectory_status"][
            "rms_reference_acceleration_mps2"
        ]
    )

    criteria = {
        "both_modes_passed": all(
            bool(record["overall_pass"])
            for record in records
        ),
        "smoothed_reference_jerk_reduced": (
            smooth_reference_jerk < raw_reference_jerk
        ),
        "smoothed_reference_acceleration_not_worse": (
            smooth_reference_acceleration
            <= raw_reference_acceleration * 1.05
        ),
        "smoothed_actual_jerk_not_worse": (
            float(smooth_record["rms_jerk_mps3"])
            <= float(raw_record["rms_jerk_mps3"]) * 1.05
        ),
        "smoothed_actual_acceleration_not_worse": (
            float(smooth_record["rms_acceleration_mps2"])
            <= float(raw_record["rms_acceleration_mps2"]) * 1.05
        ),
        "smoothed_clearance_preserved": (
            float(smooth_record["minimum_voxel_clearance_m"])
            >= 0.30
        ),
        "smoothed_path_length_overhead_below_10_percent": (
            float(smooth_record["reference_path_length_m"])
            <= float(raw_record["reference_path_length_m"]) * 1.10
        ),
    }
    overall_pass = all(criteria.values())

    rows = []
    for record in records:
        rows.append({
            "mode": record["mode"],
            "overall_pass": record["overall_pass"],
            "arrival_time_s": record["arrival_time_s"],
            "final_error_m": record["final_error_m"],
            "reference_path_length_m": (
                record["reference_path_length_m"]
            ),
            "actual_path_length_m": (
                record["actual_path_length_m"]
            ),
            "minimum_clearance_m": (
                record["minimum_voxel_clearance_m"]
            ),
            "maximum_acceleration_mps2": (
                record["maximum_acceleration_mps2"]
            ),
            "rms_acceleration_mps2": (
                record["rms_acceleration_mps2"]
            ),
            "maximum_jerk_mps3": (
                record["maximum_jerk_mps3"]
            ),
            "rms_jerk_mps3": (
                record["rms_jerk_mps3"]
            ),
            "reference_rms_acceleration_mps2": (
                record["trajectory_status"][
                    "rms_reference_acceleration_mps2"
                ]
            ),
            "reference_rms_jerk_mps3": (
                record["trajectory_status"][
                    "rms_reference_jerk_mps3"
                ]
            ),
        })

    with (output / "trajectory_comparison.csv").open(
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
    (output / "trajectory_comparison.json").write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    plots = [
        (
            "arrival_time_s",
            "arrival_time_comparison.png",
            "Arrival Time",
            "time (s)",
        ),
        (
            "rms_acceleration_mps2",
            "actual_rms_acceleration.png",
            "Actual RMS Acceleration",
            "m/s²",
        ),
        (
            "rms_jerk_mps3",
            "actual_rms_jerk.png",
            "Actual RMS Jerk",
            "m/s³",
        ),
        (
            "minimum_voxel_clearance_m",
            "clearance_comparison.png",
            "Minimum Voxel Clearance",
            "clearance (m)",
        ),
    ]
    for key, filename, title, ylabel in plots:
        create_plot(
            output,
            records,
            key,
            filename,
            title,
            ylabel,
        )

    reference_rows = [
        {
            "mode": record["mode"],
            "reference_rms_acceleration_mps2": float(
                record["trajectory_status"][
                    "rms_reference_acceleration_mps2"
                ]
            ),
            "reference_rms_jerk_mps3": float(
                record["trajectory_status"][
                    "rms_reference_jerk_mps3"
                ]
            ),
        }
        for record in records
    ]
    create_plot(
        output,
        reference_rows,
        "reference_rms_acceleration_mps2",
        "reference_rms_acceleration.png",
        "Reference RMS Acceleration",
        "m/s²",
    )
    create_plot(
        output,
        reference_rows,
        "reference_rms_jerk_mps3",
        "reference_rms_jerk.png",
        "Reference RMS Jerk",
        "m/s³",
    )

    lines = [
        "===== Raw vs Smoothed Trajectory Comparison =====",
        f"Overall pass: {overall_pass}",
        "",
    ]
    for record in records:
        lines.extend([
            f"[{record['mode']}]",
            f"Overall pass: {record['overall_pass']}",
            f"Arrival time: {record['arrival_time_s']} s",
            f"Final error: {record['final_error_m']:.4f} m",
            (
                "Minimum clearance: "
                f"{record['minimum_voxel_clearance_m']:.3f} m"
            ),
            (
                "Actual RMS acceleration: "
                f"{record['rms_acceleration_mps2']:.3f} m/s^2"
            ),
            (
                "Actual RMS jerk: "
                f"{record['rms_jerk_mps3']:.3f} m/s^3"
            ),
            (
                "Reference RMS acceleration: "
                f"{record['trajectory_status']['rms_reference_acceleration_mps2']:.3f} m/s^2"
            ),
            (
                "Reference RMS jerk: "
                f"{record['trajectory_status']['rms_reference_jerk_mps3']:.3f} m/s^3"
            ),
            "",
        ])

    lines.append("Comparison criteria:")
    for key, passed in criteria.items():
        lines.append(
            f"- {key}: {'PASS' if passed else 'FAIL'}"
        )

    text = "\n".join(lines)
    (output / "trajectory_comparison.txt").write_text(
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
