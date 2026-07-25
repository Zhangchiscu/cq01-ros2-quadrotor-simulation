#!/usr/bin/env python3
"""Run a deterministic multi-condition dynamic-replanning regression."""

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


CASES = [
    {
        "name": "nominal",
        "speed_mps": 0.55,
        "start_delay_s": 3.0,
        "center_dwell_s": 8.0,
    },
    {
        "name": "slow_crossing",
        "speed_mps": 0.45,
        "start_delay_s": 2.5,
        "center_dwell_s": 8.0,
    },
    {
        "name": "fast_crossing",
        "speed_mps": 0.70,
        "start_delay_s": 3.2,
        "center_dwell_s": 6.0,
    },
    {
        "name": "early_entry",
        "speed_mps": 0.55,
        "start_delay_s": 2.0,
        "center_dwell_s": 8.0,
    },
    {
        "name": "late_entry",
        "speed_mps": 0.55,
        "start_delay_s": 4.5,
        "center_dwell_s": 8.0,
    },
]


def result_directories(root: Path) -> List[Path]:
    if not root.exists():
        return []
    return [path for path in root.iterdir() if path.is_dir()]


def find_new_result(
    root: Path,
    known: List[Path],
    started: float,
) -> Optional[Path]:
    known_set = {path.resolve() for path in known}
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_set:
            continue
        if not path.name.startswith("dynamic_replanning_"):
            continue
        if path.stat().st_mtime < started - 2.0:
            continue
        if not (path / "metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        return None

    return max(candidates, key=lambda item: item.stat().st_mtime)


def stop_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def run_case(
    specification: Dict[str, object],
    raw_root: Path,
    timeout_seconds: float,
    use_rviz: bool,
) -> Path:
    known = result_directories(raw_root)
    started = time.time()

    command = [
        "ros2",
        "launch",
        "drone_dynamic_replanning",
        "dynamic_replanning.launch.py",
        f"use_rviz:={'true' if use_rviz else 'false'}",
        f"obstacle_speed_mps:={specification['speed_mps']}",
        f"obstacle_start_delay_s:={specification['start_delay_s']}",
        f"obstacle_center_dwell_s:={specification['center_dwell_s']}",
        "obstacle_prediction_horizon_s:=6.0",
        "planning_prediction_horizon_s:=3.5",
    ]

    print()
    print(
        f"===== Running {specification['name']} "
        f"(speed={specification['speed_mps']} m/s, "
        f"delay={specification['start_delay_s']} s, "
        f"dwell={specification['center_dwell_s']} s) ====="
    )

    process = subprocess.Popen(command, start_new_session=True)
    result = None
    deadline = time.monotonic() + timeout_seconds

    try:
        while time.monotonic() < deadline:
            result = find_new_result(
                raw_root,
                known,
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
            f"Case {specification['name']} produced no result."
        )

    time.sleep(1.5)
    return result


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def safe_float(
    value: object,
    default: float = float("nan"),
) -> float:
    if finite(value):
        return float(value)
    return default


def format_number(
    value: object,
    digits: int = 3,
    missing: str = "N/A",
) -> str:
    if not finite(value):
        return missing
    return f"{float(value):.{digits}f}"


def bar_plot(
    output: Path,
    labels: List[str],
    values: List[float],
    filename: str,
    title: str,
    ylabel: str,
) -> None:
    plt.figure(figsize=(8.6, 5.0))
    plt.bar(labels, values)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xticks(rotation=20)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(output / filename, dpi=180)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--timeout", type=float, default=100.0)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--cases",
        nargs="+",
        choices=[case["name"] for case in CASES],
        help="Run only the selected named cases.",
    )
    arguments = parser.parse_args()

    if arguments.repeats < 1:
        raise SystemExit("--repeats must be at least 1")

    selected_cases = CASES
    if arguments.cases:
        requested = set(arguments.cases)
        selected_cases = [
            case
            for case in CASES
            if case["name"] in requested
        ]

    workspace = Path.home() / "drone_task" / "your_drone_sim_ws"
    raw_root = workspace / "results" / "dynamic_replanning_raw"
    raw_root.mkdir(parents=True, exist_ok=True)

    output = (
        workspace
        / "results"
        / f"dynamic_robustness_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    output.mkdir(parents=True, exist_ok=False)

    records: List[Dict[str, object]] = []

    for repeat_index in range(arguments.repeats):
        for specification in selected_cases:
            result = run_case(
                specification,
                raw_root,
                arguments.timeout,
                arguments.rviz,
            )
            metrics = json.loads(
                (result / "metrics.json").read_text(encoding="utf-8")
            )

            record = dict(metrics)
            record["case_name"] = str(specification["name"])
            record["repeat_index"] = repeat_index + 1
            record["obstacle_speed_mps"] = float(
                specification["speed_mps"]
            )
            record["obstacle_start_delay_s"] = float(
                specification["start_delay_s"]
            )
            record["obstacle_center_dwell_s"] = float(
                specification["center_dwell_s"]
            )
            records.append(record)

            destination_name = (
                f"{specification['name']}_run_{repeat_index + 1}"
            )
            shutil.copytree(result, output / destination_name)

            print(
                f"{destination_name}: "
                f"pass={metrics['overall_pass']}, "
                f"arrival={metrics['arrival_time_s']}, "
                f"dynamic_clearance="
                f"{metrics['minimum_dynamic_clearance_m']:.3f}, "
                f"replans={metrics['successful_replans']}/"
                f"{metrics['replan_count']}"
            )

    criteria = {
        "all_cases_passed": all(
            bool(record["overall_pass"]) for record in records
        ),
        "all_missions_successful": all(
            bool(record["success"]) for record in records
        ),
        "every_case_detected_predicted_collision": all(
            int(record["predicted_collision_count"]) >= 1
            for record in records
        ),
        "every_case_completed_online_replan": all(
            int(record["successful_replans"]) >= 1
            for record in records
        ),
        "all_final_errors_below_0_3_m": all(
            finite(record.get("final_error_m"))
            and safe_float(record.get("final_error_m")) < 0.30
            for record in records
        ),
        "all_dynamic_clearances_above_0_3_m": all(
            finite(record.get("minimum_dynamic_clearance_m"))
            and safe_float(
                record.get("minimum_dynamic_clearance_m")
            ) >= 0.30
            for record in records
        ),
        "all_static_clearances_above_0_3_m": all(
            finite(record.get("minimum_static_clearance_m"))
            and safe_float(
                record.get("minimum_static_clearance_m")
            ) >= 0.30
            for record in records
        ),
        "all_no_motor_saturation": all(
            finite(record.get("maximum_motor_rpm"))
            and safe_float(record.get("maximum_motor_rpm")) < 10000.0
            for record in records
        ),
    }
    overall_pass = all(criteria.values())

    rows: List[Dict[str, object]] = []
    for record in records:
        rows.append({
            "case_name": record["case_name"],
            "repeat_index": record["repeat_index"],
            "speed_mps": record["obstacle_speed_mps"],
            "start_delay_s": record["obstacle_start_delay_s"],
            "center_dwell_s": record["obstacle_center_dwell_s"],
            "overall_pass": record["overall_pass"],
            "arrival_time_s": record["arrival_time_s"],
            "final_error_m": record["final_error_m"],
            "minimum_dynamic_clearance_m": (
                record["minimum_dynamic_clearance_m"]
            ),
            "minimum_static_clearance_m": (
                record["minimum_static_clearance_m"]
            ),
            "predicted_collision_count": (
                record["predicted_collision_count"]
            ),
            "hold_count": record["hold_count"],
            "replan_count": record["replan_count"],
            "successful_replans": record["successful_replans"],
            "failed_replans": record.get("failed_replans", 0),
            "recovery_wait_count": (
                record.get("recovery_wait_count", 0)
            ),
            "planner_path_count": record["planner_path_count"],
            "last_replan_latency_ms": (
                record["last_replan_latency_ms"]
            ),
            "maximum_motor_rpm": record["maximum_motor_rpm"],
        })

    with (output / "dynamic_robustness_runs.csv").open(
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

    arrival_values = [
        float(record["arrival_time_s"])
        for record in records
        if finite(record["arrival_time_s"])
    ]
    dynamic_clearances = [
        safe_float(record.get("minimum_dynamic_clearance_m"))
        for record in records
        if finite(record.get("minimum_dynamic_clearance_m"))
    ]
    static_clearances = [
        safe_float(record.get("minimum_static_clearance_m"))
        for record in records
        if finite(record.get("minimum_static_clearance_m"))
    ]
    latencies = [
        float(record["last_replan_latency_ms"])
        for record in records
        if finite(record["last_replan_latency_ms"])
    ]

    aggregate = {
        "overall_pass": overall_pass,
        "run_count": len(records),
        "pass_count": sum(
            1 for record in records if bool(record["overall_pass"])
        ),
        "success_rate": (
            sum(1 for record in records if bool(record["overall_pass"]))
            / len(records)
        ),
        "criteria": criteria,
        "mean_arrival_time_s": (
            sum(arrival_values) / len(arrival_values)
            if arrival_values else None
        ),
        "worst_dynamic_clearance_m": (
            min(dynamic_clearances)
            if dynamic_clearances
            else None
        ),
        "worst_static_clearance_m": (
            min(static_clearances)
            if static_clearances
            else None
        ),
        "maximum_replan_latency_ms": (
            max(latencies) if latencies else None
        ),
        "total_successful_replans": sum(
            int(record["successful_replans"]) for record in records
        ),
        "total_temporary_failed_replans": sum(
            int(record.get("failed_replans", 0))
            for record in records
        ),
        "records": records,
    }

    (output / "dynamic_robustness_summary.json").write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    labels = [
        f"{record['case_name']}\nR{record['repeat_index']}"
        for record in records
    ]

    lines = [
        "===== Dynamic Replanning Robustness Regression =====",
        f"Overall pass: {overall_pass}",
        f"Run count: {len(records)}",
        f"Pass count: {aggregate['pass_count']}",
        f"Success rate: {aggregate['success_rate'] * 100.0:.1f}%",
        (
            "Mean arrival time: "
            f"{format_number(aggregate['mean_arrival_time_s'])} s"
        ),
        (
            "Worst dynamic clearance: "
            f"{format_number(aggregate['worst_dynamic_clearance_m'])} m"
        ),
        (
            "Worst static clearance: "
            f"{format_number(aggregate['worst_static_clearance_m'])} m"
        ),
        (
            "Maximum replan latency: "
            f"{format_number(aggregate['maximum_replan_latency_ms'])} ms"
        ),
        (
            "Total successful replans: "
            f"{aggregate['total_successful_replans']}"
        ),
        (
            "Total temporary failed replans: "
            f"{aggregate['total_temporary_failed_replans']}"
        ),
        "",
    ]

    for record in records:
        lines.extend([
            (
                f"[{record['case_name']} "
                f"run {record['repeat_index']}]"
            ),
            f"Overall pass: {record['overall_pass']}",
            f"Mission success: {record.get('success')}",
            f"Failure reason: {record.get('failure_reason') or 'none'}",
            (
                "Parameters: "
                f"speed={record['obstacle_speed_mps']:.2f} m/s, "
                f"delay={record['obstacle_start_delay_s']:.2f} s, "
                f"dwell={record['obstacle_center_dwell_s']:.2f} s"
            ),
            (
                "Arrival time: "
                f"{format_number(record.get('arrival_time_s'))} s"
            ),
            (
                "Final error: "
                f"{format_number(record.get('final_error_m'), 4)} m"
            ),
            (
                "Minimum dynamic clearance: "
                f"{format_number(record.get('minimum_dynamic_clearance_m'))} m"
            ),
            (
                "Minimum static clearance: "
                f"{format_number(record.get('minimum_static_clearance_m'))} m"
            ),
            (
                "Successful replans: "
                f"{record.get('successful_replans', 0)}/"
                f"{record.get('replan_count', 0)}"
            ),
            (
                "Temporary failed replans: "
                f"{record.get('failed_replans', 0)}"
            ),
            "",
        ])

    lines.append("Regression criteria:")
    for key, passed in criteria.items():
        lines.append(f"- {key}: {'PASS' if passed else 'FAIL'}")

    summary_text = "\n".join(lines)
    (output / "dynamic_robustness_summary.txt").write_text(
        summary_text + "\n",
        encoding="utf-8",
    )

    plot_jobs = [
        (
            [safe_float(record.get("arrival_time_s")) for record in records],
            "arrival_time_by_case.png",
            "Dynamic Replanning Arrival Time by Case",
            "arrival time (s)",
        ),
        (
            [
                safe_float(record.get("minimum_dynamic_clearance_m"))
                for record in records
            ],
            "dynamic_clearance_by_case.png",
            "Minimum Dynamic-Obstacle Clearance by Case",
            "clearance (m)",
        ),
        (
            [
                safe_float(record.get("successful_replans"), 0.0)
                for record in records
            ],
            "successful_replans_by_case.png",
            "Successful Online Replans by Case",
            "count",
        ),
        (
            [
                safe_float(record.get("replan_count"), 0.0)
                for record in records
            ],
            "replan_requests_by_case.png",
            "Online Replan Requests by Case",
            "count",
        ),
    ]

    for values, filename, title, ylabel in plot_jobs:
        try:
            bar_plot(
                output,
                labels,
                values,
                filename,
                title,
                ylabel,
            )
        except Exception as error:
            print(
                f"[warning] Could not create {filename}: {error}"
            )

    print()
    print(summary_text)
    print(f"Results directory: {output}")

    if not overall_pass:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
