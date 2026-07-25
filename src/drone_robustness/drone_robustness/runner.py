#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import signal
import shutil
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCENARIOS = [
    {
        "name": "nominal",
        "position_std_m": 0.0,
        "velocity_std_mps": 0.0,
        "seed": 2026,
    },
    {
        "name": "noise_1cm",
        "position_std_m": 0.01,
        "velocity_std_mps": 0.02,
        "seed": 2026,
    },
    {
        "name": "noise_3cm",
        "position_std_m": 0.03,
        "velocity_std_mps": 0.05,
        "seed": 2026,
    },
]


def list_results(root: Path) -> List[Path]:
    return [
        path
        for path in root.glob("avoidance_*")
        if path.is_dir()
    ]


def find_new_result(
    root: Path,
    known: List[Path],
    start_time: float,
) -> Optional[Path]:
    known_paths = {path.resolve() for path in known}
    candidates = []

    for path in list_results(root):
        if path.resolve() in known_paths:
            continue
        if path.stat().st_mtime < start_time - 2.0:
            continue
        if not (path / "metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        return None

    return max(candidates, key=lambda path: path.stat().st_mtime)


def stop_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    time.sleep(1.0)
    os.killpg(os.getpgid(process.pid), signal.SIGINT)

    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def copy_artifacts(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)

    for filename in [
        "flight_data.csv",
        "metrics.json",
        "summary.txt",
        "trajectory_xy.png",
        "tracking_error.png",
        "obstacle_clearance.png",
        "motor_rpm.png",
        "position_response.png",
    ]:
        path = source / filename
        if path.exists():
            shutil.copy2(path, destination / filename)


def load_metrics(path: Path) -> Dict[str, object]:
    return json.loads(
        (path / "metrics.json").read_text(encoding="utf-8")
    )


def finite(value) -> bool:
    return (
        isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def run_scenario(
    scenario: Dict[str, object],
    raw_root: Path,
    use_rviz: bool,
    timeout_seconds: float,
) -> Path:
    known = list_results(raw_root)
    start_time = time.time()

    command = [
        "ros2",
        "launch",
        "drone_robustness",
        "noise_corridor_evaluation.launch.py",
        f"use_rviz:={'true' if use_rviz else 'false'}",
        (
            "position_noise_std_m:="
            f"{scenario['position_std_m']}"
        ),
        (
            "velocity_noise_std_mps:="
            f"{scenario['velocity_std_mps']}"
        ),
        f"seed:={scenario['seed']}",
    ]

    print()
    print(
        f"===== Scenario: {scenario['name']} ====="
    )
    print(
        f"Position noise std: "
        f"{scenario['position_std_m']} m"
    )
    print(
        f"Velocity noise std: "
        f"{scenario['velocity_std_mps']} m/s"
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
            f"Scenario {scenario['name']} produced no result."
        )

    return result


def create_plots(
    output_dir: Path,
    records: List[Dict[str, object]],
) -> None:
    labels = [str(record["scenario"]) for record in records]

    def bar_plot(
        filename: str,
        key: str,
        title: str,
        ylabel: str,
        threshold=None,
    ) -> None:
        values = [float(record[key]) for record in records]
        plt.figure(figsize=(8.5, 5.2))
        plt.bar(labels, values)

        if threshold is not None:
            plt.axhline(
                threshold,
                linestyle="--",
                label="acceptance threshold",
            )
            plt.legend()

        plt.title(title)
        plt.xlabel("Feedback-noise scenario")
        plt.ylabel(ylabel)
        plt.grid(True, axis="y")
        plt.tight_layout()
        plt.savefig(output_dir / filename, dpi=180)
        plt.close()

    bar_plot(
        "arrival_time_comparison.png",
        "arrival_time_s",
        "Arrival Time under Odometry Noise",
        "Arrival time (s)",
    )
    bar_plot(
        "final_error_comparison.png",
        "final_error_m",
        "Final Error under Odometry Noise",
        "Final error (m)",
        threshold=0.3,
    )
    bar_plot(
        "path_ratio_comparison.png",
        "actual_over_planned_path_ratio",
        "Path Efficiency under Odometry Noise",
        "Actual/planned path ratio",
    )
    bar_plot(
        "clearance_comparison.png",
        "minimum_obstacle_clearance_m",
        "Minimum Obstacle Clearance under Noise",
        "Clearance (m)",
        threshold=0.0,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Enable RViz for each scenario",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=80.0,
    )
    arguments = parser.parse_args()

    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    raw_root = workspace / "results" / "robustness_raw"
    raw_root.mkdir(parents=True, exist_ok=True)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = (
        workspace
        / "results"
        / f"robustness_noise_{timestamp}"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    records: List[Dict[str, object]] = []

    for scenario in SCENARIOS:
        result_dir = run_scenario(
            scenario=scenario,
            raw_root=raw_root,
            use_rviz=arguments.rviz,
            timeout_seconds=arguments.timeout,
        )
        metrics = load_metrics(result_dir)

        record: Dict[str, object] = {
            "scenario": scenario["name"],
            "position_noise_std_m": scenario[
                "position_std_m"
            ],
            "velocity_noise_std_mps": scenario[
                "velocity_std_mps"
            ],
            "success": bool(metrics.get("success")),
            "arrival_time_s": metrics.get("arrival_time_s"),
            "final_error_m": metrics.get("final_error_m"),
            "steady_state_error_m": metrics.get(
                "steady_state_error_m"
            ),
            "planned_path_length_m": metrics.get(
                "planned_path_length_m"
            ),
            "actual_path_length_m": metrics.get(
                "actual_path_length_m"
            ),
            "actual_over_planned_path_ratio": metrics.get(
                "actual_over_planned_path_ratio"
            ),
            "minimum_obstacle_clearance_m": metrics.get(
                "minimum_obstacle_clearance_m"
            ),
            "minimum_safety_zone_clearance_m": metrics.get(
                "minimum_safety_zone_clearance_m"
            ),
            "maximum_motor_rpm": metrics.get(
                "maximum_motor_rpm"
            ),
            "rpm_saturation_ratio": metrics.get(
                "rpm_saturation_ratio"
            ),
            "source_result_directory": str(result_dir),
        }
        records.append(record)

        copy_artifacts(
            result_dir,
            output_dir / str(scenario["name"]),
        )

        print(
            f"Completed {scenario['name']}: "
            f"success={record['success']}, "
            f"arrival={record['arrival_time_s']}, "
            f"final_error={record['final_error_m']}, "
            f"clearance="
            f"{record['minimum_obstacle_clearance_m']}"
        )

    nominal = records[0]

    for record in records:
        for key in [
            "arrival_time_s",
            "final_error_m",
            "actual_over_planned_path_ratio",
            "minimum_obstacle_clearance_m",
        ]:
            nominal_value = nominal.get(key)
            value = record.get(key)

            if finite(nominal_value) and finite(value):
                record[f"{key}_change_vs_nominal"] = (
                    float(value) - float(nominal_value)
                )
            else:
                record[f"{key}_change_vs_nominal"] = None

    criteria = {
        "all_scenarios_successful": all(
            bool(record["success"])
            for record in records
        ),
        "all_final_errors_below_0_3_m": all(
            finite(record["final_error_m"])
            and float(record["final_error_m"]) < 0.3
            for record in records
        ),
        "all_physical_clearances_positive": all(
            finite(record["minimum_obstacle_clearance_m"])
            and float(
                record["minimum_obstacle_clearance_m"]
            ) > 0.0
            for record in records
        ),
        "all_safety_clearances_positive": all(
            finite(
                record["minimum_safety_zone_clearance_m"]
            )
            and float(
                record["minimum_safety_zone_clearance_m"]
            ) > 0.0
            for record in records
        ),
        "all_path_ratios_below_1_4": all(
            finite(record["actual_over_planned_path_ratio"])
            and float(
                record["actual_over_planned_path_ratio"]
            ) < 1.4
            for record in records
        ),
        "no_motor_saturation": all(
            finite(record["rpm_saturation_ratio"])
            and float(record["rpm_saturation_ratio"]) <= 0.0
            for record in records
        ),
    }
    overall_pass = all(criteria.values())

    fieldnames = []
    for record in records:
        for key in record:
            if key not in fieldnames:
                fieldnames.append(key)

    with (output_dir / "robustness_runs.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(records)

    summary_data = {
        "scenario": "corridor_odometry_noise_robustness",
        "overall_pass": overall_pass,
        "criteria": criteria,
        "records": records,
    }
    (output_dir / "robustness_summary.json").write_text(
        json.dumps(
            summary_data,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    lines = [
        "===== Corridor Odometry-Noise Robustness Summary =====",
        f"Overall pass: {overall_pass}",
        "",
    ]

    for record in records:
        lines.extend([
            f"[{record['scenario']}]",
            (
                "Position noise std: "
                f"{record['position_noise_std_m']:.3f} m"
            ),
            (
                "Velocity noise std: "
                f"{record['velocity_noise_std_mps']:.3f} m/s"
            ),
            f"Success: {record['success']}",
            (
                "Arrival time: "
                f"{float(record['arrival_time_s']):.3f} s"
            ),
            (
                "Final error: "
                f"{float(record['final_error_m']):.4f} m"
            ),
            (
                "Actual/planned ratio: "
                f"{float(record['actual_over_planned_path_ratio']):.3f}"
            ),
            (
                "Minimum obstacle clearance: "
                f"{float(record['minimum_obstacle_clearance_m']):.3f} m"
            ),
            (
                "Minimum safety clearance: "
                f"{float(record['minimum_safety_zone_clearance_m']):.3f} m"
            ),
            (
                "Maximum motor RPM: "
                f"{float(record['maximum_motor_rpm']):.1f} RPM"
            ),
            "",
        ])

    lines.append("Acceptance criteria:")
    for key, passed in criteria.items():
        lines.append(
            f"- {key}: {'PASS' if passed else 'FAIL'}"
        )

    summary_text = "\n".join(lines) + "\n"
    (output_dir / "robustness_summary.txt").write_text(
        summary_text,
        encoding="utf-8",
    )

    create_plots(output_dir, records)

    print()
    print(summary_text)
    print(f"Results directory: {output_dir}")

    if not overall_pass:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
