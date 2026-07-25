#!/usr/bin/env python3

import argparse
import csv
import json
import math
import shutil
import os
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Sequence

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


METRIC_SPECS = [
    ("arrival_time_s", "Arrival time", "s"),
    ("final_error_m", "Final error", "m"),
    ("steady_state_error_m", "Steady-state error", "m"),
    ("planned_path_length_m", "Planned path length", "m"),
    ("actual_path_length_m", "Actual path length", "m"),
    (
        "actual_over_planned_path_ratio",
        "Actual/planned path ratio",
        "",
    ),
    (
        "minimum_obstacle_clearance_m",
        "Minimum obstacle clearance",
        "m",
    ),
    (
        "minimum_safety_zone_clearance_m",
        "Minimum safety-zone clearance",
        "m",
    ),
    ("maximum_absolute_roll_deg", "Maximum |roll|", "deg"),
    ("maximum_absolute_pitch_deg", "Maximum |pitch|", "deg"),
    ("maximum_motor_rpm", "Maximum motor RPM", "RPM"),
    ("rpm_saturation_ratio", "RPM saturation ratio", ""),
]


def finite_number(value):
    return (
        isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def newest_result_after(
    results_root: Path,
    known_directories: Sequence[Path],
    launch_start_time: float,
) -> Path:
    known = {path.resolve() for path in known_directories}
    candidates = []

    for path in results_root.glob("avoidance_*"):
        if not path.is_dir():
            continue
        if path.resolve() in known:
            continue
        if path.stat().st_mtime < launch_start_time - 2.0:
            continue
        if not (path / "metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        raise RuntimeError(
            "The launch finished, but no new avoidance result "
            "directory was found."
        )

    return max(
        candidates,
        key=lambda path: path.stat().st_mtime,
    )


def load_metrics(result_directory: Path) -> Dict[str, object]:
    metrics_path = result_directory / "metrics.json"

    if not metrics_path.exists():
        raise FileNotFoundError(
            f"Missing metrics file: {metrics_path}"
        )

    return json.loads(
        metrics_path.read_text(encoding="utf-8")
    )


def calculate_statistics(
    run_records: Sequence[Dict[str, object]],
) -> Dict[str, Dict[str, float]]:
    output: Dict[str, Dict[str, float]] = {}

    for key, _, _ in METRIC_SPECS:
        values = [
            float(record[key])
            for record in run_records
            if finite_number(record.get(key))
        ]

        if not values:
            continue

        output[key] = {
            "count": float(len(values)),
            "mean": statistics.mean(values),
            "std": (
                statistics.stdev(values)
                if len(values) >= 2
                else 0.0
            ),
            "min": min(values),
            "max": max(values),
        }

        mean_value = output[key]["mean"]
        output[key]["coefficient_of_variation"] = (
            output[key]["std"] / abs(mean_value)
            if abs(mean_value) > 1.0e-12
            else 0.0
        )

    return output


def copy_run_artifacts(
    source_directory: Path,
    destination_directory: Path,
) -> None:
    destination_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    filenames = [
        "summary.txt",
        "metrics.json",
        "trajectory_xy.png",
        "tracking_error.png",
        "obstacle_clearance.png",
        "motor_rpm.png",
        "position_response.png",
    ]

    for filename in filenames:
        source = source_directory / filename
        if source.exists():
            shutil.copy2(
                source,
                destination_directory / filename,
            )


def write_runs_csv(
    output_directory: Path,
    run_records: Sequence[Dict[str, object]],
) -> None:
    fieldnames = [
        "run",
        "result_directory",
        "success",
        "termination_reason",
    ]
    fieldnames.extend(key for key, _, _ in METRIC_SPECS)

    with (output_directory / "runs.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
        )
        writer.writeheader()

        for record in run_records:
            writer.writerow(
                {
                    key: record.get(key)
                    for key in fieldnames
                }
            )


def format_stat(
    statistics_map: Dict[str, Dict[str, float]],
    key: str,
    unit: str,
    digits: int = 4,
) -> str:
    record = statistics_map.get(key)

    if record is None:
        return "N/A"

    return (
        f"{record['mean']:.{digits}f} ± "
        f"{record['std']:.{digits}f} {unit}"
        f"  [min={record['min']:.{digits}f}, "
        f"max={record['max']:.{digits}f}]"
    )


def write_summary(
    output_directory: Path,
    run_records: Sequence[Dict[str, object]],
    statistics_map: Dict[str, Dict[str, float]],
) -> Dict[str, object]:
    success_count = sum(
        1
        for record in run_records
        if bool(record.get("success"))
    )

    all_final_errors = [
        float(record["final_error_m"])
        for record in run_records
        if finite_number(record.get("final_error_m"))
    ]
    all_obstacle_clearances = [
        float(record["minimum_obstacle_clearance_m"])
        for record in run_records
        if finite_number(
            record.get("minimum_obstacle_clearance_m")
        )
    ]
    all_safety_clearances = [
        float(record["minimum_safety_zone_clearance_m"])
        for record in run_records
        if finite_number(
            record.get(
                "minimum_safety_zone_clearance_m"
            )
        )
    ]
    all_maximum_rpm = [
        float(record["maximum_motor_rpm"])
        for record in run_records
        if finite_number(record.get("maximum_motor_rpm"))
    ]
    rpm_limits = [
        float(record["rpm_limit"])
        for record in run_records
        if finite_number(record.get("rpm_limit"))
    ]

    criteria = {
        "all_runs_successful": success_count == len(run_records),
        "all_final_errors_below_0_3_m": (
            len(all_final_errors) == len(run_records)
            and all(value < 0.3 for value in all_final_errors)
        ),
        "all_obstacle_clearances_positive": (
            len(all_obstacle_clearances) == len(run_records)
            and all(value > 0.0 for value in all_obstacle_clearances)
        ),
        "all_safety_zone_clearances_positive": (
            len(all_safety_clearances) == len(run_records)
            and all(value > 0.0 for value in all_safety_clearances)
        ),
        "no_motor_saturation": (
            len(all_maximum_rpm) == len(run_records)
            and len(rpm_limits) == len(run_records)
            and all(
                maximum < 0.99 * limit
                for maximum, limit in zip(
                    all_maximum_rpm,
                    rpm_limits,
                )
            )
        ),
    }
    overall_pass = all(criteria.values())

    summary = {
        "scenario": "standard_avoidance_repeatability",
        "run_count": len(run_records),
        "success_count": success_count,
        "overall_pass": overall_pass,
        "criteria": criteria,
        "statistics": statistics_map,
        "runs": run_records,
    }

    (output_directory / "summary.json").write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    lines = [
        "===== Standard Avoidance Repeatability Summary =====",
        f"Runs: {len(run_records)}",
        f"Successful runs: {success_count}",
        f"Overall pass: {overall_pass}",
        "",
        (
            "Arrival time: "
            + format_stat(
                statistics_map,
                "arrival_time_s",
                "s",
                3,
            )
        ),
        (
            "Final error: "
            + format_stat(
                statistics_map,
                "final_error_m",
                "m",
            )
        ),
        (
            "Steady-state error: "
            + format_stat(
                statistics_map,
                "steady_state_error_m",
                "m",
            )
        ),
        (
            "Planned path length: "
            + format_stat(
                statistics_map,
                "planned_path_length_m",
                "m",
                3,
            )
        ),
        (
            "Actual path length: "
            + format_stat(
                statistics_map,
                "actual_path_length_m",
                "m",
                3,
            )
        ),
        (
            "Actual/planned path ratio: "
            + format_stat(
                statistics_map,
                "actual_over_planned_path_ratio",
                "",
                3,
            )
        ),
        (
            "Minimum obstacle clearance: "
            + format_stat(
                statistics_map,
                "minimum_obstacle_clearance_m",
                "m",
                3,
            )
        ),
        (
            "Minimum safety-zone clearance: "
            + format_stat(
                statistics_map,
                "minimum_safety_zone_clearance_m",
                "m",
                3,
            )
        ),
        (
            "Maximum motor RPM: "
            + format_stat(
                statistics_map,
                "maximum_motor_rpm",
                "RPM",
                1,
            )
        ),
        "",
        "Acceptance criteria:",
    ]

    for name, passed in criteria.items():
        lines.append(
            f"- {name}: {'PASS' if passed else 'FAIL'}"
        )

    (output_directory / "summary.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    return summary


def create_bar_plot(
    output_path: Path,
    run_records: Sequence[Dict[str, object]],
    key: str,
    title: str,
    y_label: str,
    threshold=None,
) -> None:
    values = [
        float(record[key])
        for record in run_records
    ]
    run_labels = [
        f"Run {int(record['run'])}"
        for record in run_records
    ]

    plt.figure(figsize=(8.5, 5.2))
    plt.bar(run_labels, values)

    if threshold is not None:
        plt.axhline(
            threshold,
            linestyle="--",
            label="acceptance threshold",
        )
        plt.legend()

    plt.title(title)
    plt.xlabel("Experiment run")
    plt.ylabel(y_label)
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


def create_plots(
    output_directory: Path,
    run_records: Sequence[Dict[str, object]],
) -> None:
    create_bar_plot(
        output_directory / "arrival_time_repeatability.png",
        run_records,
        "arrival_time_s",
        "Arrival Time Repeatability",
        "Arrival time (s)",
    )
    create_bar_plot(
        output_directory / "final_error_repeatability.png",
        run_records,
        "final_error_m",
        "Final Position Error Repeatability",
        "Final error (m)",
        threshold=0.3,
    )
    create_bar_plot(
        output_directory / "path_ratio_repeatability.png",
        run_records,
        "actual_over_planned_path_ratio",
        "Path Tracking Efficiency Repeatability",
        "Actual/planned path ratio",
    )

    run_labels = [
        f"Run {int(record['run'])}"
        for record in run_records
    ]
    obstacle_values = [
        float(record["minimum_obstacle_clearance_m"])
        for record in run_records
    ]
    safety_values = [
        float(record["minimum_safety_zone_clearance_m"])
        for record in run_records
    ]

    indices = list(range(len(run_records)))
    width = 0.35

    plt.figure(figsize=(8.5, 5.2))
    plt.bar(
        [index - width / 2 for index in indices],
        obstacle_values,
        width=width,
        label="physical obstacle clearance",
    )
    plt.bar(
        [index + width / 2 for index in indices],
        safety_values,
        width=width,
        label="safety-zone clearance",
    )
    plt.axhline(
        0.0,
        linestyle="--",
        label="collision boundary",
    )
    plt.xticks(indices, run_labels)
    plt.title("Obstacle Clearance Repeatability")
    plt.xlabel("Experiment run")
    plt.ylabel("Minimum clearance (m)")
    plt.grid(True, axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        output_directory / "clearance_repeatability.png",
        dpi=180,
    )
    plt.close()


def run_repeatability(
    run_count: int,
    use_rviz: bool,
    pause_seconds: float,
) -> Path:
    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    results_root = workspace / "results"
    results_root.mkdir(parents=True, exist_ok=True)

    known_directories = [
        path
        for path in results_root.glob("avoidance_*")
        if path.is_dir()
    ]

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_directory = (
        results_root
        / f"repeatability_standard_{timestamp}"
    )
    output_directory.mkdir(
        parents=True,
        exist_ok=False,
    )

    run_records: List[Dict[str, object]] = []

    print(
        "===== Standard Avoidance Repeatability Test ====="
    )
    print(f"Runs: {run_count}")
    print(f"RViz enabled: {use_rviz}")
    print(f"Output: {output_directory}")
    print()

    for run_index in range(1, run_count + 1):
        print(
            f"===== Starting run {run_index}/{run_count} ====="
        )
        launch_start_time = time.time()

        command = [
            "ros2",
            "launch",
            "drone_evaluation",
            "avoidance_experiment.launch.py",
            f"use_rviz:={'true' if use_rviz else 'false'}",
        ]

        process = subprocess.Popen(
            command,
            start_new_session=True,
        )

        result_directory = None
        result_deadline = time.monotonic() + 70.0

        while time.monotonic() < result_deadline:
            if process.poll() is not None:
                break

            try:
                result_directory = newest_result_after(
                    results_root,
                    known_directories,
                    launch_start_time,
                )
                break
            except RuntimeError:
                time.sleep(0.5)

        if result_directory is None:
            try:
                result_directory = newest_result_after(
                    results_root,
                    known_directories,
                    launch_start_time,
                )
            except RuntimeError as error:
                if process.poll() is None:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGINT,
                    )
                raise RuntimeError(
                    f"Run {run_index} did not produce a result "
                    "within 70 seconds"
                ) from error

        if process.poll() is None:
            time.sleep(1.0)
            os.killpg(
                os.getpgid(process.pid),
                signal.SIGINT,
            )

            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGTERM,
                )
                process.wait(timeout=5.0)

        if process.returncode not in (0, -signal.SIGINT):
            print(
                f"[warning] launch process exited with "
                f"code {process.returncode}, but the result "
                "directory was created successfully"
            )
        known_directories.append(result_directory)

        metrics = load_metrics(result_directory)

        record: Dict[str, object] = {
            "run": run_index,
            "result_directory": str(result_directory),
            "success": bool(metrics.get("success")),
            "termination_reason": metrics.get(
                "termination_reason"
            ),
        }

        for key, _, _ in METRIC_SPECS:
            record[key] = metrics.get(key)

        record["rpm_limit"] = metrics.get("rpm_limit")
        run_records.append(record)

        copy_run_artifacts(
            result_directory,
            output_directory / f"run_{run_index}",
        )

        print(
            f"Run {run_index} complete: "
            f"success={record['success']}, "
            f"arrival={record.get('arrival_time_s')}, "
            f"final_error={record.get('final_error_m')}, "
            f"clearance={record.get('minimum_obstacle_clearance_m')}"
        )
        print()

        if run_index < run_count:
            time.sleep(pause_seconds)

    statistics_map = calculate_statistics(run_records)
    write_runs_csv(output_directory, run_records)
    summary = write_summary(
        output_directory,
        run_records,
        statistics_map,
    )
    create_plots(output_directory, run_records)

    print()
    print(
        (output_directory / "summary.txt").read_text(
            encoding="utf-8"
        )
    )
    print(f"Results directory: {output_directory}")

    if not bool(summary["overall_pass"]):
        print(
            "One or more acceptance criteria failed.",
            file=sys.stderr,
        )
        sys.exit(2)

    return output_directory


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run the standard A-star avoidance experiment "
            "multiple times and aggregate repeatability metrics."
        )
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=3,
        help="Number of repeated experiments (default: 3)",
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Enable RViz during repeated experiments",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=2.0,
        help="Pause between experiments in seconds",
    )

    arguments = parser.parse_args()

    if arguments.runs < 2:
        parser.error("--runs must be at least 2")

    run_repeatability(
        run_count=arguments.runs,
        use_rviz=arguments.rviz,
        pause_seconds=max(0.0, arguments.pause),
    )


if __name__ == "__main__":
    main()
