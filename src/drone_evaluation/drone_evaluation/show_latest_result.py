#!/usr/bin/env python3

import json
from pathlib import Path


def format_value(value, unit="", digits=4):
    if value is None:
        return "N/A"

    if isinstance(value, bool):
        return str(value)

    if isinstance(value, (int, float)):
        return f"{float(value):.{digits}f}{unit}"

    return str(value)


def main():
    results_root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
    )

    directories = sorted(
        (
            path
            for path in results_root.glob("avoidance_*")
            if path.is_dir()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not directories:
        print("No avoidance result directory found.")
        return

    latest = directories[0]
    metrics_path = latest / "metrics.json"

    print(f"Latest result: {latest}")

    if not metrics_path.exists():
        print("metrics.json is missing.")
        return

    metrics = json.loads(
        metrics_path.read_text(encoding="utf-8")
    )

    print(f"Success: {metrics.get('success')}")
    print(
        "Arrival time: "
        + format_value(metrics.get("arrival_time_s"), " s", 3)
    )
    print(
        "Final error: "
        + format_value(metrics.get("final_error_m"), " m")
    )
    print(
        "Steady-state error: "
        + format_value(
            metrics.get("steady_state_error_m"),
            " m",
        )
    )
    print(
        "Planned path length: "
        + format_value(
            metrics.get("planned_path_length_m"),
            " m",
            3,
        )
    )
    print(
        "Actual path length: "
        + format_value(
            metrics.get("actual_path_length_m"),
            " m",
            3,
        )
    )
    print(
        "Actual/planned ratio: "
        + format_value(
            metrics.get(
                "actual_over_planned_path_ratio"
            ),
            "",
            3,
        )
    )
    print(
        "Minimum obstacle clearance: "
        + format_value(
            metrics.get(
                "minimum_obstacle_clearance_m"
            ),
            " m",
            3,
        )
    )
    print(
        "Minimum safety-zone clearance: "
        + format_value(
            metrics.get(
                "minimum_safety_zone_clearance_m"
            ),
            " m",
            3,
        )
    )
    print(
        "Maximum motor RPM: "
        + format_value(
            metrics.get("maximum_motor_rpm"),
            " RPM",
            1,
        )
    )


if __name__ == "__main__":
    main()
