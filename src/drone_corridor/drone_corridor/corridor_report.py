#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


CORRIDOR_X_MIN = 0.50
CORRIDOR_X_MAX = 6.00
OUTER_WALL_INNER_Y = 0.90
DRONE_RADIUS_M = 0.15

FIRST_GATE_X_MIN = 1.80
FIRST_GATE_X_MAX = 2.60
SECOND_GATE_X_MIN = 3.90
SECOND_GATE_X_MAX = 4.70


def finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def latest_result_directory() -> Path:
    results_root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "corridor"
    )

    candidates = sorted(
        (
            path
            for path in results_root.glob("avoidance_*")
            if path.is_dir()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        raise RuntimeError(
            "No corridor evaluation result directory found."
        )

    return candidates[0]


def load_rows(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []

    with csv_path.open(
        "r",
        newline="",
        encoding="utf-8",
    ) as handle:
        reader = csv.DictReader(handle)

        for raw in reader:
            row: Dict[str, float] = {}
            for key, value in raw.items():
                if finite(value):
                    row[key] = float(value)
            rows.append(row)

    return rows


def main() -> None:
    result_dir = latest_result_directory()
    metrics_path = result_dir / "metrics.json"
    flight_path = result_dir / "flight_data.csv"

    if not metrics_path.exists() or not flight_path.exists():
        raise RuntimeError(
            "The latest corridor result is incomplete."
        )

    metrics = json.loads(
        metrics_path.read_text(encoding="utf-8")
    )
    rows = load_rows(flight_path)

    corridor_rows = [
        row
        for row in rows
        if (
            CORRIDOR_X_MIN
            <= row.get("x_m", -1.0e9)
            <= CORRIDOR_X_MAX
        )
    ]
    first_gate_rows = [
        row
        for row in rows
        if (
            FIRST_GATE_X_MIN
            <= row.get("x_m", -1.0e9)
            <= FIRST_GATE_X_MAX
        )
    ]
    second_gate_rows = [
        row
        for row in rows
        if (
            SECOND_GATE_X_MIN
            <= row.get("x_m", -1.0e9)
            <= SECOND_GATE_X_MAX
        )
    ]

    max_abs_y = (
        max(abs(row["y_m"]) for row in corridor_rows)
        if corridor_rows
        else None
    )
    outer_wall_body_clearance = (
        min(
            OUTER_WALL_INNER_Y
            - abs(row["y_m"])
            - DRONE_RADIUS_M
            for row in corridor_rows
        )
        if corridor_rows
        else None
    )
    first_gate_min_y = (
        min(row["y_m"] for row in first_gate_rows)
        if first_gate_rows
        else None
    )
    second_gate_max_y = (
        max(row["y_m"] for row in second_gate_rows)
        if second_gate_rows
        else None
    )

    s_curve_confirmed = (
        first_gate_min_y is not None
        and second_gate_max_y is not None
        and first_gate_min_y < -0.20
        and second_gate_max_y > 0.20
    )

    minimum_obstacle_clearance = metrics.get(
        "minimum_obstacle_clearance_m"
    )
    minimum_safety_clearance = metrics.get(
        "minimum_safety_zone_clearance_m"
    )

    criteria = {
        "goal_reached": bool(metrics.get("success")),
        "final_error_below_0_3_m": (
            finite(metrics.get("final_error_m"))
            and float(metrics["final_error_m"]) < 0.3
        ),
        "physical_clearance_positive": (
            finite(minimum_obstacle_clearance)
            and float(minimum_obstacle_clearance) > 0.0
        ),
        "safety_zone_clearance_positive": (
            finite(minimum_safety_clearance)
            and float(minimum_safety_clearance) > 0.0
        ),
        "outer_wall_body_clearance_positive": (
            outer_wall_body_clearance is not None
            and outer_wall_body_clearance > 0.0
        ),
        "s_curve_passage_confirmed": s_curve_confirmed,
    }
    overall_pass = all(criteria.values())

    corridor_metrics = {
        "scenario": "narrow_s_corridor",
        "source_result_directory": str(result_dir),
        "overall_pass": overall_pass,
        "criteria": criteria,
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
        "minimum_obstacle_clearance_m": (
            minimum_obstacle_clearance
        ),
        "minimum_safety_zone_clearance_m": (
            minimum_safety_clearance
        ),
        "maximum_absolute_lateral_deviation_m": max_abs_y,
        "minimum_outer_wall_body_clearance_m": (
            outer_wall_body_clearance
        ),
        "first_gate_minimum_y_m": first_gate_min_y,
        "second_gate_maximum_y_m": second_gate_max_y,
        "maximum_motor_rpm": metrics.get(
            "maximum_motor_rpm"
        ),
    }

    (result_dir / "corridor_metrics.json").write_text(
        json.dumps(
            corridor_metrics,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    def value_text(value, unit="", digits=3):
        if not finite(value):
            return "N/A"
        return f"{float(value):.{digits}f}{unit}"

    lines = [
        "===== Narrow S-Corridor Evaluation Summary =====",
        f"Overall pass: {overall_pass}",
        (
            "Arrival time: "
            + value_text(
                metrics.get("arrival_time_s"),
                " s",
            )
        ),
        (
            "Final error: "
            + value_text(
                metrics.get("final_error_m"),
                " m",
                4,
            )
        ),
        (
            "Planned path length: "
            + value_text(
                metrics.get("planned_path_length_m"),
                " m",
            )
        ),
        (
            "Actual path length: "
            + value_text(
                metrics.get("actual_path_length_m"),
                " m",
            )
        ),
        (
            "Minimum physical obstacle clearance: "
            + value_text(
                minimum_obstacle_clearance,
                " m",
            )
        ),
        (
            "Minimum safety-zone clearance: "
            + value_text(
                minimum_safety_clearance,
                " m",
            )
        ),
        (
            "Maximum |y| inside corridor: "
            + value_text(max_abs_y, " m")
        ),
        (
            "Minimum outer-wall body clearance: "
            + value_text(
                outer_wall_body_clearance,
                " m",
            )
        ),
        (
            "First gate minimum y: "
            + value_text(first_gate_min_y, " m")
        ),
        (
            "Second gate maximum y: "
            + value_text(second_gate_max_y, " m")
        ),
        "",
        "Acceptance criteria:",
    ]

    for key, passed in criteria.items():
        lines.append(
            f"- {key}: {'PASS' if passed else 'FAIL'}"
        )

    summary_text = "\n".join(lines) + "\n"
    (result_dir / "corridor_summary.txt").write_text(
        summary_text,
        encoding="utf-8",
    )

    x_values = [row["x_m"] for row in rows]
    y_values = [row["y_m"] for row in rows]

    plt.figure(figsize=(10, 5.5))
    plt.plot(
        x_values,
        y_values,
        label="actual trajectory",
    )
    plt.axhline(
        OUTER_WALL_INNER_Y,
        linestyle="--",
        label="physical corridor boundary",
    )
    plt.axhline(
        -OUTER_WALL_INNER_Y,
        linestyle="--",
    )
    plt.axvspan(
        FIRST_GATE_X_MIN,
        FIRST_GATE_X_MAX,
        alpha=0.12,
        label="upper gate region",
    )
    plt.axvspan(
        SECOND_GATE_X_MIN,
        SECOND_GATE_X_MAX,
        alpha=0.12,
        label="lower gate region",
    )
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Narrow S-Corridor Trajectory")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        result_dir / "corridor_analysis.png",
        dpi=180,
    )
    plt.close()

    print(f"Latest corridor result: {result_dir}")
    print()
    print(summary_text)


if __name__ == "__main__":
    main()
