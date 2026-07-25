#!/usr/bin/env python3
"""Show or recover the latest dynamic-replanning robustness report."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def fmt(
    value: object,
    digits: int = 3,
    missing: str = "N/A",
) -> str:
    if not finite(value):
        return missing
    return f"{float(value):.{digits}f}"


def render(data: Dict[str, Any]) -> str:
    records = list(data.get("records", []))
    criteria = dict(data.get("criteria", {}))

    lines = [
        "===== Dynamic Replanning Robustness Regression =====",
        f"Overall pass: {data.get('overall_pass')}",
        f"Run count: {data.get('run_count', len(records))}",
        f"Pass count: {data.get('pass_count', 0)}",
        (
            "Success rate: "
            f"{fmt(100.0 * float(data.get('success_rate', 0.0)), 1)}%"
        ),
        (
            "Mean arrival time: "
            f"{fmt(data.get('mean_arrival_time_s'))} s"
        ),
        (
            "Worst dynamic clearance: "
            f"{fmt(data.get('worst_dynamic_clearance_m'))} m"
        ),
        (
            "Worst static clearance: "
            f"{fmt(data.get('worst_static_clearance_m'))} m"
        ),
        (
            "Maximum replan latency: "
            f"{fmt(data.get('maximum_replan_latency_ms'))} ms"
        ),
        "",
    ]

    for record in records:
        lines.extend([
            (
                f"[{record.get('case_name')} "
                f"run {record.get('repeat_index')}]"
            ),
            f"Overall pass: {record.get('overall_pass')}",
            f"Mission success: {record.get('success')}",
            f"Failure reason: {record.get('failure_reason') or 'none'}",
            (
                "Arrival time: "
                f"{fmt(record.get('arrival_time_s'))} s"
            ),
            (
                "Final error: "
                f"{fmt(record.get('final_error_m'), 4)} m"
            ),
            (
                "Minimum dynamic clearance: "
                f"{fmt(record.get('minimum_dynamic_clearance_m'))} m"
            ),
            (
                "Minimum static clearance: "
                f"{fmt(record.get('minimum_static_clearance_m'))} m"
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

    return "\n".join(lines)


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
    )
    candidates = sorted(
        [
            path
            for path in root.glob("dynamic_robustness_*")
            if path.is_dir()
        ],
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("No dynamic robustness regression found.")
        return

    latest = candidates[0]
    print(f"Latest result: {latest}")

    summary = latest / "dynamic_robustness_summary.txt"
    if summary.exists():
        print()
        print(summary.read_text(encoding="utf-8"))
        return

    json_path = latest / "dynamic_robustness_summary.json"
    if not json_path.exists():
        print(
            "The result directory is incomplete: "
            "neither summary text nor summary JSON exists."
        )
        return

    data = json.loads(json_path.read_text(encoding="utf-8"))
    recovered = render(data)
    summary.write_text(recovered + "\n", encoding="utf-8")

    print()
    print("[recovered report from dynamic_robustness_summary.json]")
    print(recovered)


if __name__ == "__main__":
    main()
