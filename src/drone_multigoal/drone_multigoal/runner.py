#!/usr/bin/env python3

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List


def result_directories(
    root: Path,
) -> List[Path]:
    return [
        path
        for path in root.glob("multigoal_*")
        if path.is_dir()
    ]


def newest_new_result(
    root: Path,
    known: List[Path],
    start_time: float,
):
    known_resolved = {
        path.resolve()
        for path in known
    }
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_resolved:
            continue
        if path.stat().st_mtime < start_time - 2.0:
            continue
        if not (path / "mission_metrics.json").exists():
            continue
        candidates.append(path)

    if not candidates:
        return None

    return max(
        candidates,
        key=lambda path: path.stat().st_mtime,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Enable RViz during the mission",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=190.0,
    )
    arguments = parser.parse_args()

    results_root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "multigoal"
    )
    results_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    known = result_directories(results_root)
    start_time = time.time()

    command = [
        "ros2",
        "launch",
        "drone_multigoal",
        "multi_goal_evaluation.launch.py",
        f"use_rviz:={'true' if arguments.rviz else 'false'}",
    ]

    print("Starting continuous multi-goal mission")
    print(f"RViz enabled: {arguments.rviz}")

    process = subprocess.Popen(
        command,
        start_new_session=True,
    )

    result_dir = None
    deadline = time.monotonic() + arguments.timeout

    try:
        while time.monotonic() < deadline:
            result_dir = newest_new_result(
                results_root,
                known,
                start_time,
            )
            if result_dir is not None:
                break

            if process.poll() is not None:
                break

            time.sleep(0.5)
    finally:
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

    if result_dir is None:
        raise RuntimeError(
            "No multi-goal result was generated."
        )

    summary_path = result_dir / "mission_summary.txt"
    metrics_path = result_dir / "mission_metrics.json"

    print()
    print(f"Results directory: {result_dir}")
    print()

    if summary_path.exists():
        print(
            summary_path.read_text(
                encoding="utf-8"
            )
        )

    metrics = json.loads(
        metrics_path.read_text(encoding="utf-8")
    )

    if not bool(metrics.get("overall_pass")):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
