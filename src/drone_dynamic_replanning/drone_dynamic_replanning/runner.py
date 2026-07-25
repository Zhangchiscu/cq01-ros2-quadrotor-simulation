#!/usr/bin/env python3
"""Run one dynamic replanning mission and clean up the launch process."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional


def result_directories(root: Path) -> List[Path]:
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
    started: float,
) -> Optional[Path]:
    known_set = {
        path.resolve()
        for path in known
    }
    candidates = []

    for path in result_directories(root):
        if path.resolve() in known_set:
            continue
        if not path.name.startswith(
            "dynamic_replanning_"
        ):
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


def stop_process_group(
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rviz",
        action="store_true",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=90.0,
    )
    arguments = parser.parse_args()

    workspace = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )
    results_root = (
        workspace
        / "results"
        / "dynamic_replanning_raw"
    )
    results_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    known = result_directories(results_root)
    started = time.time()

    command = [
        "ros2",
        "launch",
        "drone_dynamic_replanning",
        "dynamic_replanning.launch.py",
        f"use_rviz:={'true' if arguments.rviz else 'false'}",
    ]

    process = subprocess.Popen(
        command,
        start_new_session=True,
    )
    result = None
    deadline = time.monotonic() + arguments.timeout

    try:
        while time.monotonic() < deadline:
            result = find_new_result(
                results_root,
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
            "Dynamic replanning mission produced no result."
        )

    metrics = json.loads(
        (result / "metrics.json").read_text(
            encoding="utf-8"
        )
    )
    print()
    print(
        (result / "summary.txt").read_text(
            encoding="utf-8"
        )
    )
    print(f"Results directory: {result}")

    if not bool(metrics["overall_pass"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
