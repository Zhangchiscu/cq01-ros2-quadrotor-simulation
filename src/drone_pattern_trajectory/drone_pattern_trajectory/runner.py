#!/usr/bin/env python3
"""Run circle and figure-eight validation sequentially."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional

PATTERNS = ("circle", "figure_eight")


def result_dirs(root: Path) -> List[Path]:
    return [path for path in root.iterdir() if path.is_dir()] if root.exists() else []


def find_new_result(root: Path, known: List[Path], pattern: str, started: float) -> Optional[Path]:
    known_set = {path.resolve() for path in known}
    candidates = []
    for path in result_dirs(root):
        if path.resolve() in known_set:
            continue
        if not path.name.startswith(f"{pattern}_"):
            continue
        if path.stat().st_mtime < started - 2.0:
            continue
        if (path / "summary.json").exists():
            candidates.append(path)
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def stop_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=4.0)


def run_one(pattern: str, root: Path, use_rviz: bool, timeout: float) -> Path:
    known = result_dirs(root)
    started = time.time()
    command = [
        "ros2", "launch", "drone_pattern_trajectory", "pattern_mission.launch.py",
        f"pattern:={pattern}", f"use_rviz:={'true' if use_rviz else 'false'}",
    ]
    process = subprocess.Popen(command, start_new_session=True)
    result = None
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            result = find_new_result(root, known, pattern, started)
            if result is not None:
                break
            if process.poll() is not None:
                time.sleep(0.5)
                result = find_new_result(root, known, pattern, started)
                break
            time.sleep(0.5)
    finally:
        stop_process_group(process)
    if result is None:
        raise RuntimeError(f"{pattern} produced no summary")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--timeout", type=float, default=65.0)
    args = parser.parse_args()

    workspace = Path.home() / "drone_task" / "your_drone_sim_ws"
    root = workspace / "results" / "pattern_trajectory"
    root.mkdir(parents=True, exist_ok=True)
    output = workspace / "results" / f"pattern_trajectory_suite_{time.strftime('%Y%m%d_%H%M%S')}"
    output.mkdir(parents=True, exist_ok=False)

    records: List[Dict[str, object]] = []
    sources: Dict[str, str] = {}
    for pattern in PATTERNS:
        print(f"\n===== Running {pattern} =====")
        result = run_one(pattern, root, args.rviz, args.timeout)
        summary = json.loads((result / "summary.json").read_text(encoding="utf-8"))
        records.append(summary)
        sources[pattern] = str(result / "summary.json")
        print(
            f"{pattern}: pass={summary['pass']} "
            f"rms={summary.get('rms_tracking_error_m')} "
            f"max={summary.get('maximum_tracking_error_m')}"
        )

    criteria = {
        "circle_pass": bool(records[0].get("pass", False)),
        "figure_eight_pass": bool(records[1].get("pass", False)),
    }
    suite = {
        "experiment": "circle_and_figure_eight_pattern_suite",
        "sources": sources,
        "patterns": records,
        "criteria": criteria,
        "pass": all(criteria.values()),
    }
    (output / "summary.json").write_text(
        json.dumps(suite, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(suite, indent=2, ensure_ascii=False))
    raise SystemExit(0 if suite["pass"] else 2)
