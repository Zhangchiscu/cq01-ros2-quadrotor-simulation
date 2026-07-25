#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def newest_summary(root: Path, profile: str, after_mtime: float) -> Path:
    candidates = [
        path
        for path in root.glob(f"sensor_{profile}_*/summary.json")
        if path.stat().st_mtime >= after_mtime
    ]
    if not candidates:
        raise RuntimeError(f"No new {profile} summary was produced")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def run_profile(workspace: Path, profile: str, root: Path) -> Path:
    before = datetime.now().timestamp()
    command = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        f"ros2 launch drone_sensor_sim sensor_noise_experiment.launch.py "
        f"profile:={profile} use_rviz:=false"
    )
    result = subprocess.run(["bash", "-lc", command], check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f"Profile {profile} launch exited with code {result.returncode}"
        )
    return newest_summary(root, profile, before)


def main() -> None:
    workspace = Path(
        os.environ.get(
            "DRONE_WS",
            str(Path.home() / "drone_task" / "your_drone_sim_ws"),
        )
    )
    root = workspace / "results" / "sensor_noise"
    root.mkdir(parents=True, exist_ok=True)

    clean_path = run_profile(workspace, "clean", root)
    nominal_path = run_profile(workspace, "nominal", root)
    clean = json.loads(clean_path.read_text(encoding="utf-8"))
    nominal = json.loads(nominal_path.read_text(encoding="utf-8"))

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = workspace / "results" / "sensor_comparison" / f"comparison_{timestamp}"
    output.mkdir(parents=True, exist_ok=True)

    summary = {
        "experiment": "clean_vs_nominal_sensor_comparison",
        "clean_summary": str(clean_path),
        "nominal_summary": str(nominal_path),
        "clean_pass": bool(clean.get("pass", False)),
        "nominal_pass": bool(nominal.get("pass", False)),
        "clean_final_error_m": clean.get("final_truth_position_error_m"),
        "nominal_final_error_m": nominal.get("final_truth_position_error_m"),
        "nominal_odom_position_rmse_m": nominal.get("odom_position_rmse_m"),
        "nominal_imu_acceleration_rmse_mps2": nominal.get(
            "imu_acceleration_rmse_mps2"
        ),
        "pass": bool(clean.get("pass", False) and nominal.get("pass", False)),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )

    metrics = [
        clean.get("final_truth_position_error_m", 0.0),
        nominal.get("final_truth_position_error_m", 0.0),
    ]
    plt.figure(figsize=(7, 5))
    plt.bar(["Clean", "Nominal noise"], metrics)
    plt.ylabel("Final truth position error [m]")
    plt.grid(axis="y")
    plt.tight_layout()
    plt.savefig(output / "final_error_comparison.png", dpi=160)
    plt.close()

    print(f"Comparison result: {output / 'summary.json'}")
    print(json.dumps(summary, indent=2))
    if not summary["pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
