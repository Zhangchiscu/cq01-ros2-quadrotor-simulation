#!/usr/bin/env python3
"""Show latest raw versus smoothed trajectory comparison."""

from pathlib import Path


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
            for path in root.glob("trajectory_comparison_*")
            if path.is_dir()
        ],
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("No trajectory comparison found.")
        return

    latest = candidates[0]
    print(f"Latest result: {latest}")
    summary = latest / "trajectory_comparison.txt"
    if summary.exists():
        print()
        print(summary.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
