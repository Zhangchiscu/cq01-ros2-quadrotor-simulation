#!/usr/bin/env python3
"""Show the latest multi-scenario regression summary."""

from pathlib import Path


def main() -> None:
    results_root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
    )

    candidates = sorted(
        [
            path
            for path in results_root.glob(
                "world_regression_*"
            )
            if path.is_dir()
        ],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("No world regression result found.")
        return

    latest = candidates[0]
    print(f"Latest result: {latest}")

    summary = latest / "world_regression_summary.txt"
    if summary.exists():
        print()
        print(summary.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
