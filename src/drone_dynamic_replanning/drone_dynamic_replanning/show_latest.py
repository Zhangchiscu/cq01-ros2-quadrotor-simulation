#!/usr/bin/env python3
"""Show the latest dynamic replanning result."""

from pathlib import Path


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "dynamic_replanning_raw"
    )
    candidates = sorted(
        [
            path
            for path in root.glob(
                "dynamic_replanning_*"
            )
            if path.is_dir()
        ],
        key=lambda item: item.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("No dynamic replanning result found.")
        return

    latest = candidates[0]
    print(f"Latest result: {latest}")
    summary = latest / "summary.txt"
    if summary.exists():
        print()
        print(summary.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
