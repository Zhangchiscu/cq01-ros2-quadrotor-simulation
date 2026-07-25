#!/usr/bin/env python3

from pathlib import Path


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
    )
    candidates = sorted(
        (
            path
            for path in root.glob("robustness_noise_*")
            if path.is_dir()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not candidates:
        print("No robustness result found.")
        return

    latest = candidates[0]
    print(f"Latest robustness result: {latest}")

    summary = latest / "robustness_summary.txt"
    if summary.exists():
        print()
        print(summary.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
