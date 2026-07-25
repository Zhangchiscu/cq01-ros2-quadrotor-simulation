#!/usr/bin/env python3

from pathlib import Path


def main() -> None:
    results_root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
    )

    directories = sorted(
        (
            path
            for path in results_root.glob(
                "repeatability_standard_*"
            )
            if path.is_dir()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not directories:
        print(
            "No standard repeatability result directory found."
        )
        return

    latest = directories[0]
    summary_path = latest / "summary.txt"

    print(f"Latest repeatability result: {latest}")

    if summary_path.exists():
        print()
        print(summary_path.read_text(encoding="utf-8"))
    else:
        print("summary.txt is missing.")


if __name__ == "__main__":
    main()
