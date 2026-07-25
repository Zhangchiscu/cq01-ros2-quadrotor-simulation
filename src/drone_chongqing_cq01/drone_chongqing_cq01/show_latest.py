#!/usr/bin/env python3
"""Print the latest CQ-01 result."""

from pathlib import Path
import json


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "chongqing_cq01"
    )
    results = sorted(
        root.glob("cq01_eling_liziba_*/summary.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not results:
        raise SystemExit("No CQ-01 result found")
    path = results[0]
    print(path)
    print(json.dumps(
        json.loads(path.read_text(encoding="utf-8")),
        ensure_ascii=False,
        indent=2,
    ))


if __name__ == "__main__":
    main()
