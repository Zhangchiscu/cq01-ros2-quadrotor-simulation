#!/usr/bin/env python3

import json
from pathlib import Path


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "sensor_noise"
    )
    summaries = sorted(
        root.glob("sensor_*/summary.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not summaries:
        raise SystemExit(f"No sensor summary found below {root}")
    latest = summaries[0]
    print(f"Result: {latest}")
    print(json.dumps(json.loads(latest.read_text(encoding="utf-8")), indent=2))


if __name__ == "__main__":
    main()
