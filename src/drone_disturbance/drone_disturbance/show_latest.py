#!/usr/bin/env python3

import json
from pathlib import Path


def main() -> None:
    root = (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "disturbance_recovery"
    )
    summaries = sorted(root.glob("disturbance_*/summary.json"))
    if not summaries:
        print(f"No disturbance results found under: {root}")
        return

    latest = summaries[-1]
    data = json.loads(latest.read_text(encoding="utf-8"))
    print(f"Result: {latest}")
    print(json.dumps(data, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
