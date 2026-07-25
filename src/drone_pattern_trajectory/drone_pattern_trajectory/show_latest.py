#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


def main() -> None:
    root = Path.home() / "drone_task" / "your_drone_sim_ws" / "results"
    candidates = list((root / "pattern_trajectory").glob("*/summary.json"))
    candidates += list(root.glob("pattern_trajectory_suite_*/summary.json"))
    if not candidates:
        raise SystemExit("No pattern trajectory summary found")
    latest = max(candidates, key=lambda path: path.stat().st_mtime)
    print(f"Result: {latest}")
    print(json.dumps(json.loads(latest.read_text(encoding="utf-8")), indent=2, ensure_ascii=False))
