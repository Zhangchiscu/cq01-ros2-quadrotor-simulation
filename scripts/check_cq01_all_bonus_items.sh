#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
ROOT="$WS/results/cq01_bonus_suite"

"$WS/scripts/generate_cq01_final_bonus_package.py"

LATEST="$(
  find "$ROOT" \
    -maxdepth 1 \
    -mindepth 1 \
    -type d \
    -name 'cq01_final_bonus_package_*' \
    -printf '%T@ %p\n' \
  | sort -nr \
  | head -n 1 \
  | cut -d' ' -f2-
)"

if [ -z "${LATEST:-}" ]; then
  echo "[FAIL] 没有找到最终加分包"
  exit 1
fi

python3 - "$LATEST/final_bonus_summary.json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
payload = json.loads(path.read_text(encoding="utf-8"))

print("=" * 72)
print("CQ01 ALL BONUS ITEMS AUDIT")
print("Summary:", path)

for name, passed in payload.get("checks", {}).items():
    state = (
        "EXCLUDED"
        if name == "multi_uav_excluded_by_user_decision"
        else ("PASS" if passed else "FAIL")
    )
    print(f"[{state}] {name}")

print()
print(
    "ALL BONUS ITEMS RESULT: "
    f'pass={payload.get("pass")}; '
    f'failed={payload.get("failed_criteria")}'
)
print("=" * 72)

raise SystemExit(0 if payload.get("pass") is True else 1)
PY
