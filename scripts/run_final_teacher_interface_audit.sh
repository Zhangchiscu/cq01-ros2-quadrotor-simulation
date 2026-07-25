#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/drone_task/your_drone_sim_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$WS/results/teacher_interface_final_audit_$STAMP"
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

LOG="$OUT/teacher_interface_audit.txt"
SUMMARY="$OUT/teacher_interface_summary.json"

echo "========================================================================" | tee "$LOG"
echo "CQ01 FINAL TEACHER INTERFACE AUDIT" | tee -a "$LOG"
echo "========================================================================" | tee -a "$LOG"

if [ ! -x "$WS/scripts/check_cq01_teacher_interfaces.sh" ]; then
  echo "[FAIL] existing checker missing: $WS/scripts/check_cq01_teacher_interfaces.sh" | tee -a "$LOG"
  python3 - "$SUMMARY" <<'PY'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(
    json.dumps(
        {
            "pass": False,
            "reason": "existing checker missing",
        },
        indent=2,
    ),
    encoding="utf-8",
)
PY
  exit 2
fi

set +e
"$WS/scripts/check_cq01_teacher_interfaces.sh" \
  2>&1 | tee -a "$LOG"
RC=${PIPESTATUS[0]}
set -e

python3 - "$LOG" "$SUMMARY" "$RC" <<'PY'
from pathlib import Path
import json
import re
import sys

log = Path(sys.argv[1]).read_text(
    encoding="utf-8",
    errors="replace",
)
summary = Path(sys.argv[2])
return_code = int(sys.argv[3])

patterns = [
    r"TEACHER INTERFACE RESULT:\s*pass=True",
    r"RESULT:\s*pass=True",
    r"pass=True;\s*failed=\[\]",
]

passed = any(
    re.search(pattern, log)
    for pattern in patterns
)

# Do not invent a pass merely from exit code 0. The checker must explicitly
# report pass=True / failed=[].
payload = {
    "pass": passed,
    "checker_return_code": return_code,
    "explicit_pass_marker_found": passed,
    "log_file": str(Path(sys.argv[1])),
}

summary.write_text(
    json.dumps(
        payload,
        ensure_ascii=False,
        indent=2,
    ),
    encoding="utf-8",
)

print()
print("=" * 72)
print(
    "FINAL TEACHER INTERFACE EVIDENCE:",
    "PASS" if passed else "FAIL/UNVERIFIED",
)
print("summary =", summary)
print("=" * 72)

raise SystemExit(0 if passed else 1)
PY
