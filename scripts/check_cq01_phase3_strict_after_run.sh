#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/drone_task/your_drone_sim_ws/results/cq01_bonus_suite"

LATEST="$(
  find "$ROOT" \
    -maxdepth 1 \
    -mindepth 1 \
    -type d \
    -name 'nominal_nominal_closed_loop_full_*' \
    -printf '%T@ %p\n' \
  | sort -nr \
  | head -n 1 \
  | cut -d' ' -f2-
)"

if [ -z "${LATEST:-}" ]; then
  echo "[FAIL] 找不到 nominal_nominal_closed_loop_full 结果目录"
  exit 1
fi

python3 - "$LATEST" <<'PYCODE'
from __future__ import annotations

import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
summary_path = root / "bonus_summary.json"
manifest_path = root / "experiment_manifest.json"

if not summary_path.is_file():
    print("[FAIL] 缺少 bonus_summary.json")
    raise SystemExit(1)

summary = json.loads(summary_path.read_text(encoding="utf-8"))
manifest = (
    json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest_path.is_file()
    else {}
)

checks = summary.get("checks", {})
metrics = summary.get("metrics", {})
modules = summary.get("module_status", {})

strict = {
    "overall_summary_pass":
        summary.get("pass") is True,
    "failed_criteria_empty":
        summary.get("failed_criteria") in ([], None),
    "profile_verified":
        checks.get("profile_verified") is True,
    "sensor_chain_verified":
        checks.get("sensor_chain_verified") is True,
    "sensor_metadata_matches_launch":
        checks.get("sensor_status_source_matches_launch") is True,
    "h14_retracking_passed":
        checks.get("h14_high_wind_replan_retracking") is True,
    "route_completed":
        checks.get("route_P0_P15_completed") is True,
    "pattern_passed":
        checks.get("summit_pattern_passed") is True,
    "control_state_source_correct":
        summary.get("control_state_source")
        == "/cq01_v9/sensors/nominal/odom",
    "manifest_source_correct":
        manifest.get("control_state_source")
        == "/cq01_v9/sensors/nominal/odom",
    "sensor_rmse_valid": (
        isinstance(
            metrics.get("nominal_sensor_position_rmse_m"),
            (int, float),
        )
        and 0.0
        < float(metrics["nominal_sensor_position_rmse_m"])
        < 0.15
    ),
    "final_error_valid": (
        isinstance(
            metrics.get("final_position_error_m"),
            (int, float),
        )
        and float(metrics["final_position_error_m"]) <= 0.30
    ),
    "rpm_not_saturated": (
        isinstance(metrics.get("maximum_rpm"), (int, float))
        and float(metrics["maximum_rpm"]) < 9800.0
        and metrics.get("rpm_saturation_samples") == 0
    ),
    "h14_status_pass": (
        modules.get("h14", {}).get("pass_result") is True
        and modules.get("h14", {}).get("tracking_timeout") is False
    ),
    "sensor_suite_source_correct": (
        modules.get("sensors", {}).get("control_state_source")
        == "/cq01_v9/sensors/nominal/odom"
    ),
}

failed = [
    name
    for name, passed in strict.items()
    if passed is not True
]

print("=" * 72)
print("CQ01 PHASE-3 STRICT AUDIT V1.3")
print("Result directory:", root)
for name, passed in strict.items():
    print(f'[{"PASS" if passed else "FAIL"}] {name}')

print()
print("Summary failed_criteria:", summary.get("failed_criteria"))
print("Metrics:")
for name, value in metrics.items():
    print(f"  {name}: {value}")

print()
print(
    "PHASE-3 STRICT RESULT: "
    f"pass={not failed}; failed={failed}"
)
print("=" * 72)

raise SystemExit(0 if not failed else 1)
PYCODE
