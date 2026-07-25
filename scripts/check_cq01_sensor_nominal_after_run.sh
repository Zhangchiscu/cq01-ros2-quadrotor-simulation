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
  echo "[FAIL] 找不到 nominal_nominal_closed_loop_full 结果"
  exit 1
fi

python3 - "$LATEST" <<'PY'
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
summary_path = root / 'bonus_summary.json'
manifest_path = root / 'experiment_manifest.json'
telemetry_path = root / 'telemetry.csv'

errors = []
for path in (
    summary_path,
    manifest_path,
    telemetry_path,
    root / 'position_error.png',
    root / 'altitude.png',
    root / 'maximum_rpm.png',
    root / 'body_clearance.png',
):
    if not path.is_file() or path.stat().st_size == 0:
        errors.append(f'missing_or_empty:{path.name}')

if errors:
    for error in errors:
        print('[FAIL]', error)
    raise SystemExit(1)

summary = json.loads(
    summary_path.read_text(encoding='utf-8')
)
manifest = json.loads(
    manifest_path.read_text(encoding='utf-8')
)

with telemetry_path.open(
    'r',
    encoding='utf-8',
    newline='',
) as handle:
    rows = list(csv.DictReader(handle))

metrics = summary.get('metrics', {})
checks = {
    'summary_pass':
        summary.get('pass') is True,
    'failed_criteria_empty':
        summary.get('failed_criteria') in ([], None),
    'sensor_profile_correct':
        summary.get('sensor_profile')
        == 'nominal_closed_loop',
    'control_state_source_correct':
        summary.get('control_state_source')
        == '/cq01_v9/sensors/nominal/odom',
    'manifest_control_source_correct':
        manifest.get('control_state_source')
        == '/cq01_v9/sensors/nominal/odom',
    'sensor_chain_verified':
        summary.get(
            'checks',
            {},
        ).get('sensor_chain_verified') is True,
    'route_completed':
        summary.get(
            'checks',
            {},
        ).get('route_P0_P15_completed') is True,
    'pattern_passed':
        summary.get(
            'checks',
            {},
        ).get('summit_pattern_passed') is True,
    'telemetry_sufficient':
        len(rows) >= 1000,
    'sensor_rmse_valid': (
        isinstance(
            metrics.get(
                'nominal_sensor_position_rmse_m'
            ),
            (int, float),
        )
        and 0.0
        < float(
            metrics['nominal_sensor_position_rmse_m']
        )
        < 0.15
    ),
    'final_error_valid': (
        isinstance(
            metrics.get('final_position_error_m'),
            (int, float),
        )
        and float(
            metrics['final_position_error_m']
        ) <= 0.30
    ),
    'rpm_not_saturated': (
        isinstance(
            metrics.get('maximum_rpm'),
            (int, float),
        )
        and float(metrics['maximum_rpm']) < 9800.0
        and metrics.get(
            'rpm_saturation_samples'
        ) == 0
    ),
}

for name, passed in checks.items():
    print(f'[{"PASS" if passed else "FAIL"}] {name}')

failed = [
    name
    for name, passed in checks.items()
    if not passed
]

print('Metrics:')
for key, value in metrics.items():
    print(f'  {key}: {value}')

print(
    'SENSOR NOMINAL CLOSED-LOOP AUDIT: '
    f'pass={not failed}; failed={failed}'
)
print('Result directory:', root)

raise SystemExit(0 if not failed else 1)
PY
