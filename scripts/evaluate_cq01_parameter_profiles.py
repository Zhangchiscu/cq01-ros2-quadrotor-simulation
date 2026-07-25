#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


Point = Tuple[float, float]


def latest_directory(root: Path, prefix: str) -> Path:
    candidates = sorted(
        (
            path
            for path in root.iterdir()
            if path.is_dir()
            and path.name.startswith(prefix)
            and (path / 'bonus_summary.json').is_file()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(
            f'No result directory matching {prefix} in {root}'
        )
    return candidates[0]


def load_json(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def read_telemetry(path: Path) -> List[Dict[str, str]]:
    with path.open(
        'r',
        encoding='utf-8',
        newline='',
    ) as handle:
        return list(csv.DictReader(handle))


def numeric(row: Dict[str, str], key: str) -> float:
    value = row.get(key, '')
    return float(value) if value not in ('', None) else float('nan')


def stable_hover_rpm(rows: List[Dict[str, str]]) -> float:
    tail = rows[-500:]
    candidates: List[float] = []

    for row in tail:
        speed = numeric(row, 'speed_mps')
        error = numeric(row, 'position_error_m')
        rpms = [
            numeric(row, f'rpm_{index}')
            for index in range(1, 5)
        ]
        if (
            math.isfinite(speed)
            and math.isfinite(error)
            and speed <= 0.20
            and error <= 0.10
            and all(math.isfinite(value) for value in rpms)
        ):
            candidates.extend(rpms)

    if len(candidates) < 20:
        candidates = []
        for row in rows[-50:]:
            for index in range(1, 5):
                value = numeric(row, f'rpm_{index}')
                if math.isfinite(value):
                    candidates.append(value)

    if not candidates:
        raise ValueError('No RPM samples available')
    return float(statistics.median(candidates))


def normalized_altitude(
    rows: List[Dict[str, str]],
) -> Tuple[List[float], List[float]]:
    if not rows:
        return [], []
    total = max(
        numeric(rows[-1], 'time_s'),
        1.0,
    )
    time_values: List[float] = []
    altitude_values: List[float] = []
    for row in rows:
        time_s = numeric(row, 'time_s')
        z_m = numeric(row, 'z_m')
        if math.isfinite(time_s) and math.isfinite(z_m):
            time_values.append(time_s / total)
            altitude_values.append(z_m)
    return time_values, altitude_values


def all_strings(value: Any) -> Iterable[str]:
    if isinstance(value, dict):
        for child in value.values():
            yield from all_strings(child)
    elif isinstance(value, list):
        for child in value:
            yield from all_strings(child)
    elif isinstance(value, str):
        yield value


def close_ratio(value: float, target: float) -> bool:
    return abs(value - target) <= 0.03


def main() -> int:
    workspace = Path.home() / 'drone_task/your_drone_sim_ws'
    result_root = workspace / 'results/cq01_bonus_suite'
    config_root = workspace / 'src/cq01_v9_mc/config'

    nominal_dir = latest_directory(
        result_root,
        'nominal_full_',
    )
    heavy_dir = latest_directory(
        result_root,
        'heavy_full_',
    )

    nominal_summary = load_json(
        nominal_dir / 'bonus_summary.json'
    )
    heavy_summary = load_json(
        heavy_dir / 'bonus_summary.json'
    )
    nominal_manifest = load_json(
        nominal_dir / 'experiment_manifest.json'
    )
    heavy_manifest = load_json(
        heavy_dir / 'experiment_manifest.json'
    )
    nominal_rows = read_telemetry(
        nominal_dir / 'telemetry.csv'
    )
    heavy_rows = read_telemetry(
        heavy_dir / 'telemetry.csv'
    )

    nominal_config = load_yaml(
        config_root / 'vehicle_nominal.yaml'
    )
    heavy_config = load_yaml(
        config_root / 'vehicle_heavy.yaml'
    )

    nominal_parameters = (
        nominal_config['quadrotor_dynamics']
        ['ros__parameters']
    )
    heavy_parameters = (
        heavy_config['quadrotor_dynamics']
        ['ros__parameters']
    )

    mass_ratio = (
        float(heavy_parameters['mass'])
        / float(nominal_parameters['mass'])
    )
    inertia_ratios = {
        key: (
            float(heavy_parameters[key])
            / float(nominal_parameters[key])
        )
        for key in (
            'inertia_xx',
            'inertia_yy',
            'inertia_zz',
        )
    }

    nominal_hover = stable_hover_rpm(nominal_rows)
    heavy_hover = stable_hover_rpm(heavy_rows)
    measured_rpm_ratio = heavy_hover / nominal_hover
    expected_rpm_ratio = math.sqrt(mass_ratio)

    heavy_strings = set(
        all_strings(
            heavy_summary.get('module_status', {})
        )
    )
    required_states = {
        'mission_completed',
        'recovered',
        'new_path_received',
        'results_saved',
        'high_altitude_result_saved',
        'dynamic_obstacle_completed',
        'pattern_completed',
        'integrated_completed',
    }

    nominal_metrics = nominal_summary.get('metrics', {})
    heavy_metrics = heavy_summary.get('metrics', {})
    nominal_checks = nominal_summary.get('checks', {})
    heavy_checks = heavy_summary.get('checks', {})

    checks = {
        'nominal_profile_manifest':
            nominal_manifest.get('vehicle_profile') == 'nominal',
        'heavy_profile_manifest':
            heavy_manifest.get('vehicle_profile') == 'heavy',
        'mass_ratio_1p20':
            math.isclose(
                mass_ratio,
                1.20,
                rel_tol=1.0e-9,
                abs_tol=1.0e-9,
            ),
        'inertia_ratios_1p20':
            all(
                math.isclose(
                    ratio,
                    1.20,
                    rel_tol=1.0e-9,
                    abs_tol=1.0e-9,
                )
                for ratio in inertia_ratios.values()
            ),
        'nominal_route_completed':
            nominal_checks.get(
                'route_P0_P15_completed'
            ) is True,
        'heavy_route_completed':
            heavy_checks.get(
                'route_P0_P15_completed'
            ) is True,
        'nominal_pattern_passed':
            nominal_checks.get(
                'summit_pattern_passed'
            ) is True,
        'heavy_pattern_passed':
            heavy_checks.get(
                'summit_pattern_passed'
            ) is True,
        'heavy_all_closed_loop_stages_executed':
            required_states.issubset(heavy_strings),
        'nominal_final_error':
            float(
                nominal_metrics.get(
                    'final_position_error_m',
                    float('inf'),
                )
            ) <= 0.30,
        'heavy_final_error':
            float(
                heavy_metrics.get(
                    'final_position_error_m',
                    float('inf'),
                )
            ) <= 0.30,
        'nominal_rpm_not_saturated':
            float(
                nominal_metrics.get(
                    'maximum_rpm',
                    float('inf'),
                )
            ) < 9800.0,
        'heavy_rpm_not_saturated':
            float(
                heavy_metrics.get(
                    'maximum_rpm',
                    float('inf'),
                )
            ) < 9800.0,
        'steady_hover_rpm_increased':
            heavy_hover > nominal_hover,
        'steady_hover_rpm_ratio_matches_theory':
            close_ratio(
                measured_rpm_ratio,
                expected_rpm_ratio,
            ),
        'nominal_telemetry_sufficient':
            len(nominal_rows) >= 1000,
        'heavy_telemetry_sufficient':
            len(heavy_rows) >= 1000,
    }

    failed = [
        name
        for name, passed in checks.items()
        if not passed
    ]

    profile_observations = {
        'heavy_original_bonus_summary_pass':
            heavy_summary.get('pass'),
        'heavy_original_bonus_failed_criteria':
            heavy_summary.get('failed_criteria'),
        'heavy_low_strong_module':
            heavy_summary.get(
                'module_status',
                {},
            ).get('low_strong'),
        'heavy_h14_module':
            heavy_summary.get(
                'module_status',
                {},
            ).get('h14'),
        'interpretation': (
            'The heavy-profile experiment evaluates configuration '
            'loading, closed-loop stability, route completion, '
            'pattern completion and actuator margin. Nominal-tuned '
            'disturbance displacement thresholds are preserved as '
            'separate profile-dependent observations and are not '
            'rewritten.'
        ),
    }

    output_dir = (
        result_root
        / (
            'parameter_profile_comparison_'
            + datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )
        )
    )
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    summary = {
        'pass': not failed,
        'failed_criteria': failed,
        'experiment':
            'cq01_nominal_vs_heavy_parameterization',
        'evaluation_method':
            'offline_config_plus_closed_loop_differential_evidence',
        'nominal_result_dir': str(nominal_dir),
        'heavy_result_dir': str(heavy_dir),
        'checks': checks,
        'configuration': {
            'nominal_mass_kg':
                float(nominal_parameters['mass']),
            'heavy_mass_kg':
                float(heavy_parameters['mass']),
            'mass_ratio': mass_ratio,
            'inertia_ratios': inertia_ratios,
        },
        'physical_evidence': {
            'nominal_stable_hover_rpm': nominal_hover,
            'heavy_stable_hover_rpm': heavy_hover,
            'measured_rpm_ratio': measured_rpm_ratio,
            'expected_sqrt_mass_ratio':
                expected_rpm_ratio,
            'rpm_ratio_absolute_error':
                abs(
                    measured_rpm_ratio
                    - expected_rpm_ratio
                ),
        },
        'performance': {
            'nominal_final_error_m':
                nominal_metrics.get(
                    'final_position_error_m'
                ),
            'heavy_final_error_m':
                heavy_metrics.get(
                    'final_position_error_m'
                ),
            'nominal_maximum_rpm':
                nominal_metrics.get('maximum_rpm'),
            'heavy_maximum_rpm':
                heavy_metrics.get('maximum_rpm'),
            'nominal_path_length_m':
                nominal_metrics.get('path_length_m'),
            'heavy_path_length_m':
                heavy_metrics.get('path_length_m'),
            'nominal_planning_time_mean_ms':
                nominal_metrics.get(
                    'planning_time_mean_ms'
                ),
            'heavy_planning_time_mean_ms':
                heavy_metrics.get(
                    'planning_time_mean_ms'
                ),
        },
        'profile_dependent_observations':
            profile_observations,
    }

    summary_path = (
        output_dir
        / 'parameter_profile_summary.json'
    )
    summary_path.write_text(
        json.dumps(
            summary,
            ensure_ascii=False,
            indent=2,
        ),
        encoding='utf-8',
    )

    figure = plt.figure(figsize=(8, 5))
    axis = figure.add_subplot(111)
    axis.bar(
        ['Nominal', 'Heavy'],
        [nominal_hover, heavy_hover],
    )
    axis.set_ylabel('Stable hover RPM')
    axis.set_title(
        'CQ01 Parameter Profile: Hover RPM'
    )
    figure.tight_layout()
    figure.savefig(
        output_dir
        / 'parameter_profile_rpm_comparison.png',
        dpi=180,
    )
    plt.close(figure)

    nominal_time, nominal_altitude = (
        normalized_altitude(nominal_rows)
    )
    heavy_time, heavy_altitude = (
        normalized_altitude(heavy_rows)
    )
    figure = plt.figure(figsize=(9, 5))
    axis = figure.add_subplot(111)
    axis.plot(
        nominal_time,
        nominal_altitude,
        label='Nominal',
    )
    axis.plot(
        heavy_time,
        heavy_altitude,
        label='Heavy',
    )
    axis.set_xlabel('Normalized mission time')
    axis.set_ylabel('Altitude (m)')
    axis.set_title(
        'CQ01 Nominal vs Heavy Full-Route Altitude'
    )
    axis.legend()
    figure.tight_layout()
    figure.savefig(
        output_dir
        / 'parameter_profile_altitude_comparison.png',
        dpi=180,
    )
    plt.close(figure)

    print('============================================================')
    print('CQ01 PARAMETER PROFILE COMPARISON')
    print(f'Nominal: {nominal_dir}')
    print(f'Heavy:   {heavy_dir}')
    print(f'Mass ratio: {mass_ratio:.6f}')
    print(
        'Stable hover RPM: '
        f'nominal={nominal_hover:.2f}, '
        f'heavy={heavy_hover:.2f}'
    )
    print(
        'RPM ratio: '
        f'measured={measured_rpm_ratio:.6f}, '
        f'expected={expected_rpm_ratio:.6f}'
    )
    for name, passed in checks.items():
        print(
            f'[{"PASS" if passed else "FAIL"}] {name}'
        )
    print(f'Output: {output_dir}')
    print(
        'PARAMETER PROFILE RESULT: '
        f'pass={not failed}; failed={failed}'
    )
    print('============================================================')

    return 0 if not failed else 1


if __name__ == '__main__':
    raise SystemExit(main())
