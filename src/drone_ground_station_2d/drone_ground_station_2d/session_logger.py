from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Optional

from .models import GroundSnapshot


class SessionLogger:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.active = False
        self.directory: Optional[Path] = None
        self.handle = None
        self.writer = None
        self.started_at: Optional[datetime] = None
        self.sample_count = 0
        self.metrics = {
            'minimum_inter_uav_distance_m': None,
            'maximum_speed_mps': {},
            'maximum_goal_error_m': {},
            'maximum_rpm': {},
        }
        self.events = []

    def start(self) -> Path:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.directory = self.root / f'ground_station_2d_{stamp}'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.handle = (self.directory / 'telemetry.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.handle)
        self.writer.writerow([
            'wall_time_iso', 'scenario', 'vehicle', 'connected',
            'x_m', 'y_m', 'z_m', 'vx_mps', 'vy_mps', 'vz_mps', 'speed_mps',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'goal_x_m', 'goal_y_m', 'goal_z_m', 'goal_error_m',
            'rpm_fl', 'rpm_fr', 'rpm_rr', 'rpm_rl',
            'safety_active', 'inter_uav_distance_m', 'predicted_inter_uav_distance_m',
            'planner_state', 'trajectory_state', 'replanning_state', 'coordinator_state',
        ])
        self.started_at = datetime.now()
        self.sample_count = 0
        self.metrics = {
            'minimum_inter_uav_distance_m': None,
            'maximum_speed_mps': {},
            'maximum_goal_error_m': {},
            'maximum_rpm': {},
        }
        self.events = []
        self.active = True
        return self.directory

    def add_event(self, text: str) -> None:
        if self.active:
            self.events.append({'time': datetime.now().isoformat(timespec='milliseconds'), 'event': text})

    def append(self, snapshot: GroundSnapshot) -> None:
        if not self.active or self.writer is None: return
        now = datetime.now().isoformat(timespec='milliseconds')
        scenario = snapshot.scenario()
        system = snapshot.system
        for key, state in snapshot.vehicles.items():
            connected = state.connected(snapshot.monotonic_time)
            error = state.goal_error()
            self.writer.writerow([
                now, scenario, key, connected,
                *[f'{v:.8f}' for v in state.position],
                *[f'{v:.8f}' for v in state.velocity], f'{state.speed():.8f}',
                *[f'{v:.8f}' for v in state.attitude_deg],
                *[f'{v:.8f}' for v in state.goal], '' if error is None else f'{error:.8f}',
                *[f'{v:.3f}' for v in state.rpm],
                system.safety_active,
                '' if system.current_inter_uav_distance_m is None else f'{system.current_inter_uav_distance_m:.8f}',
                '' if system.predicted_inter_uav_distance_m is None else f'{system.predicted_inter_uav_distance_m:.8f}',
                system.planner_status.get('state', ''),
                system.trajectory_status.get('state', ''),
                system.replanning_status.get('state', ''),
                system.coordinator_state,
            ])
            self.metrics['maximum_speed_mps'][key] = max(
                float(self.metrics['maximum_speed_mps'].get(key, 0.0)), state.speed()
            )
            if error is not None:
                self.metrics['maximum_goal_error_m'][key] = max(
                    float(self.metrics['maximum_goal_error_m'].get(key, 0.0)), error
                )
            self.metrics['maximum_rpm'][key] = max(
                float(self.metrics['maximum_rpm'].get(key, 0.0)), max(state.rpm or [0.0])
            )
            self.sample_count += 1
        distance = system.current_inter_uav_distance_m
        if distance is not None:
            old = self.metrics['minimum_inter_uav_distance_m']
            self.metrics['minimum_inter_uav_distance_m'] = distance if old is None else min(float(old), distance)
        if self.handle is not None: self.handle.flush()

    def stop(self, snapshot: GroundSnapshot) -> Optional[Path]:
        if not self.active: return self.directory
        assert self.directory is not None
        ended = datetime.now()
        self.events.append({
            'time': ended.isoformat(timespec='milliseconds'),
            'event': 'RECORDING STOPPED',
        })
        if self.handle is not None:
            self.handle.close()
        summary = {
            'experiment': 'advanced_ros2_quadrotor_2d_ground_station',
            'started_at': self.started_at.isoformat() if self.started_at else None,
            'ended_at': ended.isoformat(),
            'duration_s': (ended - self.started_at).total_seconds() if self.started_at else 0.0,
            'sample_count': self.sample_count,
            'scenario_at_stop': snapshot.scenario(),
            'metrics': self.metrics,
            'vehicles': {
                key: {
                    'connected': state.connected(snapshot.monotonic_time),
                    'position_m': state.position,
                    'velocity_mps': state.velocity,
                    'attitude_deg': state.attitude_deg,
                    'goal_m': state.goal,
                    'goal_error_m': state.goal_error(),
                    'rpm': state.rpm,
                }
                for key, state in snapshot.vehicles.items()
            },
            'system': {
                'planner_status': snapshot.system.planner_status,
                'trajectory_status': snapshot.system.trajectory_status,
                'replanning_status': snapshot.system.replanning_status,
                'coordinator_state': snapshot.system.coordinator_state,
                'safety_active': snapshot.system.safety_active,
            },
        }
        (self.directory / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
        with (self.directory / 'events.jsonl').open('w', encoding='utf-8') as handle:
            for event in self.events: handle.write(json.dumps(event, ensure_ascii=False) + '\n')
        self.active = False
        self.handle = None
        self.writer = None
        return self.directory
