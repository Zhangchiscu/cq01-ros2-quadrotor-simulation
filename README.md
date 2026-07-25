# CQ01 ROS2 Quadrotor Autonomous Navigation

A ROS 2 Humble quadrotor simulation and autonomous-navigation project integrating six-DOF dynamics, cascaded flight control, 3D A* planning, obstacle avoidance, disturbance testing, sensor simulation, local perception, quantitative evaluation, and a Chongqing-inspired mountainous urban scenario.

The repository is the frozen final submission baseline for the CQ01 course project. The final runtime intentionally excludes multi-UAV operation and the experimental direct-motor end-to-end neural-control branch.

## Key Features

- Six-DOF quadrotor rigid-body dynamics driven by four motor RPM commands
- Cascaded position / velocity / attitude control with motor mixing and actuator limits
- 3D A* path planning with collision checking and obstacle inflation
- Multi-goal autonomous flight and waypoint execution
- Static-obstacle avoidance, narrow-passage / bridge / gorge traversal
- Dynamic-obstacle replanning and disturbance-recovery testing
- Simplified IMU, sensor noise injection, and local `PointCloud2` perception
- Chongqing-inspired mountainous urban CQ01 scenario
- RViz2 visualization, recording, automated evaluation, and teacher-interface audit
- Reproducible acceptance evidence and experiment figures stored in the repository

## System Overview

The final submission follows a modular ROS 2 architecture:

```text
Mission / Goal
     |
     v
3D A* Planner  --->  Waypoints / Safe Goal
     |                     |
     v                     v
Reference Generation ---> Cascaded Controller
                              |
                              v
                       Four Motor RPMs
                              |
                              v
                     Six-DOF Dynamics
                              |
                              v
              Odometry / IMU / TF / Path
                              |
           +------------------+------------------+
           |                                     |
           v                                     v
   Evaluation / Recorder                  RViz2 Visualization
```

Core packages:

- `src/drone_dynamics/` — six-DOF quadrotor dynamics
- `src/drone_controller/` — cascaded position / attitude / motor-RPM controller
- `src/drone_planner_3d/` — 3D planning and collision checking
- `src/cq01_v9_mc/` — frozen integrated CQ01 mission, A* planner, recorder, dashboard, visualization, and teacher-interface bridge
- `src/cq01_requirement_closure/` — sensor-noise bridge, simplified IMU, and local perception
- `scripts/run_cq01_nominal_full.sh` — one-command nominal final entry point

## Environment

The frozen baseline was developed and validated with:

- Ubuntu 22.04
- ROS 2 Humble
- C++17 and Python
- Eigen3
- RViz2
- `colcon`

## Quick Start

Clone the repository:

```bash
git clone https://github.com/Zhangchiscu/cq01-ros2-quadrotor-simulation.git
cd cq01-ros2-quadrotor-simulation
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the frozen nominal CQ01 integrated mission:

```bash
./scripts/run_cq01_nominal_full.sh
```

Useful verification scripts:

```bash
./scripts/check_cq01_teacher_interfaces.sh
./scripts/check_cq01_phase3_strict_after_run.sh
./scripts/audit_cq01_requirement_closure_v14.sh
```

The clean repository intentionally does not track generated `build/`, `install/`, `log/`, or large runtime `results/` directories.

## Final Acceptance Results

The frozen acceptance set covers the six minimum experiments required by the course.

| Requirement | Scenario | Result | Main evidence |
|---|---|---:|---|
| Hover | Hover near `(0, 0, 1.5)` | PASS | `docs/evidence/acceptance_summary.json` |
| Single goal | Goal `(2, 1, 1.5)` and hover after arrival | PASS | `docs/evidence/acceptance_summary.json` |
| Multi-goal | Four consecutive targets / waypoint sequence | PASS | `docs/figures/03_multi_target_actual_trajectory.png` |
| Static avoidance | P0-P15 route with more than five static obstacles | PASS | `docs/evidence/acceptance_summary.json` |
| Narrow passage / detour | Bridge / gorge / corridor traversal | PASS | `docs/figures/04_route_planned_vs_actual.png` |
| Stability display | Position error, RPM, trajectory, clearance, altitude | PASS | `docs/figures/` |

The final acceptance aggregator records all six minimum scenarios as passed. Machine-readable evidence is available in:

- `docs/evidence/acceptance_summary.json`
- `docs/evidence/teacher_interface_summary.json`
- `docs/evidence/ACCEPTANCE_REPORT.md`

## Required Experiment Figures

### 1. Hover, Single-Target and Multi-Target Position Error

![Position error curve](docs/figures/01_position_error_curve.png)

### 2. Four-Motor RPM Response

![Four motor RPM curves](docs/figures/02_four_motor_rpm_curves.png)

### 3. Multi-Goal Actual Trajectory

![Multi-goal actual trajectory](docs/figures/03_multi_target_actual_trajectory.png)

### 4. P0-P15 Planned Path and Actual Trajectory

![Planned and actual trajectory](docs/figures/04_route_planned_vs_actual.png)

### 5. Minimum Static-Obstacle Distance

![Minimum obstacle distance curve](docs/figures/05_minimum_obstacle_distance_curve.png)

### 6. P0-P15 Altitude Profile

![Route altitude profile](docs/figures/06_route_altitude_profile.png)

## Core ROS 2 Interfaces

The strict teacher-interface audit verifies the following interfaces:

| Topic / TF | Role |
|---|---|
| `/drone/motor_rpm_cmd` | Four-motor RPM command sent to the dynamics |
| `/drone/odom` | Vehicle position, velocity, and attitude state |
| `/drone/imu` | Simplified IMU output |
| `/drone/path` | Actual / historical trajectory |
| `/drone/goal` | Goal pose |
| `/map/obstacles` | Obstacle map |
| `/drone/reference` | Planner / controller reference |
| `/drone/waypoints` | Planned waypoint sequence |
| `/drone/safe_goal` | Collision-safe local goal |
| `map -> base_link` | Dynamic world-to-vehicle TF |

Strict interface evidence is tracked in `docs/evidence/teacher_interface_summary.json`.

## Coordinate Frames

The simulation uses a right-handed Cartesian world convention:

- `map` is the fixed world frame
- `x` and `y` are horizontal map coordinates
- positive `z` is altitude
- `base_link` is the vehicle body frame
- the dynamics publishes the dynamic transform `map -> base_link`

Mission goals, obstacle geometry, planned paths, and actual trajectories use the same map/world convention unless a node explicitly documents an intermediate body-frame quantity.

## Goal Input from the Command Line

The controller accepts `geometry_msgs/msg/PoseStamped` on `/drone/goal`.

Example:

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

A helper script is also provided:

```text
src/drone_controller/scripts/send_goal
```

## Map, Collision Checking, and Safety

Obstacle dimensions are scenario-configured rather than globally fixed. Canonical definitions include:

- `src/drone_map/config/obstacles.yaml`
- `src/drone_world/config/*.yaml`
- `src/cq01_v2_scene/config/*.yaml`
- final CQ01 scene and planning modules under `src/cq01_v9_mc/`

Collision checking uses configured obstacle inflation / clearance parameters. The acceptance layer independently measures actual obstacle clearance.

The final success criterion is that the actual minimum obstacle distance remains greater than the configured safety threshold.

Exact tracked geometry and safety-related configuration lines are collected in:

- `docs/CONFIGURATION_EVIDENCE.md`

## Planner Behavior and Failure Conditions

The final planning stack uses 3D A* / waypoint generation with collision checking.

Inputs include:

- current vehicle state
- target goal
- obstacle map

Outputs include:

- safe reference
- waypoint sequence
- collision-safe local goal

Typical failure conditions include:

- no collision-free path exists
- start or goal lies inside occupied / inflated space
- safety inflation closes a narrow passage
- required map or state input is unavailable
- a dynamic obstacle invalidates the current route and replanning cannot produce a valid replacement

The final repository includes static avoidance, P0-P15 route clearance, bridge / gorge narrow-passage tests, and dynamic replanning experiments.

## Trajectory-Input Bonus Functions

The optional trajectory-input item is implemented in addition to the required goal-based tasks.

Implemented modes include:

- circle trajectory
- figure-eight trajectory
- waypoint-list / multi-goal trajectory

Relevant packages:

- `src/drone_pattern_trajectory/`
- `src/drone_multigoal/`
- pattern manager / orchestrator / evaluator modules under `src/cq01_v9_mc/`

## Chongqing CQ01 Scenario

The CQ01 extension is a Chongqing-inspired mountainous urban scenario designed to stress the planner and controller with:

- strong elevation variation
- dense buildings
- narrow passages
- bridge / gorge geometry
- long multi-stage routes
- static and dynamic obstacles
- external disturbances
- perception and sensor-noise tests

The scenario is intended as an engineering stress-test environment rather than a survey-grade reconstruction of real building geometry.

Packages related to Chongqing visualization and data include:

- `src/drone_chongqing/`
- `src/drone_chongqing_cq01/`
- `src/drone_chongqing_fullscene/`
- `src/drone_chongqing_map/`
- `src/drone_chongqing_realdata/`
- `src/cq01_v2_scene/`
- `src/cq01_v9_mc/`

## Final Runtime vs. Development Packages

The repository preserves development and regression packages because the system was built incrementally.

### Frozen final runtime

The teacher-facing final chain is centered on:

- `src/drone_dynamics/`
- `src/drone_controller/`
- `src/drone_planner_3d/`
- `src/cq01_v9_mc/`
- `src/cq01_requirement_closure/`
- `scripts/run_cq01_nominal_full.sh`

### Development / regression support

Packages containing earlier CQ01 versions or `regression` preserve development-stage tests such as disturbance regression, high-altitude wind checks, visualization iterations, and supporting benchmarks.

General-purpose packages such as `drone_world_validation`, `drone_robustness`, `drone_dynamic_replanning`, and analysis / evaluation modules are supporting test infrastructure rather than competing final controllers.

The frozen baseline and exclusions are documented in:

- `FINAL_BASELINE.md`

## Relationship to Reference Repositories

The repositories named in the assignment were used as design references rather than copied as the final runtime.

| Reference | Used as a reference for | Implemented independently in this repository |
|---|---|---|
| `pengyu_sim` | ROS 2-oriented simulation task decomposition and separation of dynamics, control, and visualization | `drone_dynamics`, `drone_controller`, teacher-interface bridge, acceptance tooling, and CQ01 integrated launch chain |
| `MARSIM` | 3D / point-cloud mapping and local-perception concepts | Local `PointCloud2` representation, local perception crop, 3D planning, and collision-checking chain |

The final submission does not require either reference repository as a runtime dependency.

## AI Usage and Human Verification

AI tools were used during development for code drafting, debugging support, documentation, experiment analysis, and exploration of alternative control approaches.

All final functionality was manually integrated, executed, inspected, and validated through the repository's acceptance and regression tools.

An experimental recurrent direct-motor neural policy was explored during development but did not achieve reliable closed-loop hover. It is therefore documented as a failed research experiment and is intentionally excluded from the frozen final runtime.

See:

- `ai_usage.md`

## Repository Structure

```text
.
├── docs/                 # acceptance evidence, requirement traceability, figures
├── report/assets/        # experiment outputs and quantitative analysis
├── scripts/              # final run, audit, and evaluation scripts
├── src/                  # ROS 2 packages
├── FINAL_BASELINE.md     # frozen final scope and exclusions
├── ai_usage.md           # AI-use disclosure and failed AI-control exploration
└── README.md
```

## Reproducibility and Traceability

Useful traceability documents:

- `docs/REQUIREMENTS_TRACEABILITY.md`
- `docs/CONFIGURATION_EVIDENCE.md`
- `docs/evidence/ACCEPTANCE_REPORT.md`
- `docs/evidence/acceptance_summary.json`
- `docs/evidence/teacher_interface_summary.json`

The final clean repository excludes generated build artifacts and large runtime result directories, while retaining the source, configuration, scripts, figures, and machine-readable acceptance evidence needed to inspect and reproduce the submitted baseline.

## Scope and Limitations

The frozen final runtime intentionally excludes:

- multi-UAV operation
- experimental direct-motor end-to-end neural control

The final submitted system is the validated classical dynamics + controller + 3D planning + Chongqing scene + disturbance / sensor / evaluation stack.

## Licensing and Data Notes

This repository currently contains component-level license and data-license notices, including:

- `src/drone_dynamics/LICENSE`
- `src/drone_chongqing_realdata/web/DATA_LICENSE.md`

A repository-wide license has not yet been added. Public visibility alone should not be interpreted as granting a blanket license over all source code, data, and third-party-derived assets. Component and data notices should be reviewed before reuse.
