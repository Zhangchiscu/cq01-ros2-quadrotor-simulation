# ROS2 Quadrotor Simulator

A small quadrotor dynamics and control simulator developed with ROS2 Humble.

## Planned Modules

- `drone_dynamics`: six-DOF quadrotor dynamics driven by four motor RPM commands
- `drone_controller`: cascaded position and attitude controller
- `drone_bringup`: launch files, RViz2 configuration and experiment tools

## Environment

- Ubuntu 22.04
- ROS2 Humble
- C++17
- Eigen3

<!-- CQ01_FINAL_SUBMISSION_QUICKSTART_V1 -->

# Final Submission Quick Start

This section is the teacher-facing entry point for the frozen final submission.

## 1. Environment

The frozen project was developed and validated with:

- Ubuntu 22.04
- ROS 2 Humble
- C++ and Python
- RViz2 for 3D visualization

## 2. Build

From a fresh ROS 2 Humble shell:

```bash
source /opt/ros/humble/setup.bash
cd ~/your_drone_sim_ws
colcon build --symlink-install
source install/setup.bash
```

The repository contains the ROS 2 packages under `src/`. The final clean Git repository intentionally does not track `build/`, `install/`, `log/`, or large runtime `results/`.

## 3. One-command final run

After building and sourcing the workspace:

```bash
./scripts/run_cq01_nominal_full.sh
```

This is the frozen nominal CQ01 Chongqing integrated mission used for the final submission baseline.

Useful verification scripts include:

```bash
./scripts/check_cq01_teacher_interfaces.sh
./scripts/check_cq01_phase3_strict_after_run.sh
./scripts/audit_cq01_requirement_closure_v14.sh
```

## 4. Core ROS 2 interfaces

The final teacher-interface audit explicitly verifies the following interfaces:

| Topic / TF | Role |
|---|---|
| `/drone/motor_rpm_cmd` | Four-motor RPM command sent to the quadrotor dynamics |
| `/drone/odom` | Vehicle position, velocity and attitude state |
| `/drone/imu` | Simplified IMU output |
| `/drone/path` | Actual/history trajectory |
| `/drone/goal` | Goal pose |
| `/map/obstacles` | Obstacle map used by planning / teacher interface |
| `/drone/reference` | Planner/controller reference |
| `/drone/waypoints` | Planned waypoint sequence |
| `/drone/safe_goal` | Collision-safe local goal |
| `map -> base_link` | Dynamic world-to-vehicle TF |

The strict audit evidence is tracked in `docs/evidence/teacher_interface_summary.json`.

## 5. Coordinate frames

The simulation uses a right-handed Cartesian world/map convention:

- `map` is the fixed world frame used by the final mission and planning stack.
- `x` and `y` are horizontal map coordinates.
- positive `z` is altitude.
- `base_link` is the vehicle body frame.
- the dynamics publishes the dynamic transform `map -> base_link`.

All mission goals, obstacle geometry, planned paths and actual trajectories are interpreted in the same map/world convention unless a node explicitly documents an intermediate body-frame quantity.

## 6. Obstacle geometry and dimensions

Obstacle dimensions are **scenario-configured rather than one global fixed size**.

Canonical tracked definitions include:

- `src/drone_map/config/obstacles.yaml`
- `src/drone_world/config/*.yaml`
- `src/cq01_v2_scene/config/*.yaml`
- final CQ01 scene/planning packages under `src/cq01_v9_mc/`

The map/scene configuration defines obstacle positions and geometric dimensions. This makes the same planner reusable across simple boxes, corridors, Chongqing-style buildings and multi-level terrain.

Exact tracked geometry-related configuration/code lines from the frozen repository are collected in:

`docs/CONFIGURATION_EVIDENCE.md`

## 7. Safety distance / collision clearance

The planner does not rely only on visual separation in RViz. Collision checking uses configured obstacle inflation / clearance parameters, and the acceptance layer independently measures actual obstacle clearance.

The final success condition is:

> actual minimum obstacle distance must remain greater than the configured safety/clearance threshold.

The frozen repository tracks the exact safety/clearance-related configuration lines in:

`docs/CONFIGURATION_EVIDENCE.md`

Final quantitative evidence is provided by:

- `docs/figures/05_minimum_obstacle_distance_curve.png`
- `docs/evidence/acceptance_summary.json`
- route-clearance and integrated acceptance evaluators under `src/cq01_v9_mc/`

## 8. Planner behavior and failure conditions

The final planning stack uses 3D A* / waypoint generation with collision checking. Inputs are the current vehicle state, target goal and obstacle map; outputs are safe reference / waypoint / local-goal information.

Typical failure conditions include:

- no collision-free path exists;
- the start or goal lies inside occupied/inflated space;
- safety inflation closes a narrow passage;
- the map or required state input is unavailable;
- a dynamic obstacle invalidates the current route and replanning cannot produce a valid replacement.

The final acceptance evidence includes static obstacle avoidance, P0-P15 route clearance, bridge/gorge narrow passage and dynamic replanning tests.

## 9. Scope statement

The final runtime intentionally excludes:

- multi-UAV operation;
- the experimental direct-motor end-to-end neural-control branch.

The neural-control exploration is documented in `ai_usage.md` as a failed research experiment and is not presented as a final course function.

The final submitted runtime is the validated classical dynamics + controller + 3D planning + Chongqing scene + disturbance/sensor/evaluation stack.

<!-- CQ01_PDF_SEMANTIC_FIX_V1 -->

# Teacher-facing Acceptance and Reproducibility Guide

This section maps the frozen repository directly to the course PDF requirements.

## A. Minimum acceptance experiments

The following six experiments are part of the frozen final acceptance set.

| Requirement | Scenario | Frozen result | Main evidence |
|---|---|---|---|
| Hover experiment | Hover near `(0, 0, 1.5)` | PASS | `docs/evidence/acceptance_summary.json` |
| Single-goal experiment | Goal `(2, 1, 1.5)` and hover after arrival | PASS | `docs/evidence/acceptance_summary.json` |
| Multi-goal experiment | 3-4 consecutive goals / waypoint sequence | PASS | `docs/figures/03_multi_target_actual_trajectory.png` |
| Static-obstacle avoidance | P0-P15 route with more than five static obstacles | PASS | `docs/evidence/acceptance_summary.json` |
| Narrow corridor / detour | bridge / gorge / narrow-passage route with planned and actual trajectories | PASS | `docs/figures/04_route_planned_vs_actual.png` |
| Stability display | position error, four-motor RPM, trajectory and minimum obstacle distance | PASS | figures below |

The final acceptance aggregator recorded all six minimum scenarios as passed. The exact acceptance summary is tracked in:

`docs/evidence/acceptance_summary.json`

### Required stability figures

#### Position-error curve

![Position error curve](docs/figures/01_position_error_curve.png)

#### Four-motor RPM curves

![Four motor RPM curves](docs/figures/02_four_motor_rpm_curves.png)

#### Multi-goal actual trajectory

![Multi-goal actual trajectory](docs/figures/03_multi_target_actual_trajectory.png)

#### Minimum obstacle-distance curve

![Minimum obstacle distance curve](docs/figures/05_minimum_obstacle_distance_curve.png)

Additional comparison figures are also tracked:

- `docs/figures/04_route_planned_vs_actual.png`
- `docs/figures/06_route_altitude_profile.png`

## B. Goal input from the command line

The controller accepts `geometry_msgs/msg/PoseStamped` on `/drone/goal`.

A copyable example for the PDF's single-goal case is:

```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}}"
```

The repository also includes the helper script:

`src/drone_controller/scripts/send_goal`

Therefore the final system supports changing the target through a ROS 2 command-line interface in addition to the visualized goal pipeline.

## C. Trajectory-input bonus item

The optional trajectory-input item in the course PDF is implemented rather than being replaced by the dynamic-obstacle feature.

The repository contains:

- circle trajectory support;
- figure-eight trajectory support;
- waypoint-list / multi-goal trajectory support.

Relevant packages/modules include:

- `src/drone_pattern_trajectory/`
- `src/drone_multigoal/`
- pattern manager / orchestrator / evaluator modules under `src/cq01_v9_mc/`

The frozen CQ01 integrated showcase includes circle and figure-eight tracking as bonus demonstrations.

## D. Relationship to the two reference repositories

The two repositories named in the assignment were used as **references**, not copied as the final runtime.

| Reference | What was used as a reference | What the final project does independently |
|---|---|---|
| `pengyu_sim` | ROS 2-oriented quadrotor simulation task decomposition and the general idea of separating dynamics, control and visualization responsibilities | The submitted `drone_dynamics`, `drone_controller`, teacher-interface bridge, acceptance tooling and CQ01 integrated launch chain are maintained in this repository rather than depending on a copied reference workspace |
| `MARSIM` | 3D / point-cloud mapping and local-perception concepts were used as a design reference | The submitted system publishes its own `PointCloud2` obstacle representation, performs a local perception crop, and uses the project's own 3D planning / collision-checking chain |

Why this project is organized differently:

1. The course requires a compact ROS 2 engineering submission whose core dynamics and controller can be inspected directly, so the final runtime keeps these modules explicit and separate.
2. The CQ01 extension focuses on a reproducible Chongqing-style integrated scenario, disturbance recovery, dynamic replanning, sensor noise, local perception and automatic acceptance evidence.
3. The reference repositories are therefore treated as design references; they are not runtime dependencies that must be cloned for the frozen submission to operate.

## E. Final runtime versus development / regression packages

The repository contains development and regression packages because the project was built incrementally. They are intentionally separated conceptually from the frozen final runtime.

### Final submission runtime

The teacher-facing final chain is centered on:

- `src/drone_dynamics/` - quadrotor dynamics;
- `src/drone_controller/` - position / attitude / motor-RPM controller;
- `src/drone_planner_3d/` and the CQ01 A* planner - 3D planning and collision checking;
- `src/cq01_v9_mc/` - frozen integrated Chongqing mission, visualization, recorder, dashboard and teacher-interface bridge;
- `src/cq01_requirement_closure/` - sensor-noise / simplified-IMU bridge and local PointCloud perception;
- `scripts/run_cq01_nominal_full.sh` - one-command final nominal entry point.

### Development / regression support

Packages whose names include earlier CQ01 versions or `regression` preserve development-stage tests such as disturbance regression, high-altitude wind checks and visualization iterations. They are not alternate final controllers and should not be interpreted as competing runtime versions.

Likewise, general-purpose packages such as `drone_world_validation`, `drone_robustness`, `drone_dynamic_replanning` and analysis/evaluation packages are supporting test infrastructure used to verify the final system.

The final baseline and explicit exclusions are documented in:

`FINAL_BASELINE.md`

## F. Safety and obstacle parameters

The README's earlier safety-distance and obstacle-dimension sections remain the canonical human-readable summary.

For traceability, exact matching configuration/code lines from the frozen source tree are additionally collected in:

- `docs/CONFIGURATION_EVIDENCE.md`
- `docs/FINAL_PARAMETER_CANDIDATES.md`

These files are evidence aids; when a specific numeric parameter is quoted in the final report, it should be taken from the parameter actually used by the corresponding final launch/configuration rather than inferred from an unrelated test scenario.

