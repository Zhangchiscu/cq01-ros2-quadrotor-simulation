# CQ01 Configuration Evidence

This file lists exact tracked configuration/code lines related to obstacle geometry and safety/clearance parameters.
It is generated from the frozen Git source tree so the README does not invent numeric values.

## Safety / clearance related tracked lines

- `src/cq01_v9_mc/setup.py:51` — `'course_route_clearance_evaluator = cq01_v9_mc.course_route_clearance_evaluator:main',`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:265` — `structured = np.frombuffer(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:354` — `maximum_radius: int = 6,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:359` — `for radius in range(1, maximum_radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:361` — `for dx in range(-radius, radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:362` — `for dy in range(-radius, radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:363` — `for dz in range(-radius, radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:364` — `if max(abs(dx), abs(dy), abs(dz)) != radius:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:540` — `maximum_radius: int = 4,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:545` — `for radius in range(1, maximum_radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:546` — `for dx in range(-radius, radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:547` — `for dy in range(-radius, radius + 1):`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:548` — `if max(abs(dx), abs(dy)) != radius:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_climb_h4.py:509` — `distance, _nearest = self.nearest_path_point(position)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:111` — `self.declare_parameter('gate_radius_xy_m', 13.0)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:138` — `self.declare_parameter('safety_tilt_deg', 44.0)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:140` — `self.declare_parameter('safety_rpm_ratio', 0.985)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:162` — `self.gate_radius_xy_m = max(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:277` — `self.safety_tilt_deg = float(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:283` — `self.safety_rpm_ratio = float(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:437` — `self.safety_abort = False`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:438` — `self.safety_abort_reason = ''`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:514` — `'safety_abort': self.safety_abort,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:662` — `math.hypot(dx, dy) <= self.gate_radius_xy_m`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:677` — `best_distance = float('inf')`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:702` — `distance = math.dist(position, projection)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:703` — `if distance < best_distance:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:704` — `best_distance = distance`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:717` — `best_distance = float('inf')`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:742` — `distance = math.dist(position, projection)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:743` — `if distance < best_distance:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:744` — `best_distance = distance`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:773` — `distance, _ = self.nearest_path_distance(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:777` — `if distance is not None:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:813` — `distance, _ = self.nearest_path_distance(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:818` — `distance or 0.0`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:826` — `distance, _ = self.nearest_path_distance(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:830` — `self.current_new_path_deviation_m = distance`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:1042` — `def safety_reason(self) -> Optional[str]:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_high_altitude_wind.py:1047` — `if tilt >= self.safety_tilt_deg:`

## Obstacle geometry / dimensions related tracked lines

- `src/cq01_v9_mc/setup.py:30` — `'v9_dynamic_obstacle = cq01_v9_mc.v9_dynamic_obstacle:main',`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:43` — `self.declare_parameter("fixed_height_test_z_m", 1.50)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:70` — `self.fixed_height_test_z = float(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:91` — `depth=1,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:236` — `point_count = int(message.width) * int(message.height)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:263` — `"itemsize": int(message.point_step),`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:291` — `def dimensions(self) -> Voxel:`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:293` — `dimensions = self.metadata["dimensions"]`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:329` — `dimensions = self.dimensions`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:331` — `0 <= voxel[0] < dimensions[0]`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:332` — `and 0 <= voxel[1] < dimensions[1]`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:333` — `and 0 <= voxel[2] < dimensions[2]`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:565` — `(0.0, 0.0, self.fixed_height_test_z)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:670` — `fixed_height_exists = self.fixed_height_path_exists(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:684` — `"fixed_height_test_z_m": self.fixed_height_test_z,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:685` — `"fixed_height_path_exists": fixed_height_exists,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_astar_planner.py:696` — `f"fixed-height path={fixed_height_exists}"`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:92` — `self.declare_parameter('body_width_m', 0.42)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:93` — `self.declare_parameter('body_height_m', 0.24)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:119` — `self.body_width = float(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:122` — `self.body_height = float(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:172` — `depth=1,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:293` — `body.scale.x = self.body_length`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:294` — `body.scale.y = self.body_width`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:295` — `body.scale.z = self.body_height`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:334` — `arms.scale.x = 0.10`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:393` — `rotor.scale.x = self.rotor_diameter`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:394` — `rotor.scale.y = self.rotor_diameter`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:395` — `rotor.scale.z = 0.08`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:418` — `nose.scale.x = 0.18`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:419` — `nose.scale.y = 0.18`
- `src/cq01_v9_mc/cq01_v9_mc/v9_visualizer.py:420` — `nose.scale.z = 0.18`
- `src/cq01_v9_mc/cq01_v9_mc/v9_runtime_monitor.py:14` — `super().__init__('cq01_v9_runtime_monitor');transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.data:Dict[str,Dict]={};self.reached:Set[int]=set();self.paths={'standard':0,'astar':0,'actual':0,'pattern':0};self.max_goal_raw=0;self.max_goal=0;self.max_wrench=0;self.saved=False;self.start=self.now_s();self.results_root=Path(os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v9_mc/integrated'))`
- `src/cq01_v9_mc/cq01_v9_mc/v9_runtime_monitor.py:49` — `'static_obstacle_route_P0_P15':self.data.get('route',{}).get('state')=='mission_completed',`
- `src/cq01_v9_mc/cq01_v9_mc/v9_runtime_monitor.py:54` — `'dense_building_and_high_city_climb':4 in self.reached and 12 in self.reached,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_runtime_monitor.py:56` — `'dynamic_obstacle_replan':self.data.get('dynamic',{}).get('pass_result') is True,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_climb_h4.py:97` — `self.declare_parameter('visual_force_scale_m_per_n', 8.0)`
- `src/cq01_v9_mc/cq01_v9_mc/v9_climb_h4.py:168` — `self.visual_force_scale_m_per_n = max(`
- `src/cq01_v9_mc/cq01_v9_mc/v9_climb_h4.py:242` — `depth=1,`
- `src/cq01_v9_mc/cq01_v9_mc/v9_climb_h4.py:862` — `scale_z: float,`
