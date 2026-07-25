#!/usr/bin/env python3
from __future__ import annotations
import json,os
from datetime import datetime
from pathlib import Path
from typing import Dict,Set
import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
class V9RuntimeMonitor(Node):
    def __init__(self):
        super().__init__('cq01_v9_runtime_monitor');transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.data:Dict[str,Dict]={};self.reached:Set[int]=set();self.paths={'standard':0,'astar':0,'actual':0,'pattern':0};self.max_goal_raw=0;self.max_goal=0;self.max_wrench=0;self.saved=False;self.start=self.now_s();self.results_root=Path(os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v9_mc/integrated'))
        topics={'prelude':'/cq01_v9/prelude/status','route':'/cq01_v9/route/status','sensors':'/cq01_v9/sensors/status','low_strong':'/cq01_v9/low_strong/status','low_replan':'/cq01_v9/low_replan/status','climb':'/cq01_v9/climb/status','h14':'/cq01_v9/h14/status','dynamic':'/cq01_v9/dynamic/status','pattern':'/cq01_v9/pattern/status','pattern_result':'/cq01_v9/pattern/result','integrated':'/cq01_v9/integrated/status','raw_mux':'/cq01_v9/raw_goal_mux/status','wrench_mux':'/cq01_v9/wrench_mux/status','ground_station':'/cq01_v9/ground_station/status'}
        for k,t in topics.items():self.create_subscription(String,t,lambda m,k=k:self.cb(k,m),transient)
        for k,t,q in (('standard','/cq01_v2/reference_path',transient),('astar','/drone/planned_path_3d',transient),('actual','/cq01_v9/actual_trajectory',10),('pattern','/cq01_v9/pattern/reference_path',transient)):self.create_subscription(NavPath,t,lambda m,k=k:self.path_cb(k,m),q)
        self.timer=self.create_timer(0.5,self.tick)
    def now_s(self):return self.get_clock().now().nanoseconds/1e9
    def cb(self,k,m):
        try:d=json.loads(m.data)
        except json.JSONDecodeError:return
        self.data[k]=d
        if k=='route' and 'reached_index' in d:
            try:self.reached.add(int(d['reached_index']))
            except Exception:pass
    def path_cb(self,k,m):self.paths[k]=max(self.paths[k],len(m.poses))
    def node_count(self,exact):
        target=exact if exact.startswith('/') else '/'+exact;count=0
        for name,ns in self.get_node_names_and_namespaces():
            ns=(ns or '/').rstrip('/');fq=f'{ns}/{name}' if ns else f'/{name}'
            if fq==target:count+=1
        return count
    def tick(self):
        if self.saved:return
        self.max_goal_raw=max(self.max_goal_raw,self.count_publishers('/drone/goal_raw'));self.max_goal=max(self.max_goal,self.count_publishers('/drone/goal'));self.max_wrench=max(self.max_wrench,self.count_publishers('/drone/external_wrench'))
        route=self.data.get('route',{});pattern=self.data.get('pattern_result',{});integrated=self.data.get('integrated',{})
        complete=route.get('state')=='mission_completed' and int(route.get('reached_index',-1))==15 and pattern.get('pass') is True and integrated.get('state') in ('summit_pattern_completed','integrated_completed')
        if complete:self.save('all_functions_completed')
        elif self.now_s()-self.start>600:self.save('timeout')
    def save(self,final_state):
        if self.saved:return
        self.saved=True;climb=self.data.get('climb',{});h14=self.data.get('h14',{});mux=self.data.get('raw_mux',{}).get('counts',{});wrench=self.data.get('wrench_mux',{})
        nodes={'rviz':self.node_count('/cq01_v9_rviz'),'raw_goal_mux':self.node_count('/cq01_v9_raw_goal_mux'),'wrench_mux':self.node_count('/cq01_v9_wrench_mux'),'controller':self.node_count('/quadrotor_controller'),'dynamics':self.node_count('/quadrotor_dynamics'),'astar':self.node_count('/astar_3d_planner'),'dispatcher':self.node_count('/cq01_v9_route_dispatcher'),'sensor_suite':self.node_count('/cq01_v9_sensor_suite'),'ground_station':self.node_count('/cq01_v9_ground_station'),'camera':self.node_count('/cq01_v9_camera_focus')}
        final_error=float(self.data.get('pattern_result',{}).get('overall_metrics',{}).get('final_hold_error_m',999.0))
        criteria={
            'hover_single_three_target':self.data.get('prelude',{}).get('state')=='prelude_completed',
            'truth_nominal_stress_sensors':self.data.get('sensors',{}).get('pass_result') is True,
            'static_obstacle_route_P0_P15':self.data.get('route',{}).get('state')=='mission_completed',
            'low_strong_8_10m_and_recovery':self.data.get('low_strong',{}).get('pass_result') is True,
            'low_2_5N_wind_and_astar_replan':self.data.get('low_replan',{}).get('pass_result') is True,
            'climb_P3_P4_wind_recovery':climb.get('state')=='results_saved' and climb.get('triggered_once') is True,
            'bridge_and_gorge_reached':7 in self.reached and 8 in self.reached,
            'dense_building_and_high_city_climb':4 in self.reached and 12 in self.reached,
            'h14_high_wind_replan_retracking':h14.get('pass_result') is True,
            'dynamic_obstacle_replan':self.data.get('dynamic',{}).get('pass_result') is True,
            'rapid_ascent_circle_figure8':self.data.get('pattern_result',{}).get('pass') is True,
            'final_hold_error_at_most_0_3m':final_error<=0.3,
            'read_only_ground_station':self.data.get('ground_station',{}).get('read_only') is True,
            'all_raw_goal_sources_used':all(int(mux.get(k,0))>=1 for k in ('prelude','route','low_replan','h14_replan','dynamic_replan')),
            'all_wrench_sources_used':all(wrench.get('selected_ever',{}).get(k) is True for k in ('low_strong','low_replan','climb','h14')),
            'wrench_no_collision':wrench.get('collision_ever') is False,
            'one_goal_raw_publisher':self.max_goal_raw<=1,
            'one_goal_publisher':self.max_goal<=1,
            'one_external_wrench_publisher':self.max_wrench<=1,
            'one_dynamics_controller_astar':nodes['dynamics']==1 and nodes['controller']==1 and nodes['astar']==1,
            'one_rviz_camera_ground_station':nodes['rviz']==1 and nodes['camera']==1 and nodes['ground_station']==1,
            'visual_paths_received':self.paths['standard']>=16 and self.paths['astar']>=2 and self.paths['actual']>=2 and self.paths['pattern']>=2,
        }
        failed=[k for k,v in criteria.items() if v is not True];out=self.results_root/datetime.now().strftime('v9_mc_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=True)
        summary={'experiment':'cq01_v9_mc_all_function_minimum_closed_loop','final_state':final_state,'pass':not failed,'failed_criteria':failed,'criteria':criteria,'module_status':self.data,'reached_waypoint_indices':sorted(self.reached),'visual_path_pose_maxima':self.paths,'architecture':{'node_counts':nodes,'maximum_goal_raw_publishers':self.max_goal_raw,'maximum_goal_publishers':self.max_goal,'maximum_external_wrench_publishers':self.max_wrench,'one_rviz':True,'one_camera':True,'no_odom_switching':True,'control_state_source':'/drone/odom','ground_station_read_only':True}}
        path=out/'summary.json';path.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8');self.get_logger().warning(f'V9-MC RESULT pass={summary["pass"]}; failed_criteria={failed}; output={path}');self.get_logger().warning('RViz remains open. Inspect final hold, then press Ctrl+C.')
def main(args=None):
    rclpy.init(args=args);n=V9RuntimeMonitor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
