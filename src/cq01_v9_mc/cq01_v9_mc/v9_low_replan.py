#!/usr/bin/env python3
from __future__ import annotations
import json, math, os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
Point3=Tuple[float,float,float]
def path_distance(a:List[Point3],b:List[Point3])->float:
    if not a or not b:return 0.0
    return sum(min(math.dist(p,q) for q in b) for p in a)/len(a)
class V9LowReplan(Node):
    def __init__(self):
        super().__init__('cq01_v9_low_replan');transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.wrench_pub=self.create_publisher(WrenchStamped,'/cq01_v9/low_replan/wrench_cmd',20);self.active_pub=self.create_publisher(Bool,'/cq01_v9/low_replan/active',10);self.replan_pub=self.create_publisher(PoseStamped,'/cq01_v9/low_replan_goal_raw',10);self.status_pub=self.create_publisher(String,'/cq01_v9/low_replan/status',transient)
        self.create_subscription(String,'/cq01_v9/route/status',self.route_cb,transient);self.create_subscription(String,'/cq01_v9/low_strong/status',self.strong_cb,transient);self.create_subscription(PoseStamped,'/drone/goal_raw',lambda m:setattr(self,'current_goal',m),10);self.create_subscription(NavPath,'/drone/planned_path_3d',self.path_cb,transient);self.create_subscription(Odometry,'/drone/odom',self.odom_cb,50)
        self.route:Dict={};self.strong:Dict={};self.current_goal:Optional[PoseStamped]=None;self.current_path:List[Point3]=[];self.old_path:List[Point3]=[];self.position:Optional[Point3]=None;self.state='waiting_for_strong_recovery';self.arm_time:Optional[float]=None;self.active_start:Optional[float]=None;self.replan_time:Optional[float]=None;self.replan_latency:Optional[float]=None;self.new_path=False;self.branch_m=0.0;self.triggered=False;self.output_dir:Optional[Path]=None;self.saved=False
        self.timer=self.create_timer(0.01,self.tick);self.status_timer=self.create_timer(0.5,self.publish_status)
    def now_s(self):return self.get_clock().now().nanoseconds/1e9
    def route_cb(self,m):
        try:self.route=json.loads(m.data)
        except json.JSONDecodeError:return
    def strong_cb(self,m):
        try:self.strong=json.loads(m.data)
        except json.JSONDecodeError:return
        if self.strong.get('state') in ('recovery','recovered') and float(self.strong.get('current_net_deviation_m',999.0)) <= 1.5 and int(self.route.get('current_index',-1))==2 and not self.triggered and self.arm_time is None:
            self.arm_time=self.now_s()+1.0;self.state='armed_after_strong_recovery';self.get_logger().warning('V9 LOW ORDINARY 2.5 N WIND + A* REPLAN ARMED.')
    def path_cb(self,m):
        path=[(float(p.pose.position.x),float(p.pose.position.y),float(p.pose.position.z)) for p in m.poses]
        if len(path)<2:return
        if self.replan_time is not None and not self.new_path:
            self.new_path=True;self.replan_latency=self.now_s()-self.replan_time;self.branch_m=path_distance(self.old_path,path);self.state='new_path_received';self.save();self.get_logger().warning(f'V9 LOW REPLAN PATH READY: branch={self.branch_m:.3f} m, latency={self.replan_latency:.3f} s.')
        self.current_path=path
    def odom_cb(self,m):p=m.pose.pose.position;self.position=(float(p.x),float(p.y),float(p.z))
    def force(self,magnitude,active):
        m=WrenchStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.wrench.force.y=float(magnitude);self.wrench_pub.publish(m);b=Bool();b.data=bool(active);self.active_pub.publish(b)
    def request_replan(self):
        if self.current_goal is None:self.state='failed_no_goal';self.save();return
        self.old_path=list(self.current_path);m=PoseStamped();m.header=self.current_goal.header;m.pose=self.current_goal.pose;m.header.stamp=self.get_clock().now().to_msg();self.replan_pub.publish(m);self.replan_time=self.now_s();self.state='replan_requested';self.get_logger().warning('V9 LOW ORDINARY WIND COMPLETE; A* REPLAN REQUESTED.')
    def tick(self):
        now=self.now_s()
        if self.arm_time is not None and now>=self.arm_time and not self.triggered:self.triggered=True;self.active_start=now;self.state='ordinary_wind_active'
        if self.state=='ordinary_wind_active':
            elapsed=now-(self.active_start or now)
            if elapsed<1.0:
                ramp=min(1.0,elapsed/0.15,(1.0-elapsed)/0.15);self.force(2.5*max(0.0,ramp),True)
            else:self.force(0.0,False);self.request_replan()
        else:self.force(0.0,False)
        if self.replan_time is not None and not self.new_path and now-self.replan_time>3.0:self.state='failed_replan_timeout';self.save()
    def save(self):
        if self.saved:return
        criteria={'ordinary_2_5N_pulse_applied':self.triggered,'replan_requested':self.replan_time is not None,'new_path_received':self.new_path,'route_continued':int(self.route.get('current_index',-1))>=2}
        out=Path(os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v9_mc/low_replan'))/datetime.now().strftime('low_replan_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=True);self.output_dir=out
        s={'experiment':'cq01_v9_low_2_5N_wind_astar_replan','pass':all(criteria.values()),'failed_criteria':[k for k,v in criteria.items() if v is not True],'criteria':criteria,'replan_latency_s':self.replan_latency,'new_path_branch_m':self.branch_m,'force_n':[0.0,2.5,0.0],'duration_s':1.0};(out/'summary.json').write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8');self.saved=True;self.publish_status()
    def publish_status(self):
        p={'state':self.state,'triggered':self.triggered,'replan_requested':self.replan_time is not None,'new_path_received':self.new_path,'replan_latency_s':self.replan_latency,'new_path_branch_m':self.branch_m,'output_dir':str(self.output_dir) if self.output_dir else None,'pass_result':None if not self.saved else (self.triggered and self.new_path)};m=String();m.data=json.dumps(p,ensure_ascii=False,sort_keys=True);self.status_pub.publish(m)
def main(args=None):
    rclpy.init(args=args);n=V9LowReplan()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
