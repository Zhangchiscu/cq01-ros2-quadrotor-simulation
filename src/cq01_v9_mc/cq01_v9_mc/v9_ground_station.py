#!/usr/bin/env python3
import json
from typing import Dict
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
class V9GroundStation(Node):
    def __init__(self):
        super().__init__('cq01_v9_ground_station');transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.data:Dict[str,Dict]={};self.output=self.create_publisher(String,'/cq01_v9/ground_station/status',transient)
        topics={'prelude':'/cq01_v9/prelude/status','route':'/cq01_v9/route/status','sensors':'/cq01_v9/sensors/status','low_strong':'/cq01_v9/low_strong/status','low_replan':'/cq01_v9/low_replan/status','climb':'/cq01_v9/climb/status','h14':'/cq01_v9/h14/status','dynamic':'/cq01_v9/dynamic/status','pattern':'/cq01_v9/pattern/status','integrated':'/cq01_v9/integrated/status','raw_goal_mux':'/cq01_v9/raw_goal_mux/status','wrench_mux':'/cq01_v9/wrench_mux/status'}
        for k,t in topics.items():self.create_subscription(String,t,lambda m,k=k:self.cb(k,m),transient)
        self.last_stage='';self.timer=self.create_timer(0.5,self.publish)
    def cb(self,k,m):
        try:self.data[k]=json.loads(m.data)
        except json.JSONDecodeError:return
    def stage(self):
        ps=str(self.data.get('pattern',{}).get('state',''))
        if ps and ps not in ('idle','waiting_for_start'):return 'SUMMIT_PATTERN'
        route=self.data.get('route',{})
        if route.get('state')=='mission_completed':return 'ROUTE_COMPLETE'
        idx=int(route.get('current_index',-1))
        if idx>=13:return 'HIGH_CITY'
        if idx>=7:return 'BRIDGE_GORGE'
        if idx>=4:return 'MID_CITY'
        if idx>=2:return 'LOW_CITY'
        if self.data.get('prelude',{}).get('state')=='prelude_completed':return 'ROUTE_START'
        return 'PRELUDE'
    def publish(self):
        stage=self.stage();p={'state':'running','stage':stage,'read_only':True,'publishes_control_topics':False,'modules':self.data};m=String();m.data=json.dumps(p,ensure_ascii=False,sort_keys=True);self.output.publish(m)
        if stage!=self.last_stage:self.last_stage=stage;self.get_logger().warning(f'V9 GROUND STATION STAGE: {stage}')
def main(args=None):
    rclpy.init(args=args);n=V9GroundStation()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
