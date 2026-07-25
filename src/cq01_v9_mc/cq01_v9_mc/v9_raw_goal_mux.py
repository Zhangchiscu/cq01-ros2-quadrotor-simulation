#!/usr/bin/env python3
from __future__ import annotations
import json
from typing import Dict
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
class V9RawGoalMux(Node):
    def __init__(self):
        super().__init__('cq01_v9_raw_goal_mux')
        transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output=self.create_publisher(PoseStamped,'/drone/goal_raw',10)
        self.status_pub=self.create_publisher(String,'/cq01_v9/raw_goal_mux/status',transient)
        self.counts:Dict[str,int]={k:0 for k in ('prelude','route','low_replan','h14_replan','dynamic_replan')}; self.last_source='none'
        topics={'prelude':'/cq01_v9/prelude_goal_raw','route':'/cq01_v9/route_goal_raw','low_replan':'/cq01_v9/low_replan_goal_raw','h14_replan':'/cq01_v9/h14_replan_goal_raw','dynamic_replan':'/cq01_v9/dynamic_replan_goal_raw'}
        for source,topic in topics.items(): self.create_subscription(PoseStamped,topic,lambda m,source=source:self.forward(source,m),10)
        self.publish_status()
    def forward(self,source,msg):
        out=PoseStamped(); out.header=msg.header; out.pose=msg.pose; out.header.stamp=self.get_clock().now().to_msg(); out.header.frame_id=out.header.frame_id or 'map'
        self.output.publish(out); self.counts[source]+=1; self.last_source=source; self.publish_status()
        if source.endswith('replan'): self.get_logger().warning(f'V9 raw-goal mux forwarded {source}.')
    def publish_status(self):
        p={'state':'running','last_source':self.last_source,'counts':self.counts,'total_forwarded':sum(self.counts.values())}; m=String(); m.data=json.dumps(p,ensure_ascii=False,sort_keys=True); self.status_pub.publish(m)
def main(args=None):
    rclpy.init(args=args); n=V9RawGoalMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
