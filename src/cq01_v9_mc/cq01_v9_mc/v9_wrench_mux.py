#!/usr/bin/env python3
from __future__ import annotations
import json, math
from typing import Dict, Optional
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
class V9WrenchMux(Node):
    def __init__(self):
        super().__init__('cq01_v9_wrench_mux')
        transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output=self.create_publisher(WrenchStamped,'/drone/external_wrench',20); self.active_output=self.create_publisher(Bool,'/drone/disturbance_active',10)
        self.status_pub=self.create_publisher(String,'/cq01_v9/wrench_mux/status',transient)
        self.sources=('low_strong','low_replan','climb','h14'); self.commands:Dict[str,Optional[WrenchStamped]]={s:None for s in self.sources}
        self.command_time={s:-1e9 for s in self.sources}; self.active={s:False for s in self.sources}; self.active_time={s:-1e9 for s in self.sources}; self.selected_ever={s:False for s in self.sources}
        self.collision_ever=False; self.selected='none'
        topics={'low_strong':('/cq01_v9/low_strong/wrench_cmd','/cq01_v9/low_strong/active'),'low_replan':('/cq01_v9/low_replan/wrench_cmd','/cq01_v9/low_replan/active'),'climb':('/cq01_v9/climb/wrench_cmd','/cq01_v9/climb/active'),'h14':('/cq01_v9/h14/wrench_cmd','/cq01_v9/h14/active')}
        for s,(ct,at) in topics.items():
            self.create_subscription(WrenchStamped,ct,lambda m,s=s:self.command_cb(s,m),20); self.create_subscription(Bool,at,lambda m,s=s:self.active_cb(s,m),10)
        self.timer=self.create_timer(0.005,self.tick)
    def now_s(self):return self.get_clock().now().nanoseconds/1e9
    def command_cb(self,s,m):self.commands[s]=m;self.command_time[s]=self.now_s()
    def active_cb(self,s,m):self.active[s]=bool(m.data);self.active_time[s]=self.now_s()
    def mag(self,m):
        if m is None:return 0.0
        f=m.wrench.force;t=m.wrench.torque;return math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z+t.x*t.x+t.y*t.y+t.z*t.z)
    def effective(self,s,now):return (now-self.command_time[s]<=0.2 and self.mag(self.commands[s])>1e-6) or (now-self.active_time[s]<=0.2 and self.active[s])
    def zero(self):
        m=WrenchStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();return m
    def publish_status(self,active_sources):
        p={'state':'collision' if self.collision_ever else 'running','selected_source':self.selected,'selected_ever':self.selected_ever,'collision_ever':self.collision_ever,'active_sources':active_sources}
        m=String();m.data=json.dumps(p,ensure_ascii=False,sort_keys=True);self.status_pub.publish(m)
    def tick(self):
        now=self.now_s();active_sources=[s for s in self.sources if self.effective(s,now)]
        if len(active_sources)>1:self.collision_ever=True;sel='collision';out=self.zero();active=False
        elif len(active_sources)==1:
            sel=active_sources[0];self.selected_ever[sel]=True;out=self.commands[sel] or self.zero();out.header.frame_id='map';out.header.stamp=self.get_clock().now().to_msg();active=True
        else:sel='none';out=self.zero();active=False
        self.output.publish(out);b=Bool();b.data=active;self.active_output.publish(b)
        if sel!=self.selected:self.selected=sel;self.publish_status(active_sources)
def main(args=None):
    rclpy.init(args=args);n=V9WrenchMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
