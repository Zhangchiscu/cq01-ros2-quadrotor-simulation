#!/usr/bin/env python3
from __future__ import annotations
import json, math, os
from typing import List, Optional
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

def yaw_quaternion(yaw: float) -> Quaternion:
    q = Quaternion(); q.z = math.sin(yaw*0.5); q.w = math.cos(yaw*0.5); return q

class V9PreludeRunner(Node):
    def __init__(self) -> None:
        super().__init__('cq01_v9_prelude_runner')
        transient = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goals: List[dict] = [
            {'name':'HOVER_STABILITY','position':(0.0,0.0,12.0),'hold_s':1.5},
            {'name':'SINGLE_TARGET','position':(10.0,-15.0,14.0),'hold_s':0.6},
            {'name':'MULTI_TARGET_A','position':(15.0,-45.0,15.0),'hold_s':0.2},
            {'name':'MULTI_TARGET_B','position':(18.0,-80.0,16.0),'hold_s':0.2},
            {'name':'MULTI_TARGET_C','position':(20.0,-108.0,13.0),'hold_s':0.3},
        ]
        self.goal_pub = self.create_publisher(PoseStamped,'/cq01_v9/prelude_goal_raw',10)
        self.status_pub = self.create_publisher(String,'/cq01_v9/prelude/status',transient)
        self.create_subscription(Odometry,'/drone/odom',lambda _: setattr(self,'have_odom',True),50)
        self.create_subscription(String,'/drone/planner_3d/status',self.planner_callback,transient)
        self.index=0; self.have_odom=False; self.planner_ready=False; self.goal_sent=False
        self.hold_until: Optional[float]=None; self.send_after=self.now_s()+0.35; self.completed=False
        self.timer=self.create_timer(0.05,self.timer_callback); self.publish_status('initializing')
    def now_s(self): return self.get_clock().now().nanoseconds/1e9
    def publish_status(self,state,**extra):
        payload={'state':state,'index':self.index,'count':len(self.goals),
            'goal_name':self.goals[self.index]['name'] if self.index<len(self.goals) else None,
            'planner_ready':self.planner_ready,'have_odom':self.have_odom}
        payload.update(extra); m=String(); m.data=json.dumps(payload,ensure_ascii=False,sort_keys=True); self.status_pub.publish(m)
    def planner_callback(self,msg):
        try: p=json.loads(msg.data)
        except json.JSONDecodeError: return
        s=str(p.get('state',''))
        if s=='ready': self.planner_ready=True
        elif s=='planning_failed' and self.goal_sent:
            self.publish_status('failed',planner_status=p); self.get_logger().error('V9 prelude planning failed'); os._exit(2)
        elif s=='final_waypoint_reached' and self.goal_sent:
            goal=self.goals[self.index]; self.goal_sent=False; self.hold_until=self.now_s()+float(goal['hold_s'])
            self.publish_status('goal_reached',reached_name=goal['name'],hold_s=goal['hold_s'])
    def publish_goal(self):
        g=self.goals[self.index]; x,y,z=g['position']; m=PoseStamped(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z; m.pose.orientation=yaw_quaternion(0.0)
        self.goal_pub.publish(m); self.goal_sent=True; self.publish_status('goal_sent',goal=[x,y,z])
        self.get_logger().warning(f'V9 PRELUDE {self.index+1}/{len(self.goals)} {g["name"]}: ({x:.1f},{y:.1f},{z:.1f})')
    def timer_callback(self):
        if self.completed: return
        now=self.now_s()
        if self.hold_until is not None:
            if now<self.hold_until: return
            self.hold_until=None; self.index+=1
            if self.index>=len(self.goals):
                self.completed=True; self.publish_status('prelude_completed',hover_completed=True,single_target_completed=True,three_target_completed=True)
                self.get_logger().warning('V9 PRELUDE COMPLETE. Handing off to P0-P15 route.')
                self.create_timer(0.5,lambda:os._exit(0)); return
            self.send_after=now+0.10
        if self.planner_ready and self.have_odom and not self.goal_sent and self.hold_until is None and now>=self.send_after:
            self.publish_goal()
def main(args=None):
    rclpy.init(args=args); n=V9PreludeRunner()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
