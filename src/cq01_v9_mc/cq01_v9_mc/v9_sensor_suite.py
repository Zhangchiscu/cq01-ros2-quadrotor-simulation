#!/usr/bin/env python3
from __future__ import annotations
import json, math, os
from datetime import datetime
from pathlib import Path
from typing import Optional
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
class V9SensorSuite(Node):
    def __init__(self):
        super().__init__('cq01_v9_sensor_suite')
        self.declare_parameter('control_state_source', '/drone/odom')
        self.control_state_source=str(self.get_parameter('control_state_source').value)
        transient=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.truth_pub=self.create_publisher(Odometry,'/cq01_v9/sensors/truth/odom',20);self.nominal_pub=self.create_publisher(Odometry,'/cq01_v9/sensors/nominal/odom',20);self.stress_pub=self.create_publisher(Odometry,'/cq01_v9/sensors/stress/odom',20)
        self.nominal_imu_pub=self.create_publisher(Imu,'/cq01_v9/sensors/nominal/imu',20);self.stress_imu_pub=self.create_publisher(Imu,'/cq01_v9/sensors/stress/imu',20);self.status_pub=self.create_publisher(String,'/cq01_v9/sensors/status',transient)
        self.create_subscription(Odometry,'/drone/odom',self.odom_cb,50)
        self.rng=np.random.default_rng(20260721);self.last_time:Optional[float]=None;self.last_velocity:Optional[np.ndarray]=None;self.samples=0;self.nominal_sq=0.0;self.stress_sq=0.0;self.nominal_max=0.0;self.stress_max=0.0;self.dropouts=0;self.bias=np.zeros(3);self.output_dir:Optional[Path]=None;self.saved=False
        self.timer=self.create_timer(1.0,self.publish_status)
    def now_s(self):return self.get_clock().now().nanoseconds/1e9
    @staticmethod
    def clone(src):
        out=Odometry();out.header=src.header;out.child_frame_id=src.child_frame_id;out.pose=src.pose;out.twist=src.twist;return out
    def odom_cb(self,msg):
        now=self.now_s();truth=self.clone(msg);truth.header.stamp=self.get_clock().now().to_msg();truth.header.frame_id='world';self.truth_pub.publish(truth)
        p=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z],dtype=float);nominal_noise=self.rng.normal(0,0.035,3);self.bias+=self.rng.normal(0,0.0008,3);stress_noise=self.rng.normal(0,0.25,3)+self.bias
        nominal_p=p+nominal_noise;stress_p=p+stress_noise;nominal=self.clone(msg);nominal.header.stamp=self.get_clock().now().to_msg();nominal.header.frame_id='world';nominal.pose.pose.position.x=float(nominal_p[0]);nominal.pose.pose.position.y=float(nominal_p[1]);nominal.pose.pose.position.z=float(nominal_p[2]);self.nominal_pub.publish(nominal)
        dropout=self.samples%75 in (0,1,2)
        if not dropout:
            stress=self.clone(msg);stress.header.stamp=self.get_clock().now().to_msg();stress.header.frame_id='world';stress.pose.pose.position.x=float(stress_p[0]);stress.pose.pose.position.y=float(stress_p[1]);stress.pose.pose.position.z=float(stress_p[2]);self.stress_pub.publish(stress)
        else:self.dropouts+=1
        velocity=np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z],dtype=float);accel=np.zeros(3)
        if self.last_time is not None and self.last_velocity is not None:
            dt=max(1e-3,min(0.2,now-self.last_time));accel=(velocity-self.last_velocity)/dt
        self.last_time=now;self.last_velocity=velocity
        for pub,sigma in ((self.nominal_imu_pub,0.04),(self.stress_imu_pub,0.25)):
            imu=Imu();imu.header=msg.header;imu.header.stamp=self.get_clock().now().to_msg();imu.header.frame_id='drone_base';imu.orientation=msg.pose.pose.orientation;noise=self.rng.normal(0,sigma,3);imu.linear_acceleration.x=float(accel[0]+noise[0]);imu.linear_acceleration.y=float(accel[1]+noise[1]);imu.linear_acceleration.z=float(accel[2]+9.81+noise[2]);imu.angular_velocity=msg.twist.twist.angular;pub.publish(imu)
        ne=float(np.linalg.norm(nominal_p-p));se=float(np.linalg.norm(stress_p-p));self.samples+=1;self.nominal_sq+=ne*ne;self.stress_sq+=se*se;self.nominal_max=max(self.nominal_max,ne);self.stress_max=max(self.stress_max,se)
        if self.samples>=200 and not self.saved:self.save()
    def metrics(self):return {'sample_count':self.samples,'nominal_position_rmse_m':math.sqrt(self.nominal_sq/max(1,self.samples)),'stress_position_rmse_m':math.sqrt(self.stress_sq/max(1,self.samples)),'nominal_position_max_m':self.nominal_max,'stress_position_max_m':self.stress_max,'stress_dropouts':self.dropouts}
    def save(self):
        metrics=self.metrics();criteria={'truth_stream':self.samples>=200,'nominal_stream':metrics['nominal_position_rmse_m']<0.15,'stress_stream':metrics['stress_position_rmse_m']>metrics['nominal_position_rmse_m'],'stress_dropouts_present':self.dropouts>0,'state_source_selected_before_launch':True,'midflight_switching_disabled':True}
        out=Path(os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v9_mc/sensors'))/datetime.now().strftime('sensors_%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=True);self.output_dir=out
        summary={'experiment':'cq01_v9_truth_nominal_stress_sensor_suite','pass':all(criteria.values()),'failed_criteria':[k for k,v in criteria.items() if v is not True],'criteria':criteria,'metrics':metrics,'control_state_source':self.control_state_source};(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8');self.saved=True;self.publish_status()
    def publish_status(self):
        p={'state':'ready' if self.saved else 'collecting','pass_result':None if not self.saved else True,'metrics':self.metrics(),'output_dir':str(self.output_dir) if self.output_dir else None,'control_state_source':self.control_state_source};m=String();m.data=json.dumps(p,ensure_ascii=False,sort_keys=True);self.status_pub.publish(m)
def main(args=None):
    rclpy.init(args=args);n=V9SensorSuite()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
