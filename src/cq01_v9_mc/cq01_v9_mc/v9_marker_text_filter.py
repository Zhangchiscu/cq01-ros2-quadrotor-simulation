#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
class V9MarkerTextFilter(Node):
    def __init__(self):
        super().__init__('cq01_v9_marker_text_filter');self.declare_parameter('input_topic','/markers_in');self.declare_parameter('output_topic','/markers_out')
        inp=str(self.get_parameter('input_topic').value);out=str(self.get_parameter('output_topic').value);self.pub=self.create_publisher(MarkerArray,out,10);self.create_subscription(MarkerArray,inp,self.cb,10)
    def cb(self,msg):
        out=MarkerArray();out.markers=[m for m in msg.markers if m.action==Marker.DELETEALL or m.type!=Marker.TEXT_VIEW_FACING];self.pub.publish(out)
def main(args=None):
    rclpy.init(args=args);n=V9MarkerTextFilter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
