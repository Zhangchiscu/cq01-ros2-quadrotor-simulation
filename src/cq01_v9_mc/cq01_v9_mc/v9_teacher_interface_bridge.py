#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


class TeacherInterfaceBridge(Node):
    """Expose teacher-facing aliases without changing the control loop.

    Inputs remain the already validated CQ01 interfaces:
      /drone/voxel_cloud
      /drone/planned_path_3d

    Outputs are compatibility/evidence topics only:
      /map/obstacles
      /drone/reference
      /drone/waypoints
      /drone/safe_goal
    """

    def __init__(self) -> None:
        super().__init__("cq01_v9_teacher_interface_bridge")

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.obstacles_pub = self.create_publisher(
            PointCloud2,
            "/map/obstacles",
            latched,
        )
        self.reference_pub = self.create_publisher(
            Path,
            "/drone/reference",
            latched,
        )
        self.waypoints_pub = self.create_publisher(
            Path,
            "/drone/waypoints",
            latched,
        )
        self.safe_goal_pub = self.create_publisher(
            PoseStamped,
            "/drone/safe_goal",
            latched,
        )

        self.create_subscription(
            PointCloud2,
            "/drone/voxel_cloud",
            self.obstacles_cb,
            latched,
        )
        self.create_subscription(
            Path,
            "/drone/planned_path_3d",
            self.path_cb,
            latched,
        )

        self.obstacle_messages = 0
        self.path_messages = 0
        self.get_logger().warning(
            "Teacher interface bridge ready; "
            "publishes aliases only and sends no control commands."
        )

    def obstacles_cb(self, message: PointCloud2) -> None:
        self.obstacle_messages += 1
        self.obstacles_pub.publish(message)

    def path_cb(self, message: Path) -> None:
        self.path_messages += 1
        self.reference_pub.publish(message)
        self.waypoints_pub.publish(message)

        if not message.poses:
            return

        safe_goal = PoseStamped()
        safe_goal.header = message.header
        safe_goal.pose = message.poses[-1].pose
        self.safe_goal_pub.publish(safe_goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeacherInterfaceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
