#!/usr/bin/env python3
"""Publish the CQ-01 goal after the map, planner and odometry are ready."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class CQ01Mission(Node):
    def __init__(self) -> None:
        super().__init__("cq01_eling_liziba_mission")

        share = Path(get_package_share_directory("drone_chongqing_cq01"))
        manifest_path = share / "runtime_data" / "cq01_manifest.json"
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.goal = self.manifest["simulation"]["final_goal"]

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/chongqing/cq01/mission_status",
            qos,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            20,
        )
        self.create_subscription(
            String,
            "/drone/planner_3d/status",
            self.planner_callback,
            qos,
        )
        self.create_subscription(
            String,
            "/chongqing/cq01/map_status",
            self.map_callback,
            qos,
        )

        self.have_odom = False
        self.map_ready = False
        self.planner_ready = False
        self.goal_sent = False
        self.start_time = time.monotonic()
        self.timer = self.create_timer(0.5, self.step)

    def odom_callback(self, _: Odometry) -> None:
        self.have_odom = True

    def map_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            self.map_ready = bool(data.get("success"))
        except json.JSONDecodeError:
            self.map_ready = False

    def planner_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return

        state = str(data.get("state", ""))
        if state == "ready":
            self.planner_ready = True
        if state == "path_ready":
            self.publish_status("path_ready")
        if state == "final_waypoint_reached":
            self.publish_status("mission_complete")

    def step(self) -> None:
        if self.goal_sent:
            return

        if time.monotonic() - self.start_time < 2.0:
            return

        if not (self.have_odom and self.map_ready and self.planner_ready):
            return

        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(self.goal[0])
        message.pose.position.y = float(self.goal[1])
        message.pose.position.z = float(self.goal[2])
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)

        self.goal_sent = True
        self.publish_status("goal_sent")
        self.get_logger().info(
            "CQ-01 goal sent: "
            f"x={self.goal[0]:.2f}, "
            f"y={self.goal[1]:.2f}, "
            f"z={self.goal[2]:.2f}"
        )

    def publish_status(self, state: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "scenario": self.manifest["scenario_cn"],
                "goal": self.goal,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CQ01Mission()
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
