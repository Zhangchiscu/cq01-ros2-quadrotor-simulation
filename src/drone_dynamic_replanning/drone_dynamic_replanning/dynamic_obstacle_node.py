#!/usr/bin/env python3
"""Publish a deterministic moving cylindrical obstacle and its future motion."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


Point3 = Tuple[float, float, float]


class DynamicObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle")

        self.declare_parameter("x_m", 4.5)
        self.declare_parameter("y_start_m", -2.6)
        self.declare_parameter("y_center_m", 0.0)
        self.declare_parameter("y_end_m", 2.6)
        self.declare_parameter("z_m", 1.5)
        self.declare_parameter("radius_m", 0.55)
        self.declare_parameter("height_m", 2.40)
        self.declare_parameter("start_delay_s", 3.0)
        self.declare_parameter("speed_mps", 0.55)
        self.declare_parameter("center_dwell_s", 8.0)
        self.declare_parameter("prediction_horizon_s", 6.0)
        self.declare_parameter("prediction_step_s", 0.25)
        self.declare_parameter("publish_hz", 20.0)

        self.x = float(self.get_parameter("x_m").value)
        self.y_start = float(
            self.get_parameter("y_start_m").value
        )
        self.y_center = float(
            self.get_parameter("y_center_m").value
        )
        self.y_end = float(
            self.get_parameter("y_end_m").value
        )
        self.z = float(self.get_parameter("z_m").value)
        self.radius = float(
            self.get_parameter("radius_m").value
        )
        self.height = float(
            self.get_parameter("height_m").value
        )
        self.start_delay = float(
            self.get_parameter("start_delay_s").value
        )
        self.speed = float(
            self.get_parameter("speed_mps").value
        )
        self.center_dwell = float(
            self.get_parameter("center_dwell_s").value
        )
        self.prediction_horizon = float(
            self.get_parameter(
                "prediction_horizon_s"
            ).value
        )
        self.prediction_step = float(
            self.get_parameter("prediction_step_s").value
        )
        publish_hz = float(
            self.get_parameter("publish_hz").value
        )

        if self.speed <= 0.0:
            raise ValueError("speed_mps must be positive")
        if self.radius <= 0.0 or self.height <= 0.0:
            raise ValueError(
                "Obstacle radius and height must be positive"
            )

        self.first_leg_duration = (
            abs(self.y_center - self.y_start)
            / self.speed
        )
        self.second_leg_duration = (
            abs(self.y_end - self.y_center)
            / self.speed
        )
        self.motion_start = self.start_delay
        self.center_arrival = (
            self.motion_start + self.first_leg_duration
        )
        self.center_departure = (
            self.center_arrival + self.center_dwell
        )
        self.end_arrival = (
            self.center_departure + self.second_leg_duration
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/drone/dynamic_obstacle/pose",
            transient_qos,
        )
        self.state_publisher = self.create_publisher(
            String,
            "/drone/dynamic_obstacle/state",
            transient_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/drone/dynamic_obstacle/markers",
            transient_qos,
        )

        self.started = time.monotonic()
        self.timer = self.create_timer(
            1.0 / publish_hz,
            self.publish_state,
        )
        self.publish_state()

        self.get_logger().info(
            "Dynamic obstacle ready: "
            f"x={self.x:.2f}, "
            f"y={self.y_start:.2f}->{self.y_center:.2f}"
            f"->{self.y_end:.2f}, "
            f"radius={self.radius:.2f} m"
        )

    def interpolate(
        self,
        start: float,
        end: float,
        ratio: float,
    ) -> float:
        return start + ratio * (end - start)

    def state_at(
        self,
        elapsed: float,
    ) -> Dict[str, object]:
        if elapsed < self.motion_start:
            phase = "waiting"
            y = self.y_start
            velocity_y = 0.0
        elif elapsed < self.center_arrival:
            phase = "crossing_to_center"
            ratio = (
                elapsed - self.motion_start
            ) / self.first_leg_duration
            y = self.interpolate(
                self.y_start,
                self.y_center,
                ratio,
            )
            velocity_y = math.copysign(
                self.speed,
                self.y_center - self.y_start,
            )
        elif elapsed < self.center_departure:
            phase = "center_dwell"
            y = self.y_center
            velocity_y = 0.0
        elif elapsed < self.end_arrival:
            phase = "crossing_to_end"
            ratio = (
                elapsed - self.center_departure
            ) / self.second_leg_duration
            y = self.interpolate(
                self.y_center,
                self.y_end,
                ratio,
            )
            velocity_y = math.copysign(
                self.speed,
                self.y_end - self.y_center,
            )
        else:
            phase = "finished"
            y = self.y_end
            velocity_y = 0.0

        return {
            "phase": phase,
            "position": [self.x, y, self.z],
            "velocity": [0.0, velocity_y, 0.0],
        }

    def predicted_positions(
        self,
        elapsed: float,
        prediction_active: bool,
    ) -> List[Dict[str, object]]:
        if not prediction_active:
            current = self.state_at(elapsed)
            return [{
                "t_s": 0.0,
                "position": current["position"],
                "phase": current["phase"],
            }]

        sample_count = max(
            1,
            int(
                math.ceil(
                    self.prediction_horizon
                    / self.prediction_step
                )
            ),
        )

        predictions: List[Dict[str, object]] = []
        for index in range(sample_count + 1):
            future_t = min(
                self.prediction_horizon,
                index * self.prediction_step,
            )
            state = self.state_at(elapsed + future_t)
            predictions.append({
                "t_s": future_t,
                "position": state["position"],
                "phase": state["phase"],
            })

        return predictions

    def build_markers(
        self,
        position: Point3,
        predictions: List[Dict[str, object]],
        stamp,
    ) -> MarkerArray:
        markers = MarkerArray()

        obstacle = Marker()
        obstacle.header.frame_id = "map"
        obstacle.header.stamp = stamp
        obstacle.ns = "dynamic_obstacle"
        obstacle.id = 0
        obstacle.type = Marker.CYLINDER
        obstacle.action = Marker.ADD
        obstacle.pose.position.x = position[0]
        obstacle.pose.position.y = position[1]
        obstacle.pose.position.z = position[2]
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = 2.0 * self.radius
        obstacle.scale.y = 2.0 * self.radius
        obstacle.scale.z = self.height
        obstacle.color.r = 0.95
        obstacle.color.g = 0.12
        obstacle.color.b = 0.08
        obstacle.color.a = 0.90
        markers.markers.append(obstacle)

        safety = Marker()
        safety.header = obstacle.header
        safety.ns = "dynamic_obstacle_safety"
        safety.id = 1
        safety.type = Marker.CYLINDER
        safety.action = Marker.ADD
        safety.pose = obstacle.pose
        safety.scale.x = 2.0 * (self.radius + 0.30)
        safety.scale.y = 2.0 * (self.radius + 0.30)
        safety.scale.z = self.height + 0.60
        safety.color.r = 1.00
        safety.color.g = 0.55
        safety.color.b = 0.05
        safety.color.a = 0.18
        markers.markers.append(safety)

        predicted_path = Marker()
        predicted_path.header = obstacle.header
        predicted_path.ns = "dynamic_obstacle_prediction"
        predicted_path.id = 2
        predicted_path.type = Marker.LINE_STRIP
        predicted_path.action = Marker.ADD
        predicted_path.scale.x = 0.08
        predicted_path.color.r = 1.00
        predicted_path.color.g = 0.72
        predicted_path.color.b = 0.05
        predicted_path.color.a = 0.95

        for prediction in predictions:
            point = Point()
            predicted = prediction["position"]
            point.x = float(predicted[0])
            point.y = float(predicted[1])
            point.z = float(predicted[2])
            predicted_path.points.append(point)

        markers.markers.append(predicted_path)
        return markers

    def publish_state(self) -> None:
        elapsed = time.monotonic() - self.started
        state = self.state_at(elapsed)
        prediction_active = (
            state["phase"] != "waiting"
        )
        predictions = self.predicted_positions(
            elapsed,
            prediction_active,
        )

        stamp = self.get_clock().now().to_msg()
        position = tuple(
            float(value)
            for value in state["position"]
        )

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)

        payload = {
            "elapsed_s": elapsed,
            "phase": state["phase"],
            "position": list(position),
            "velocity": state["velocity"],
            "radius_m": self.radius,
            "height_m": self.height,
            "prediction_active": prediction_active,
            "prediction_horizon_s": self.prediction_horizon,
            "predicted_positions": predictions,
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.state_publisher.publish(message)

        self.marker_publisher.publish(
            self.build_markers(
                position,
                predictions,
                stamp,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleNode()

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
