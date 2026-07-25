#!/usr/bin/env python3
"""Predict local dynamic collisions, hold, replan, and resume safely."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


Point3 = Tuple[float, float, float]
IMPLEMENTATION_REVISION = "local_horizon_v2_verified"


class ReplanningSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("replanning_supervisor")

        self.declare_parameter(
            "prediction_trigger_clearance_m",
            1.00,
        )
        self.declare_parameter("drone_radius_m", 0.15)
        self.declare_parameter(
            "physical_emergency_clearance_m",
            0.85,
        )
        self.declare_parameter(
            "resume_predicted_clearance_m",
            1.00,
        )
        self.declare_parameter(
            "resume_current_clearance_m",
            0.85,
        )
        self.declare_parameter(
            "resume_safe_hold_s",
            0.60,
        )
        self.declare_parameter(
            "prediction_path_lookahead_m",
            2.50,
        )
        self.declare_parameter(
            "resume_path_lookahead_m",
            1.50,
        )
        self.declare_parameter(
            "hold_before_replan_s",
            0.10,
        )
        self.declare_parameter(
            "resume_delay_s",
            0.20,
        )
        self.declare_parameter(
            "minimum_replan_interval_s",
            0.75,
        )
        self.declare_parameter(
            "maximum_replans",
            10,
        )
        self.declare_parameter(
            "replan_retry_delay_s",
            1.00,
        )
        self.declare_parameter(
            "retry_current_clearance_m",
            0.85,
        )
        self.declare_parameter(
            "publish_hz",
            20.0,
        )

        self.trigger_clearance = float(
            self.get_parameter(
                "prediction_trigger_clearance_m"
            ).value
        )
        self.drone_radius = float(
            self.get_parameter(
                "drone_radius_m"
            ).value
        )
        self.physical_emergency_clearance = float(
            self.get_parameter(
                "physical_emergency_clearance_m"
            ).value
        )
        self.resume_predicted_clearance = float(
            self.get_parameter(
                "resume_predicted_clearance_m"
            ).value
        )
        self.resume_current_clearance = float(
            self.get_parameter(
                "resume_current_clearance_m"
            ).value
        )
        self.resume_safe_hold = float(
            self.get_parameter(
                "resume_safe_hold_s"
            ).value
        )
        self.prediction_path_lookahead = float(
            self.get_parameter(
                "prediction_path_lookahead_m"
            ).value
        )
        self.resume_path_lookahead = float(
            self.get_parameter(
                "resume_path_lookahead_m"
            ).value
        )
        self.hold_before_replan = float(
            self.get_parameter(
                "hold_before_replan_s"
            ).value
        )
        self.resume_delay = float(
            self.get_parameter(
                "resume_delay_s"
            ).value
        )
        self.minimum_replan_interval = float(
            self.get_parameter(
                "minimum_replan_interval_s"
            ).value
        )
        self.maximum_replans = int(
            self.get_parameter(
                "maximum_replans"
            ).value
        )
        self.replan_retry_delay = float(
            self.get_parameter(
                "replan_retry_delay_s"
            ).value
        )
        self.retry_current_clearance = float(
            self.get_parameter(
                "retry_current_clearance_m"
            ).value
        )
        publish_hz = float(
            self.get_parameter(
                "publish_hz"
            ).value
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_nominal_goal: Optional[PoseStamped] = None
        self.final_goal: Optional[PoseStamped] = None
        self.reference_path: List[Point3] = []
        self.dynamic_state: Optional[Dict[str, object]] = None
        self.trajectory_collision_checked = False

        self.state = "waiting_for_trajectory"
        self.hold_point: Optional[Point3] = None
        self.hold_started: Optional[float] = None
        self.replan_requested_at: Optional[float] = None
        self.resume_at: Optional[float] = None
        self.retry_at: Optional[float] = None
        self.safe_resume_started: Optional[float] = None
        self.last_replan_completed_at = -float("inf")

        self.predicted_collision_count = 0
        self.hold_count = 0
        self.replan_count = 0
        self.successful_replans = 0
        self.failed_replans = 0
        self.recovery_wait_count = 0
        self.last_replan_latency_ms: Optional[float] = None
        self.minimum_predicted_clearance = float("inf")
        self.last_event = "initialized"

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.control_goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal",
            10,
        )
        self.replan_goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal_raw",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/drone/replanning/status",
            transient_qos,
        )

        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            50,
        )
        self.create_subscription(
            PoseStamped,
            "/drone/goal_nominal",
            self.nominal_goal_callback,
            20,
        )
        self.create_subscription(
            PoseStamped,
            "/drone/goal_raw",
            self.final_goal_callback,
            10,
        )
        self.create_subscription(
            Path,
            "/drone/trajectory_reference",
            self.reference_path_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/trajectory/status",
            self.trajectory_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/planner_3d/status",
            self.planner_status_callback,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/drone/dynamic_obstacle/state",
            self.dynamic_state_callback,
            transient_qos,
        )

        self.timer = self.create_timer(
            1.0 / publish_hz,
            self.control_step,
        )
        self.status_timer = self.create_timer(
            0.5,
            self.publish_status,
        )

        self.get_logger().info(
            "Dynamic replanning supervisor started: "
            f"revision={IMPLEMENTATION_REVISION}"
        )

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.latest_odom = message

    def nominal_goal_callback(
        self,
        message: PoseStamped,
    ) -> None:
        self.latest_nominal_goal = message

    def final_goal_callback(
        self,
        message: PoseStamped,
    ) -> None:
        self.final_goal = message

    def dynamic_state_callback(
        self,
        message: String,
    ) -> None:
        self.dynamic_state = json.loads(
            message.data
        )

    def accept_ready_trajectory(
        self,
        source: str,
        collision_checked: bool = True,
    ) -> None:
        now = time.monotonic()
        self.trajectory_collision_checked = bool(
            collision_checked
        )

        if (
            self.state == "waiting_for_new_trajectory"
            and self.replan_requested_at is not None
        ):
            self.last_replan_latency_ms = (
                now - self.replan_requested_at
            ) * 1000.0
            self.successful_replans += 1
            self.last_replan_completed_at = now
            self.retry_at = None
            self.safe_resume_started = None
            self.state = "waiting_for_safe_resume"
            self.last_event = (
                f"replan_completed_safety_gate_{source}"
            )
            self.get_logger().info(
                "Online replan completed: "
                f"latency={self.last_replan_latency_ms:.1f} ms, "
                f"source={source}. Waiting for local safety gate."
            )
            return

        if self.state == "waiting_for_trajectory":
            self.state = "following"
            self.last_event = (
                f"initial_trajectory_ready_{source}"
            )
            self.get_logger().info(
                "Initial collision-checked trajectory ready; "
                f"source={source}; following."
            )

    def reference_path_callback(
        self,
        message: Path,
    ) -> None:
        self.reference_path = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            for pose in message.poses
        ]

        # trajectory_executor publishes /drone/trajectory_reference
        # only after the processed path passes its voxel collision check.
        # It is therefore a reliable second readiness signal and removes
        # the startup race observed in the early-entry condition.
        if (
            len(self.reference_path) >= 2
            and self.state in {
                "waiting_for_trajectory",
                "waiting_for_new_trajectory",
            }
        ):
            self.accept_ready_trajectory(
                source="reference_path",
                collision_checked=True,
            )

    def enter_wait_for_clearance(
        self,
        reason: str,
    ) -> None:
        if self.state == "waiting_for_obstacle_clearance":
            return

        current = self.current_position()
        if current is not None:
            self.hold_point = current

        now = time.monotonic()
        self.trajectory_collision_checked = False
        self.failed_replans += 1
        self.recovery_wait_count += 1
        self.retry_at = now + self.replan_retry_delay
        self.state = "waiting_for_obstacle_clearance"
        self.last_event = reason

        self.get_logger().warn(
            f"{reason}; holding and retrying after clearance "
            f"recovers (failed attempts={self.failed_replans})"
        )

    def planner_status_callback(
        self,
        message: String,
    ) -> None:
        data = json.loads(message.data)

        if (
            data.get("state") == "planning_failed"
            and self.state in {
                "waiting_for_trajectory",
                "waiting_for_new_trajectory",
            }
        ):
            reason = str(
                data.get(
                    "reason",
                    "planner_failed",
                )
            )
            self.enter_wait_for_clearance(
                f"planner_{reason}"
            )

    def trajectory_status_callback(
        self,
        message: String,
    ) -> None:
        data = json.loads(message.data)
        state = data.get("state")

        if (
            state == "trajectory_failed"
            and self.state in {
                "waiting_for_trajectory",
                "waiting_for_new_trajectory",
            }
        ):
            reason = str(
                data.get(
                    "reason",
                    "trajectory_collision_rejected",
                )
            )
            self.enter_wait_for_clearance(
                f"trajectory_{reason}"
            )
            return

        if (
            state == "trajectory_ready"
            and self.state in {
                "waiting_for_trajectory",
                "waiting_for_new_trajectory",
            }
        ):
            self.accept_ready_trajectory(
                source="status",
                collision_checked=bool(
                    data.get(
                        "collision_checked",
                        True,
                    )
                ),
            )

    def current_position(
        self,
    ) -> Optional[Point3]:
        if self.latest_odom is None:
            return None

        position = self.latest_odom.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def cylinder_clearance(
        self,
        point: Sequence[float],
        center: Sequence[float],
        radius: float,
        height: float,
    ) -> float:
        horizontal = (
            math.hypot(
                float(point[0]) - float(center[0]),
                float(point[1]) - float(center[1]),
            )
            - radius
            - self.drone_radius
        )
        vertical = (
            abs(
                float(point[2]) - float(center[2])
            )
            - 0.5 * height
            - self.drone_radius
        )

        if horizontal > 0.0 and vertical > 0.0:
            return math.hypot(
                horizontal,
                vertical,
            )
        if horizontal > 0.0:
            return horizontal
        if vertical > 0.0:
            return vertical
        return max(
            horizontal,
            vertical,
        )

    def live_obstacle_clearance(
        self,
    ) -> float:
        point = self.current_position()
        if point is None or self.dynamic_state is None:
            return float("inf")

        return self.cylinder_clearance(
            point,
            self.dynamic_state["position"],
            float(
                self.dynamic_state["radius_m"]
            ),
            float(
                self.dynamic_state["height_m"]
            ),
        )

    def current_obstacle_clearance(
        self,
    ) -> float:
        point = (
            self.hold_point
            if self.hold_point is not None
            else self.current_position()
        )
        if point is None or self.dynamic_state is None:
            return float("inf")

        return self.cylinder_clearance(
            point,
            self.dynamic_state["position"],
            float(
                self.dynamic_state["radius_m"]
            ),
            float(
                self.dynamic_state["height_m"]
            ),
        )

    def remaining_path(
        self,
    ) -> List[Point3]:
        current = self.current_position()
        if current is None or not self.reference_path:
            return []

        nearest_index = min(
            range(len(self.reference_path)),
            key=lambda index: math.dist(
                current,
                self.reference_path[index],
            ),
        )
        return self.reference_path[
            nearest_index:
        ]

    def path_within_lookahead(
        self,
        maximum_distance_m: float,
    ) -> List[Point3]:
        path = self.remaining_path()
        if not path:
            return []

        if (
            maximum_distance_m <= 0.0
            or len(path) == 1
        ):
            return [path[0]]

        output: List[Point3] = [path[0]]
        accumulated = 0.0

        for first, second in zip(
            path[:-1],
            path[1:],
        ):
            segment_length = math.dist(
                first,
                second,
            )
            if segment_length <= 1.0e-9:
                continue

            remaining = (
                maximum_distance_m - accumulated
            )
            if remaining <= 0.0:
                break

            if segment_length <= remaining:
                output.append(second)
                accumulated += segment_length
                continue

            ratio = remaining / segment_length
            output.append((
                first[0]
                + ratio * (second[0] - first[0]),
                first[1]
                + ratio * (second[1] - first[1]),
                first[2]
                + ratio * (second[2] - first[2]),
            ))
            break

        return output

    def predicted_clearance(
        self,
        lookahead_m: Optional[float] = None,
    ) -> float:
        if (
            self.dynamic_state is None
            or not bool(
                self.dynamic_state.get(
                    "prediction_active",
                    False,
                )
            )
        ):
            return float("inf")

        path = (
            self.remaining_path()
            if lookahead_m is None
            else self.path_within_lookahead(
                lookahead_m
            )
        )
        if not path:
            return float("inf")

        radius = float(
            self.dynamic_state["radius_m"]
        )
        height = float(
            self.dynamic_state["height_m"]
        )
        predictions = self.dynamic_state.get(
            "predicted_positions",
            [],
        )

        minimum = float("inf")
        for prediction in predictions:
            center = prediction["position"]
            for point in path:
                minimum = min(
                    minimum,
                    self.cylinder_clearance(
                        point,
                        center,
                        radius,
                        height,
                    ),
                )

        return minimum

    def safe_to_resume(
        self,
    ) -> bool:
        local_path = self.path_within_lookahead(
            self.resume_path_lookahead
        )
        local_predicted_clearance = (
            self.predicted_clearance(
                self.resume_path_lookahead
            )
        )

        return (
            self.trajectory_collision_checked
            and bool(local_path)
            and self.live_obstacle_clearance()
            >= self.resume_current_clearance
            and local_predicted_clearance
            >= self.resume_predicted_clearance
        )

    def copy_goal(
        self,
        source: PoseStamped,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = (
            source.header.frame_id or "map"
        )
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.pose = source.pose
        return message

    def hold_goal_message(
        self,
        point: Point3,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.pose.position.x = point[0]
        message.pose.position.y = point[1]
        message.pose.position.z = point[2]
        message.pose.orientation.w = 1.0
        return message

    def start_hold(
        self,
        clearance: float,
    ) -> None:
        current = self.current_position()
        if current is None:
            return

        self.hold_point = current
        self.hold_started = time.monotonic()
        self.state = "holding_before_replan"
        self.predicted_collision_count += 1
        self.hold_count += 1
        self.last_event = "collision_predicted"
        self.minimum_predicted_clearance = min(
            self.minimum_predicted_clearance,
            clearance,
        )

        self.get_logger().warn(
            "Predicted trajectory collision: "
            f"clearance={clearance:.3f} m. "
            "Holding before online replan."
        )

    def request_replan(
        self,
    ) -> None:
        if self.final_goal is None:
            return

        if self.replan_count >= self.maximum_replans:
            self.state = "emergency_hold"
            self.last_event = (
                "maximum_replans_reached"
            )
            self.get_logger().error(
                "Maximum online replan attempts reached; "
                "remaining in emergency hold."
            )
            return

        self.replan_count += 1
        self.trajectory_collision_checked = False
        self.replan_requested_at = time.monotonic()
        self.state = "waiting_for_new_trajectory"
        self.last_event = "replan_requested"
        self.replan_goal_publisher.publish(
            self.copy_goal(self.final_goal)
        )

        self.get_logger().info(
            "Online 3D A* replan requested "
            f"(count={self.replan_count})"
        )

    def publish_control_hold(
        self,
    ) -> None:
        if self.hold_point is None:
            current = self.current_position()
            if current is not None:
                self.hold_point = current

        if self.hold_point is not None:
            self.control_goal_publisher.publish(
                self.hold_goal_message(
                    self.hold_point
                )
            )

    def control_step(
        self,
    ) -> None:
        now = time.monotonic()

        if self.state == "following":
            predicted_clearance = (
                self.predicted_clearance(
                    self.prediction_path_lookahead
                )
            )
            current_clearance = (
                self.live_obstacle_clearance()
            )
            effective_clearance = min(
                predicted_clearance,
                current_clearance,
            )
            self.minimum_predicted_clearance = min(
                self.minimum_predicted_clearance,
                predicted_clearance,
            )

            emergency = (
                current_clearance
                <= self.physical_emergency_clearance
            )
            predicted_risk = (
                predicted_clearance
                <= self.trigger_clearance
                and self.replan_count
                < self.maximum_replans
                and now
                - self.last_replan_completed_at
                >= self.minimum_replan_interval
            )

            if emergency or predicted_risk:
                self.start_hold(
                    effective_clearance
                )
            elif self.latest_nominal_goal is not None:
                self.control_goal_publisher.publish(
                    self.copy_goal(
                        self.latest_nominal_goal
                    )
                )

        elif self.state == "holding_before_replan":
            self.publish_control_hold()
            if (
                self.hold_started is not None
                and now - self.hold_started
                >= self.hold_before_replan
            ):
                self.request_replan()

        elif self.state == "waiting_for_new_trajectory":
            self.publish_control_hold()

        elif self.state == "waiting_for_obstacle_clearance":
            self.publish_control_hold()
            if (
                self.retry_at is not None
                and now >= self.retry_at
                and self.current_obstacle_clearance()
                >= self.retry_current_clearance
            ):
                self.last_event = (
                    "clearance_recovered_retrying"
                )
                self.request_replan()

        elif self.state == "waiting_for_safe_resume":
            self.publish_control_hold()

            if self.safe_to_resume():
                if self.safe_resume_started is None:
                    self.safe_resume_started = now
                    self.last_event = (
                        "safe_resume_window_started"
                    )
                elif (
                    now - self.safe_resume_started
                    >= self.resume_safe_hold
                ):
                    self.resume_at = (
                        now + self.resume_delay
                    )
                    self.state = "resume_delay"
                    self.last_event = (
                        "safe_resume_confirmed"
                    )
                    self.get_logger().info(
                        "Local current and predicted clearances "
                        "remained safe; preparing to resume."
                    )
            else:
                self.safe_resume_started = None

        elif self.state == "resume_delay":
            self.publish_control_hold()
            if (
                self.resume_at is not None
                and now >= self.resume_at
            ):
                self.state = "following"
                self.last_event = "flight_resumed"
                self.get_logger().info(
                    "New trajectory accepted; "
                    "flight resumed."
                )

        elif self.state == "waiting_for_trajectory":
            self.publish_control_hold()

        elif self.state == "emergency_hold":
            self.publish_control_hold()

    def publish_status(
        self,
    ) -> None:
        local_predicted = self.predicted_clearance(
            self.prediction_path_lookahead
        )
        resume_predicted = self.predicted_clearance(
            self.resume_path_lookahead
        )
        full_route_predicted = (
            self.predicted_clearance()
        )
        finite_minimum = (
            self.minimum_predicted_clearance
            if math.isfinite(
                self.minimum_predicted_clearance
            )
            else None
        )

        message = String()
        message.data = json.dumps({
            "implementation_revision": (
                IMPLEMENTATION_REVISION
            ),
            "state": self.state,
            "success": True,
            "last_event": self.last_event,
            "predicted_clearance_m": (
                local_predicted
                if math.isfinite(local_predicted)
                else None
            ),
            "resume_local_predicted_clearance_m": (
                resume_predicted
                if math.isfinite(resume_predicted)
                else None
            ),
            "full_route_predicted_clearance_m": (
                full_route_predicted
                if math.isfinite(
                    full_route_predicted
                )
                else None
            ),
            "minimum_predicted_clearance_m": (
                finite_minimum
            ),
            "prediction_path_lookahead_m": (
                self.prediction_path_lookahead
            ),
            "resume_path_lookahead_m": (
                self.resume_path_lookahead
            ),
            "trajectory_collision_checked": (
                self.trajectory_collision_checked
            ),
            "predicted_collision_count": (
                self.predicted_collision_count
            ),
            "hold_count": self.hold_count,
            "replan_count": self.replan_count,
            "successful_replans": (
                self.successful_replans
            ),
            "failed_replans": self.failed_replans,
            "recovery_wait_count": (
                self.recovery_wait_count
            ),
            "current_obstacle_clearance_m": (
                self.current_obstacle_clearance()
            ),
            "live_obstacle_clearance_m": (
                self.live_obstacle_clearance()
            ),
            "prediction_trigger_clearance_m": (
                self.trigger_clearance
            ),
            "physical_emergency_clearance_m": (
                self.physical_emergency_clearance
            ),
            "resume_predicted_clearance_m": (
                self.resume_predicted_clearance
            ),
            "resume_current_clearance_m": (
                self.resume_current_clearance
            ),
            "safe_to_resume": self.safe_to_resume(),
            "maximum_replans": self.maximum_replans,
            "last_replan_latency_ms": (
                self.last_replan_latency_ms
            ),
        }, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplanningSupervisor()

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
