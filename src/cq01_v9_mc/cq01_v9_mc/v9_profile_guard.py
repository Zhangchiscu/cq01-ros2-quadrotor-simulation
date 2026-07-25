#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
import yaml
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class V9ProfileGuard(Node):
    '''Verify plant and controller parameters through native ROS services.'''

    PARAMETER_NAMES = (
        "mass",
        "inertia_xx",
        "inertia_yy",
        "inertia_zz",
        "arm_length",
        "thrust_coefficient",
        "torque_coefficient",
        "maximum_rpm",
    )

    def __init__(self) -> None:
        super().__init__("cq01_v9_profile_guard")

        self.declare_parameter("profile", "nominal")
        self.declare_parameter("profile_file", "")
        self.declare_parameter(
            "results_root",
            os.path.expanduser(
                "~/drone_task/your_drone_sim_ws/"
                "results/cq01_bonus_suite"
            ),
        )
        self.declare_parameter("initial_delay_s", 3.0)
        self.declare_parameter("maximum_wait_s", 45.0)

        self.profile = str(
            self.get_parameter("profile").value
        )
        self.profile_file = Path(
            os.path.expanduser(
                str(self.get_parameter("profile_file").value)
            )
        )
        self.results_root = Path(
            os.path.expanduser(
                str(self.get_parameter("results_root").value)
            )
        )
        self.initial_delay_s = float(
            self.get_parameter("initial_delay_s").value
        )
        self.maximum_wait_s = float(
            self.get_parameter("maximum_wait_s").value
        )

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String,
            "/cq01_v9/profile/status",
            transient,
        )

        self.parameter_clients = {
            "dynamics": self.create_client(
                GetParameters,
                "/quadrotor_dynamics/get_parameters",
            ),
            "controller": self.create_client(
                GetParameters,
                "/quadrotor_controller/get_parameters",
            ),
        }
        self.futures: Dict[str, Any] = {}
        self.actual: Dict[str, Dict[str, float]] = {}
        self.expected: Dict[str, Any] = {}
        self.result: Optional[Dict[str, Any]] = None
        self.started_s = self.now_s()

        if not self.profile_file.is_file():
            self.result = self.failure(
                f"profile_file_not_found:{self.profile_file}"
            )
            self.finish()
        else:
            self.expected = yaml.safe_load(
                self.profile_file.read_text(encoding="utf-8")
            )
            self.get_logger().warning(
                "PROFILE GUARD NATIVE waiting: "
                f"profile={self.profile}; "
                f"file={self.profile_file}"
            )

        self.timer = self.create_timer(0.25, self.tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def failure(self, reason: str) -> Dict[str, Any]:
        return {
            "state": "profile_failed",
            "pass": False,
            "profile": self.profile,
            "profile_file": str(self.profile_file),
            "verification_method":
                "rcl_interfaces/GetParameters",
            "failure_reason": reason,
            "failed_criteria": [reason],
            "checks": {},
            "expected": {},
            "actual": self.actual,
        }

    @staticmethod
    def numeric(value) -> Optional[float]:
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value.type == ParameterType.PARAMETER_INTEGER:
            return float(value.integer_value)
        return None

    def services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in self.parameter_clients.values()
        )

    def issue_requests(self) -> None:
        request = GetParameters.Request()
        request.names = list(self.PARAMETER_NAMES)
        for key, client in self.parameter_clients.items():
            self.futures[key] = client.call_async(request)
        self.get_logger().info(
            "PROFILE GUARD native requests issued."
        )

    def collect(self) -> bool:
        if not self.futures:
            return False
        if not all(
            future.done()
            for future in self.futures.values()
        ):
            return False

        for node_key, future in self.futures.items():
            try:
                response = future.result()
            except Exception as exc:
                self.result = self.failure(
                    "parameter_request_exception:"
                    f"{node_key}:{exc}"
                )
                return True

            if (
                response is None
                or len(response.values)
                != len(self.PARAMETER_NAMES)
            ):
                self.result = self.failure(
                    f"invalid_parameter_response:{node_key}"
                )
                return True

            values: Dict[str, float] = {}
            for parameter_name, value in zip(
                self.PARAMETER_NAMES,
                response.values,
            ):
                converted = self.numeric(value)
                if converted is None:
                    self.result = self.failure(
                        "non_numeric_parameter:"
                        f"{node_key}:{parameter_name}"
                    )
                    return True
                values[parameter_name] = converted
            self.actual[node_key] = values

        self.result = self.verify()
        return True

    @staticmethod
    def close(first: float, second: float) -> bool:
        tolerance = max(
            1.0e-9,
            1.0e-6
            * max(abs(first), abs(second), 1.0),
        )
        return math.isclose(
            first,
            second,
            rel_tol=1.0e-6,
            abs_tol=tolerance,
        )

    def verify(self) -> Dict[str, Any]:
        expected = {
            "dynamics": {
                name: float(
                    self.expected["quadrotor_dynamics"]
                    ["ros__parameters"][name]
                )
                for name in self.PARAMETER_NAMES
            },
            "controller": {
                name: float(
                    self.expected["quadrotor_controller"]
                    ["ros__parameters"][name]
                )
                for name in self.PARAMETER_NAMES
            },
        }

        checks: Dict[str, bool] = {}
        for name in self.PARAMETER_NAMES:
            checks[f"dynamics_{name}"] = self.close(
                self.actual["dynamics"][name],
                expected["dynamics"][name],
            )
            checks[f"controller_{name}"] = self.close(
                self.actual["controller"][name],
                expected["controller"][name],
            )
            checks[f"plant_controller_match_{name}"] = (
                self.close(
                    self.actual["dynamics"][name],
                    self.actual["controller"][name],
                )
            )

        failed = [
            name
            for name, passed in checks.items()
            if passed is not True
        ]

        return {
            "state": (
                "profile_verified"
                if not failed
                else "profile_mismatch"
            ),
            "pass": not failed,
            "profile": self.profile,
            "profile_file": str(self.profile_file),
            "verification_method":
                "rcl_interfaces/GetParameters",
            "failure_reason": (
                ""
                if not failed
                else "parameter_mismatch"
            ),
            "failed_criteria": failed,
            "checks": checks,
            "expected": expected,
            "actual": self.actual,
        }

    def save(self) -> None:
        if self.result is None:
            return
        self.results_root.mkdir(
            parents=True,
            exist_ok=True,
        )
        output = (
            self.results_root
            / (
                f"profile_guard_{self.profile}_"
                + datetime.now().strftime(
                    "%Y%m%d_%H%M%S"
                )
                + ".json"
            )
        )
        self.result["result_file"] = str(output)
        output.write_text(
            json.dumps(
                self.result,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

    def publish(self) -> None:
        if self.result is None:
            return
        message = String()
        message.data = json.dumps(
            self.result,
            ensure_ascii=False,
            sort_keys=True,
        )
        self.publisher.publish(message)

    def finish(self) -> None:
        if self.result is None:
            return
        self.save()
        self.publish()
        if self.result.get("pass") is True:
            mass = self.result["actual"]["dynamics"]["mass"]
            self.get_logger().warning(
                "PROFILE VERIFIED NATIVE: "
                f"{self.profile}; mass={mass:.4f} kg"
            )
        else:
            self.get_logger().error(
                "PROFILE GUARD NATIVE FAILED: "
                f'{self.result.get("failed_criteria")}'
            )

    def tick(self) -> None:
        if self.result is not None:
            self.publish()
            return

        elapsed = self.now_s() - self.started_s
        if elapsed < self.initial_delay_s:
            return

        if not self.futures:
            if self.services_ready():
                self.issue_requests()
            elif elapsed >= self.maximum_wait_s:
                self.result = self.failure(
                    "parameter_services_unavailable"
                )
                self.finish()
            return

        if self.collect():
            self.finish()
            return

        if elapsed >= self.maximum_wait_s:
            self.result = self.failure(
                "parameter_response_timeout"
            )
            self.finish()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V9ProfileGuard()
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
