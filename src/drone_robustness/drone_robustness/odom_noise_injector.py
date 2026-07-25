#!/usr/bin/env python3

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomNoiseInjector(Node):
    def __init__(self) -> None:
        super().__init__("odom_noise_injector")

        self.declare_parameter("position_noise_std_m", 0.0)
        self.declare_parameter("velocity_noise_std_mps", 0.0)
        self.declare_parameter("seed", 2026)

        self.position_std = float(
            self.get_parameter("position_noise_std_m").value
        )
        self.velocity_std = float(
            self.get_parameter("velocity_noise_std_mps").value
        )
        self.seed = int(self.get_parameter("seed").value)

        if self.position_std < 0.0 or self.velocity_std < 0.0:
            raise RuntimeError("Noise standard deviation must be non-negative.")

        self.random = random.Random(self.seed)

        self.publisher = self.create_publisher(
            Odometry,
            "/drone/odom_noisy",
            20,
        )
        self.subscription = self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            20,
        )

        self.sample_count = 0
        self.position_noise_sum_sq = 0.0
        self.velocity_noise_sum_sq = 0.0

        self.get_logger().info(
            "Odometry noise injector started: "
            f"position_std={self.position_std:.3f} m, "
            f"velocity_std={self.velocity_std:.3f} m/s, "
            f"seed={self.seed}"
        )

    def gaussian(self, standard_deviation: float) -> float:
        if standard_deviation <= 0.0:
            return 0.0
        return self.random.gauss(0.0, standard_deviation)

    def odom_callback(self, message: Odometry) -> None:
        noisy = Odometry()
        noisy.header = message.header
        noisy.child_frame_id = message.child_frame_id
        noisy.pose = message.pose
        noisy.twist = message.twist

        px = self.gaussian(self.position_std)
        py = self.gaussian(self.position_std)
        pz = self.gaussian(self.position_std)

        vx = self.gaussian(self.velocity_std)
        vy = self.gaussian(self.velocity_std)
        vz = self.gaussian(self.velocity_std)

        noisy.pose.pose.position.x += px
        noisy.pose.pose.position.y += py
        noisy.pose.pose.position.z += pz

        noisy.twist.twist.linear.x += vx
        noisy.twist.twist.linear.y += vy
        noisy.twist.twist.linear.z += vz

        self.publisher.publish(noisy)

        self.sample_count += 1
        self.position_noise_sum_sq += px * px + py * py + pz * pz
        self.velocity_noise_sum_sq += vx * vx + vy * vy + vz * vz

        if self.sample_count % 1000 == 0:
            position_rms = math.sqrt(
                self.position_noise_sum_sq
                / (3.0 * self.sample_count)
            )
            velocity_rms = math.sqrt(
                self.velocity_noise_sum_sq
                / (3.0 * self.sample_count)
            )
            self.get_logger().info(
                "Injected noise RMS: "
                f"position={position_rms:.4f} m, "
                f"velocity={velocity_rms:.4f} m/s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNoiseInjector()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
