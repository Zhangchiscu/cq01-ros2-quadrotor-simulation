#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SingleGoalExperiment(Node):
    """Run, record, analyse and plot one quadrotor point-to-point experiment."""

    def __init__(self) -> None:
        super().__init__('single_goal_experiment')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 1.5)
        self.declare_parameter('duration_s', 20.0)
        self.declare_parameter('startup_delay_s', 2.0)
        self.declare_parameter('goal_publish_duration_s', 2.0)
        self.declare_parameter('arrival_tolerance_m', 0.30)
        self.declare_parameter('settling_window_s', 2.0)
        self.declare_parameter('steady_state_window_s', 2.0)
        self.declare_parameter('maximum_rpm', 10000.0)
        self.declare_parameter(
            'output_root',
            str(
                Path.home()
                / 'drone_task'
                / 'your_drone_sim_ws'
                / 'results'
            ),
        )

        self.goal = [
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_z').value),
        ]
        self.duration_s = float(
            self.get_parameter('duration_s').value
        )
        self.startup_delay_s = float(
            self.get_parameter('startup_delay_s').value
        )
        self.goal_publish_duration_s = float(
            self.get_parameter('goal_publish_duration_s').value
        )
        self.arrival_tolerance_m = float(
            self.get_parameter('arrival_tolerance_m').value
        )
        self.settling_window_s = float(
            self.get_parameter('settling_window_s').value
        )
        self.steady_state_window_s = float(
            self.get_parameter('steady_state_window_s').value
        )
        self.maximum_rpm = float(
            self.get_parameter('maximum_rpm').value
        )
        self.output_root = Path(
            os.path.expanduser(
                str(self.get_parameter('output_root').value)
            )
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/drone/goal',
            10,
        )
        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            '/drone/motor_rpm',
            self.rpm_callback,
            50,
        )

        self.latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self.samples: List[Dict[str, float]] = []

        self.first_odom_received = False
        self.started = False
        self.finished = False
        self.goal_start_ns: Optional[int] = None
        self.initial_position: Optional[List[float]] = None

        self.start_timer = self.create_timer(
            self.startup_delay_s,
            self.start_experiment,
        )
        self.goal_timer = None
        self.finish_timer = None

        self.get_logger().info(
            'Single-goal experiment ready: '
            f'goal=({self.goal[0]:.2f}, '
            f'{self.goal[1]:.2f}, '
            f'{self.goal[2]:.2f}), '
            f'duration={self.duration_s:.1f} s'
        )

    def start_experiment(self) -> None:
        if self.started:
            return

        if not self.first_odom_received:
            self.get_logger().warn(
                'Waiting for the first /drone/odom message...'
            )
            return

        self.started = True
        self.start_timer.cancel()
        self.goal_start_ns = self.get_clock().now().nanoseconds

        self.publish_goal()
        self.goal_timer = self.create_timer(
            0.10,
            self.publish_goal,
        )
        self.finish_timer = self.create_timer(
            self.duration_s,
            self.finish_experiment,
        )

        self.get_logger().info(
            'Experiment started and goal published'
        )

    def publish_goal(self) -> None:
        if not self.started or self.finished:
            return

        elapsed = self.elapsed_time()

        if elapsed > self.goal_publish_duration_s:
            if self.goal_timer is not None:
                self.goal_timer.cancel()
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.position.z = self.goal[2]
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        if len(message.data) >= 4:
            self.latest_rpm = [
                float(message.data[index])
                for index in range(4)
            ]

    @staticmethod
    def quaternion_to_euler(
        w: float,
        x: float,
        y: float,
        z: float,
    ) -> List[float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.first_odom_received = True

        if not self.started or self.finished:
            return

        position = [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ]

        if self.initial_position is None:
            self.initial_position = position.copy()

        error = [
            self.goal[index] - position[index]
            for index in range(3)
        ]

        error_norm = math.sqrt(
            sum(value * value for value in error)
        )

        orientation = message.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )

        velocity = message.twist.twist.linear
        angular_velocity = message.twist.twist.angular

        self.samples.append({
            'time_s': self.elapsed_time(),
            'goal_x': self.goal[0],
            'goal_y': self.goal[1],
            'goal_z': self.goal[2],
            'position_x': position[0],
            'position_y': position[1],
            'position_z': position[2],
            'error_x': error[0],
            'error_y': error[1],
            'error_z': error[2],
            'error_norm': error_norm,
            'velocity_body_x': float(velocity.x),
            'velocity_body_y': float(velocity.y),
            'velocity_body_z': float(velocity.z),
            'angular_velocity_x': float(angular_velocity.x),
            'angular_velocity_y': float(angular_velocity.y),
            'angular_velocity_z': float(angular_velocity.z),
            'roll_rad': roll,
            'pitch_rad': pitch,
            'yaw_rad': yaw,
            'rpm_front_left': self.latest_rpm[0],
            'rpm_front_right': self.latest_rpm[1],
            'rpm_rear_right': self.latest_rpm[2],
            'rpm_rear_left': self.latest_rpm[3],
        })

    def elapsed_time(self) -> float:
        if self.goal_start_ns is None:
            return 0.0

        return (
            self.get_clock().now().nanoseconds
            - self.goal_start_ns
        ) / 1.0e9

    @staticmethod
    def distance(
        point_a: List[float],
        point_b: List[float],
    ) -> float:
        return math.sqrt(
            sum(
                (point_a[index] - point_b[index]) ** 2
                for index in range(3)
            )
        )

    def compute_metrics(self) -> Dict[str, object]:
        if not self.samples:
            raise RuntimeError(
                'No odometry samples were recorded'
            )

        final_sample = self.samples[-1]
        final_position = [
            final_sample['position_x'],
            final_sample['position_y'],
            final_sample['position_z'],
        ]

        final_error = float(final_sample['error_norm'])

        steady_cutoff = max(
            0.0,
            float(final_sample['time_s'])
            - self.steady_state_window_s,
        )
        steady_errors = [
            float(sample['error_norm'])
            for sample in self.samples
            if float(sample['time_s']) >= steady_cutoff
        ]
        steady_state_error = (
            sum(steady_errors) / len(steady_errors)
            if steady_errors
            else final_error
        )

        path_length = 0.0
        previous_position = [
            self.samples[0]['position_x'],
            self.samples[0]['position_y'],
            self.samples[0]['position_z'],
        ]

        for sample in self.samples[1:]:
            current_position = [
                sample['position_x'],
                sample['position_y'],
                sample['position_z'],
            ]
            path_length += self.distance(
                previous_position,
                current_position,
            )
            previous_position = current_position

        initial_position = (
            self.initial_position
            if self.initial_position is not None
            else [0.0, 0.0, 0.0]
        )

        straight_distance = self.distance(
            initial_position,
            self.goal,
        )

        path_efficiency = (
            straight_distance / path_length
            if path_length > 1.0e-9
            else 0.0
        )

        arrival_time = None
        last_outside_index = -1

        for index, sample in enumerate(self.samples):
            if (
                float(sample['error_norm'])
                > self.arrival_tolerance_m
            ):
                last_outside_index = index

        candidate_index = last_outside_index + 1

        if candidate_index < len(self.samples):
            candidate_time = float(
                self.samples[candidate_index]['time_s']
            )
            remaining_time = (
                float(self.samples[-1]['time_s'])
                - candidate_time
            )

            if remaining_time >= self.settling_window_s:
                arrival_time = candidate_time

        overshoot_distance = 0.0

        if straight_distance > 1.0e-9:
            unit_direction = [
                (
                    self.goal[index]
                    - initial_position[index]
                ) / straight_distance
                for index in range(3)
            ]

            maximum_projection = -float('inf')

            for sample in self.samples:
                displacement = [
                    sample['position_x'] - initial_position[0],
                    sample['position_y'] - initial_position[1],
                    sample['position_z'] - initial_position[2],
                ]
                projection = sum(
                    displacement[index]
                    * unit_direction[index]
                    for index in range(3)
                )
                maximum_projection = max(
                    maximum_projection,
                    projection,
                )

            overshoot_distance = max(
                0.0,
                maximum_projection - straight_distance,
            )

        overshoot_percent = (
            100.0 * overshoot_distance / straight_distance
            if straight_distance > 1.0e-9
            else 0.0
        )

        maximum_tilt_deg = max(
            math.degrees(
                math.sqrt(
                    float(sample['roll_rad']) ** 2
                    + float(sample['pitch_rad']) ** 2
                )
            )
            for sample in self.samples
        )

        maximum_speed = max(
            math.sqrt(
                float(sample['velocity_body_x']) ** 2
                + float(sample['velocity_body_y']) ** 2
                + float(sample['velocity_body_z']) ** 2
            )
            for sample in self.samples
        )

        rpm_values: List[float] = []
        saturated_count = 0

        for sample in self.samples:
            sample_rpms = [
                float(sample['rpm_front_left']),
                float(sample['rpm_front_right']),
                float(sample['rpm_rear_right']),
                float(sample['rpm_rear_left']),
            ]
            rpm_values.extend(sample_rpms)
            saturated_count += sum(
                rpm >= 0.99 * self.maximum_rpm
                for rpm in sample_rpms
            )

        maximum_rpm_observed = (
            max(rpm_values)
            if rpm_values
            else 0.0
        )

        rpm_saturation_ratio = (
            saturated_count / len(rpm_values)
            if rpm_values
            else 0.0
        )

        return {
            'goal_m': {
                'x': self.goal[0],
                'y': self.goal[1],
                'z': self.goal[2],
            },
            'initial_position_m': {
                'x': initial_position[0],
                'y': initial_position[1],
                'z': initial_position[2],
            },
            'final_position_m': {
                'x': final_position[0],
                'y': final_position[1],
                'z': final_position[2],
            },
            'experiment_duration_s': float(
                final_sample['time_s']
            ),
            'sample_count': len(self.samples),
            'final_position_error_m': final_error,
            'steady_state_error_m': steady_state_error,
            'arrival_tolerance_m': self.arrival_tolerance_m,
            'arrival_time_s': arrival_time,
            'overshoot_distance_m': overshoot_distance,
            'overshoot_percent': overshoot_percent,
            'path_length_m': path_length,
            'straight_distance_m': straight_distance,
            'path_efficiency_ratio': path_efficiency,
            'maximum_speed_mps': maximum_speed,
            'maximum_tilt_deg': maximum_tilt_deg,
            'maximum_rpm_observed': maximum_rpm_observed,
            'rpm_saturation_ratio': rpm_saturation_ratio,
        }

    def write_csv(self, output_dir: Path) -> Path:
        csv_path = output_dir / 'flight_data.csv'
        fieldnames = list(self.samples[0].keys())

        with csv_path.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=fieldnames,
            )
            writer.writeheader()
            writer.writerows(self.samples)

        return csv_path

    def write_summary(
        self,
        output_dir: Path,
        metrics: Dict[str, object],
    ) -> None:
        json_path = output_dir / 'summary.json'
        json_path.write_text(
            json.dumps(
                metrics,
                indent=2,
                ensure_ascii=False,
            ),
            encoding='utf-8',
        )

        arrival_time = metrics['arrival_time_s']
        arrival_text = (
            f'{arrival_time:.3f} s'
            if isinstance(arrival_time, (int, float))
            else 'Not settled within experiment'
        )

        lines = [
            'ROS2 Quadrotor Single-Goal Experiment',
            '======================================',
            (
                'Goal: '
                f"({metrics['goal_m']['x']:.3f}, "
                f"{metrics['goal_m']['y']:.3f}, "
                f"{metrics['goal_m']['z']:.3f}) m"
            ),
            (
                'Final position: '
                f"({metrics['final_position_m']['x']:.3f}, "
                f"{metrics['final_position_m']['y']:.3f}, "
                f"{metrics['final_position_m']['z']:.3f}) m"
            ),
            (
                'Final position error: '
                f"{metrics['final_position_error_m']:.4f} m"
            ),
            (
                'Steady-state error: '
                f"{metrics['steady_state_error_m']:.4f} m"
            ),
            f'Arrival time: {arrival_text}',
            (
                'Overshoot: '
                f"{metrics['overshoot_distance_m']:.4f} m "
                f"({metrics['overshoot_percent']:.2f}%)"
            ),
            (
                'Path length: '
                f"{metrics['path_length_m']:.4f} m"
            ),
            (
                'Path efficiency: '
                f"{metrics['path_efficiency_ratio']:.4f}"
            ),
            (
                'Maximum speed: '
                f"{metrics['maximum_speed_mps']:.4f} m/s"
            ),
            (
                'Maximum tilt: '
                f"{metrics['maximum_tilt_deg']:.3f} deg"
            ),
            (
                'Maximum RPM: '
                f"{metrics['maximum_rpm_observed']:.2f}"
            ),
            (
                'RPM saturation ratio: '
                f"{100.0 * metrics['rpm_saturation_ratio']:.3f}%"
            ),
            f"Samples: {metrics['sample_count']}",
        ]

        (output_dir / 'summary.txt').write_text(
            '\n'.join(lines) + '\n',
            encoding='utf-8',
        )

    def create_plots(self, output_dir: Path) -> None:
        times = [
            float(sample['time_s'])
            for sample in self.samples
        ]

        figure = plt.figure(figsize=(9, 5))
        plt.plot(
            times,
            [sample['position_x'] for sample in self.samples],
            label='x',
        )
        plt.plot(
            times,
            [sample['position_y'] for sample in self.samples],
            label='y',
        )
        plt.plot(
            times,
            [sample['position_z'] for sample in self.samples],
            label='z',
        )
        plt.axhline(
            self.goal[0],
            linestyle='--',
            linewidth=1,
            label='goal x',
        )
        plt.axhline(
            self.goal[1],
            linestyle='--',
            linewidth=1,
            label='goal y',
        )
        plt.axhline(
            self.goal[2],
            linestyle='--',
            linewidth=1,
            label='goal z',
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Position Response')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir / 'position_response.png',
            dpi=180,
        )
        plt.close(figure)

        figure = plt.figure(figsize=(9, 5))
        plt.plot(
            times,
            [sample['error_norm'] for sample in self.samples],
            label='position error norm',
        )
        plt.axhline(
            self.arrival_tolerance_m,
            linestyle='--',
            linewidth=1,
            label='arrival tolerance',
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Position error (m)')
        plt.title('Position Error')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir / 'position_error.png',
            dpi=180,
        )
        plt.close(figure)

        figure = plt.figure(figsize=(9, 5))
        for key, label in (
            ('rpm_front_left', 'front left'),
            ('rpm_front_right', 'front right'),
            ('rpm_rear_right', 'rear right'),
            ('rpm_rear_left', 'rear left'),
        ):
            plt.plot(
                times,
                [sample[key] for sample in self.samples],
                label=label,
            )
        plt.xlabel('Time (s)')
        plt.ylabel('Motor speed (RPM)')
        plt.title('Motor RPM')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir / 'motor_rpm.png',
            dpi=180,
        )
        plt.close(figure)

    def finish_experiment(self) -> None:
        if self.finished:
            return

        self.finished = True

        if self.goal_timer is not None:
            self.goal_timer.cancel()
        if self.finish_timer is not None:
            self.finish_timer.cancel()

        try:
            if not self.samples:
                raise RuntimeError(
                    'Experiment ended without odometry samples'
                )

            timestamp = datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )
            output_dir = (
                self.output_root
                / f'single_goal_{timestamp}'
            )
            output_dir.mkdir(
                parents=True,
                exist_ok=False,
            )

            metrics = self.compute_metrics()
            csv_path = self.write_csv(output_dir)
            self.write_summary(output_dir, metrics)
            self.create_plots(output_dir)

            self.get_logger().info(
                'Experiment complete'
            )
            self.get_logger().info(
                f'Results directory: {output_dir}'
            )
            self.get_logger().info(
                'Final error: '
                f"{metrics['final_position_error_m']:.4f} m"
            )
            self.get_logger().info(
                'Steady-state error: '
                f"{metrics['steady_state_error_m']:.4f} m"
            )
            self.get_logger().info(
                f'CSV: {csv_path}'
            )
        except Exception as exception:
            self.get_logger().error(
                f'Failed to save experiment: {exception}'
            )
        finally:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleGoalExperiment()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finished and node.samples:
            node.finish_experiment()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
