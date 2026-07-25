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


class MultiGoalMission(Node):
    """Run a sequential waypoint mission and generate quantitative results."""

    def __init__(self) -> None:
        super().__init__('multi_goal_experiment')

        self.declare_parameter(
            'waypoint_x',
            [0.0, 2.0, -1.0, 0.0],
        )
        self.declare_parameter(
            'waypoint_y',
            [0.0, 1.0, 2.0, 0.0],
        )
        self.declare_parameter(
            'waypoint_z',
            [1.5, 1.5, 2.0, 1.5],
        )

        self.declare_parameter('startup_delay_s', 2.0)
        self.declare_parameter('goal_publish_rate_hz', 10.0)
        self.declare_parameter('arrival_tolerance_m', 0.30)
        self.declare_parameter('hold_time_s', 1.0)
        self.declare_parameter('waypoint_timeout_s', 15.0)
        self.declare_parameter('mission_timeout_s', 70.0)
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

        waypoint_x = [
            float(value)
            for value in self.get_parameter(
                'waypoint_x'
            ).value
        ]
        waypoint_y = [
            float(value)
            for value in self.get_parameter(
                'waypoint_y'
            ).value
        ]
        waypoint_z = [
            float(value)
            for value in self.get_parameter(
                'waypoint_z'
            ).value
        ]

        if not (
            len(waypoint_x)
            == len(waypoint_y)
            == len(waypoint_z)
        ):
            raise RuntimeError(
                'waypoint_x, waypoint_y and waypoint_z '
                'must have equal lengths'
            )

        if len(waypoint_x) < 2:
            raise RuntimeError(
                'At least two waypoints are required'
            )

        self.waypoints = [
            [
                waypoint_x[index],
                waypoint_y[index],
                waypoint_z[index],
            ]
            for index in range(len(waypoint_x))
        ]

        self.startup_delay_s = float(
            self.get_parameter('startup_delay_s').value
        )
        self.goal_publish_rate_hz = float(
            self.get_parameter(
                'goal_publish_rate_hz'
            ).value
        )
        self.arrival_tolerance_m = float(
            self.get_parameter(
                'arrival_tolerance_m'
            ).value
        )
        self.hold_time_s = float(
            self.get_parameter('hold_time_s').value
        )
        self.waypoint_timeout_s = float(
            self.get_parameter(
                'waypoint_timeout_s'
            ).value
        )
        self.mission_timeout_s = float(
            self.get_parameter(
                'mission_timeout_s'
            ).value
        )
        self.maximum_rpm = float(
            self.get_parameter('maximum_rpm').value
        )
        self.output_root = Path(
            os.path.expanduser(
                str(
                    self.get_parameter(
                        'output_root'
                    ).value
                )
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
        self.waypoint_results: List[Dict[str, object]] = []

        self.first_odom_received = False
        self.started = False
        self.finished = False
        self.current_waypoint_index = 0

        self.mission_start_ns: Optional[int] = None
        self.waypoint_command_ns: Optional[int] = None
        self.within_tolerance_ns: Optional[int] = None
        self.initial_position: Optional[List[float]] = None
        self.latest_position: Optional[List[float]] = None

        self.start_timer = self.create_timer(
            self.startup_delay_s,
            self.try_start_mission,
        )

        self.goal_timer = None
        self.watchdog_timer = self.create_timer(
            0.10,
            self.watchdog_step,
        )

        self.get_logger().info(
            f'Multi-goal mission ready with '
            f'{len(self.waypoints)} waypoints'
        )

        for index, waypoint in enumerate(
            self.waypoints,
            start=1,
        ):
            self.get_logger().info(
                f'Waypoint {index}: '
                f'({waypoint[0]:.2f}, '
                f'{waypoint[1]:.2f}, '
                f'{waypoint[2]:.2f})'
            )

    @staticmethod
    def distance(
        point_a: List[float],
        point_b: List[float],
    ) -> float:
        return math.sqrt(
            sum(
                (
                    point_a[index]
                    - point_b[index]
                ) ** 2
                for index in range(3)
            )
        )

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
            pitch = math.copysign(
                math.pi / 2.0,
                sinp,
            )
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )
        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        return [roll, pitch, yaw]

    def rpm_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        if len(message.data) >= 4:
            self.latest_rpm = [
                float(message.data[index])
                for index in range(4)
            ]

    def try_start_mission(self) -> None:
        if self.started or self.finished:
            return

        if not self.first_odom_received:
            self.get_logger().warn(
                'Waiting for /drone/odom...'
            )
            return

        odom_publishers = (
            self.get_publishers_info_by_topic(
                '/drone/odom'
            )
        )

        if len(odom_publishers) != 1:
            self.get_logger().error(
                'Expected exactly one /drone/odom '
                f'publisher, found {len(odom_publishers)}. '
                'Stop all old drone launch processes '
                'before running this experiment.'
            )
            self.finished = True
            rclpy.shutdown()
            return

        self.started = True
        self.start_timer.cancel()

        now_ns = self.get_clock().now().nanoseconds
        self.mission_start_ns = now_ns
        self.waypoint_command_ns = now_ns
        self.within_tolerance_ns = None

        self.goal_timer = self.create_timer(
            1.0 / self.goal_publish_rate_hz,
            self.publish_current_goal,
        )

        self.publish_current_goal()

        self.get_logger().info(
            'Mission started'
        )
        self.log_current_waypoint()

    def log_current_waypoint(self) -> None:
        waypoint = self.waypoints[
            self.current_waypoint_index
        ]
        self.get_logger().info(
            f'Commanding waypoint '
            f'{self.current_waypoint_index + 1}/'
            f'{len(self.waypoints)}: '
            f'({waypoint[0]:.2f}, '
            f'{waypoint[1]:.2f}, '
            f'{waypoint[2]:.2f})'
        )

    def publish_current_goal(self) -> None:
        if not self.started or self.finished:
            return

        waypoint = self.waypoints[
            self.current_waypoint_index
        ]

        message = PoseStamped()
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = 'map'
        message.pose.position.x = waypoint[0]
        message.pose.position.y = waypoint[1]
        message.pose.position.z = waypoint[2]
        message.pose.orientation.w = 1.0

        self.goal_pub.publish(message)

    def mission_elapsed_s(self) -> float:
        if self.mission_start_ns is None:
            return 0.0

        return (
            self.get_clock().now().nanoseconds
            - self.mission_start_ns
        ) / 1.0e9

    def waypoint_elapsed_s(self) -> float:
        if self.waypoint_command_ns is None:
            return 0.0

        return (
            self.get_clock().now().nanoseconds
            - self.waypoint_command_ns
        ) / 1.0e9

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        self.first_odom_received = True

        position = [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ]

        self.latest_position = position

        if not self.started or self.finished:
            return

        if self.initial_position is None:
            self.initial_position = position.copy()

        waypoint = self.waypoints[
            self.current_waypoint_index
        ]

        error = [
            waypoint[index] - position[index]
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
        angular_velocity = (
            message.twist.twist.angular
        )

        self.samples.append({
            'time_s': self.mission_elapsed_s(),
            'waypoint_index': float(
                self.current_waypoint_index
            ),
            'goal_x': waypoint[0],
            'goal_y': waypoint[1],
            'goal_z': waypoint[2],
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
            'angular_velocity_x': float(
                angular_velocity.x
            ),
            'angular_velocity_y': float(
                angular_velocity.y
            ),
            'angular_velocity_z': float(
                angular_velocity.z
            ),
            'roll_rad': roll,
            'pitch_rad': pitch,
            'yaw_rad': yaw,
            'rpm_front_left': self.latest_rpm[0],
            'rpm_front_right': self.latest_rpm[1],
            'rpm_rear_right': self.latest_rpm[2],
            'rpm_rear_left': self.latest_rpm[3],
        })

        now_ns = self.get_clock().now().nanoseconds

        if error_norm <= self.arrival_tolerance_m:
            if self.within_tolerance_ns is None:
                self.within_tolerance_ns = now_ns

            held_s = (
                now_ns
                - self.within_tolerance_ns
            ) / 1.0e9

            if held_s >= self.hold_time_s:
                self.complete_current_waypoint(
                    status='reached',
                    final_error=error_norm,
                )
        else:
            self.within_tolerance_ns = None

    def watchdog_step(self) -> None:
        if not self.started or self.finished:
            return

        if (
            self.mission_elapsed_s()
            > self.mission_timeout_s
        ):
            self.get_logger().error(
                'Mission timeout reached'
            )
            self.finish_mission(
                completion_status='mission_timeout'
            )
            return

        if (
            self.waypoint_elapsed_s()
            > self.waypoint_timeout_s
        ):
            final_error = None

            if self.latest_position is not None:
                final_error = self.distance(
                    self.latest_position,
                    self.waypoints[
                        self.current_waypoint_index
                    ],
                )

            self.complete_current_waypoint(
                status='timeout',
                final_error=final_error,
            )

    def complete_current_waypoint(
        self,
        status: str,
        final_error: Optional[float],
    ) -> None:
        if self.finished:
            return

        waypoint = self.waypoints[
            self.current_waypoint_index
        ]

        result: Dict[str, object] = {
            'waypoint_index': (
                self.current_waypoint_index + 1
            ),
            'goal_m': {
                'x': waypoint[0],
                'y': waypoint[1],
                'z': waypoint[2],
            },
            'status': status,
            'time_from_command_s': (
                self.waypoint_elapsed_s()
            ),
            'mission_time_s': (
                self.mission_elapsed_s()
            ),
            'final_error_m': final_error,
        }

        if self.latest_position is not None:
            result['final_position_m'] = {
                'x': self.latest_position[0],
                'y': self.latest_position[1],
                'z': self.latest_position[2],
            }

        self.waypoint_results.append(result)

        self.get_logger().info(
            f'Waypoint '
            f'{self.current_waypoint_index + 1} '
            f'{status}; '
            f'time={result["time_from_command_s"]:.3f} s, '
            f'error={final_error if final_error is not None else float("nan"):.4f} m'
        )

        if (
            self.current_waypoint_index
            >= len(self.waypoints) - 1
        ):
            self.finish_mission(
                completion_status='completed'
            )
            return

        self.current_waypoint_index += 1
        now_ns = self.get_clock().now().nanoseconds
        self.waypoint_command_ns = now_ns
        self.within_tolerance_ns = None

        self.publish_current_goal()
        self.log_current_waypoint()

    def compute_metrics(self) -> Dict[str, object]:
        if not self.samples:
            raise RuntimeError(
                'No odometry samples were recorded'
            )

        path_length = 0.0
        previous = [
            self.samples[0]['position_x'],
            self.samples[0]['position_y'],
            self.samples[0]['position_z'],
        ]

        for sample in self.samples[1:]:
            current = [
                sample['position_x'],
                sample['position_y'],
                sample['position_z'],
            ]
            path_length += self.distance(
                previous,
                current,
            )
            previous = current

        planned_distance = 0.0
        start = (
            self.initial_position
            if self.initial_position is not None
            else [0.0, 0.0, 0.0]
        )

        previous_goal = start

        for waypoint in self.waypoints:
            planned_distance += self.distance(
                previous_goal,
                waypoint,
            )
            previous_goal = waypoint

        path_efficiency = (
            planned_distance / path_length
            if path_length > 1.0e-9
            else 0.0
        )

        maximum_speed = max(
            math.sqrt(
                float(
                    sample['velocity_body_x']
                ) ** 2
                + float(
                    sample['velocity_body_y']
                ) ** 2
                + float(
                    sample['velocity_body_z']
                ) ** 2
            )
            for sample in self.samples
        )

        maximum_tilt_deg = max(
            math.degrees(
                math.sqrt(
                    float(sample['roll_rad']) ** 2
                    + float(
                        sample['pitch_rad']
                    ) ** 2
                )
            )
            for sample in self.samples
        )

        rpm_values: List[float] = []

        for sample in self.samples:
            rpm_values.extend([
                float(sample['rpm_front_left']),
                float(sample['rpm_front_right']),
                float(sample['rpm_rear_right']),
                float(sample['rpm_rear_left']),
            ])

        maximum_rpm_observed = (
            max(rpm_values)
            if rpm_values
            else 0.0
        )

        saturated_count = sum(
            rpm >= 0.99 * self.maximum_rpm
            for rpm in rpm_values
        )

        rpm_saturation_ratio = (
            saturated_count / len(rpm_values)
            if rpm_values
            else 0.0
        )

        reached_count = sum(
            result['status'] == 'reached'
            for result in self.waypoint_results
        )

        final_sample = self.samples[-1]

        return {
            'waypoint_count': len(self.waypoints),
            'waypoints_reached': reached_count,
            'mission_completion_ratio': (
                reached_count / len(self.waypoints)
            ),
            'mission_duration_s': float(
                final_sample['time_s']
            ),
            'sample_count': len(self.samples),
            'planned_path_length_m': planned_distance,
            'actual_path_length_m': path_length,
            'path_length_ratio_actual_over_planned': (
                path_length / planned_distance
                if planned_distance > 1.0e-9
                else 0.0
            ),
            'maximum_speed_mps': maximum_speed,
            'maximum_tilt_deg': maximum_tilt_deg,
            'maximum_rpm_observed': (
                maximum_rpm_observed
            ),
            'rpm_saturation_ratio': (
                rpm_saturation_ratio
            ),
            'final_position_m': {
                'x': float(
                    final_sample['position_x']
                ),
                'y': float(
                    final_sample['position_y']
                ),
                'z': float(
                    final_sample['position_z']
                ),
            },
            'waypoint_results': (
                self.waypoint_results
            ),
        }

    def write_csv(self, output_dir: Path) -> None:
        csv_path = output_dir / 'mission_data.csv'

        with csv_path.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=list(
                    self.samples[0].keys()
                ),
            )
            writer.writeheader()
            writer.writerows(self.samples)

    def write_summary(
        self,
        output_dir: Path,
        metrics: Dict[str, object],
        completion_status: str,
    ) -> None:
        metrics['completion_status'] = (
            completion_status
        )

        (output_dir / 'summary.json').write_text(
            json.dumps(
                metrics,
                indent=2,
                ensure_ascii=False,
            ),
            encoding='utf-8',
        )

        lines = [
            'ROS2 Quadrotor Multi-Goal Mission',
            '==================================',
            (
                f'Completion status: '
                f'{completion_status}'
            ),
            (
                f'Waypoints reached: '
                f"{metrics['waypoints_reached']}/"
                f"{metrics['waypoint_count']}"
            ),
            (
                f'Mission duration: '
                f"{metrics['mission_duration_s']:.3f} s"
            ),
            (
                f'Planned path length: '
                f"{metrics['planned_path_length_m']:.4f} m"
            ),
            (
                f'Actual path length: '
                f"{metrics['actual_path_length_m']:.4f} m"
            ),
            (
                f'Actual/planned path ratio: '
                f"{metrics['path_length_ratio_actual_over_planned']:.4f}"
            ),
            (
                f'Maximum speed: '
                f"{metrics['maximum_speed_mps']:.4f} m/s"
            ),
            (
                f'Maximum tilt: '
                f"{metrics['maximum_tilt_deg']:.3f} deg"
            ),
            (
                f'Maximum RPM: '
                f"{metrics['maximum_rpm_observed']:.2f}"
            ),
            (
                f'RPM saturation ratio: '
                f"{100.0 * metrics['rpm_saturation_ratio']:.3f}%"
            ),
            '',
            'Per-waypoint results:',
        ]

        for result in self.waypoint_results:
            final_error = result['final_error_m']
            error_text = (
                f'{final_error:.4f} m'
                if isinstance(
                    final_error,
                    (int, float),
                )
                else 'N/A'
            )

            lines.append(
                f"  WP{result['waypoint_index']}: "
                f"{result['status']}, "
                f"time={result['time_from_command_s']:.3f} s, "
                f"error={error_text}"
            )

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
            [
                sample['position_x']
                for sample in self.samples
            ],
            label='x',
        )
        plt.plot(
            times,
            [
                sample['position_y']
                for sample in self.samples
            ],
            label='y',
        )
        plt.plot(
            times,
            [
                sample['position_z']
                for sample in self.samples
            ],
            label='z',
        )
        plt.plot(
            times,
            [
                sample['goal_x']
                for sample in self.samples
            ],
            '--',
            linewidth=1,
            label='goal x',
        )
        plt.plot(
            times,
            [
                sample['goal_y']
                for sample in self.samples
            ],
            '--',
            linewidth=1,
            label='goal y',
        )
        plt.plot(
            times,
            [
                sample['goal_z']
                for sample in self.samples
            ],
            '--',
            linewidth=1,
            label='goal z',
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Multi-Goal Position Response')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir
            / 'multi_goal_position_response.png',
            dpi=180,
        )
        plt.close(figure)

        figure = plt.figure(figsize=(9, 5))
        plt.plot(
            times,
            [
                sample['error_norm']
                for sample in self.samples
            ],
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
        plt.title('Multi-Goal Position Error')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir
            / 'multi_goal_position_error.png',
            dpi=180,
        )
        plt.close(figure)

        figure = plt.figure(figsize=(8, 7))
        axis = figure.add_subplot(
            111,
            projection='3d',
        )
        axis.plot(
            [
                sample['position_x']
                for sample in self.samples
            ],
            [
                sample['position_y']
                for sample in self.samples
            ],
            [
                sample['position_z']
                for sample in self.samples
            ],
            label='actual trajectory',
        )
        axis.scatter(
            [
                waypoint[0]
                for waypoint in self.waypoints
            ],
            [
                waypoint[1]
                for waypoint in self.waypoints
            ],
            [
                waypoint[2]
                for waypoint in self.waypoints
            ],
            marker='o',
            s=55,
            label='waypoints',
        )

        for index, waypoint in enumerate(
            self.waypoints,
            start=1,
        ):
            axis.text(
                waypoint[0],
                waypoint[1],
                waypoint[2],
                f' WP{index}',
            )

        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_zlabel('Z (m)')
        axis.set_title('3D Multi-Goal Trajectory')
        axis.legend()
        figure.tight_layout()
        figure.savefig(
            output_dir
            / 'multi_goal_trajectory_3d.png',
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
                [
                    sample[key]
                    for sample in self.samples
                ],
                label=label,
            )

        plt.xlabel('Time (s)')
        plt.ylabel('Motor speed (RPM)')
        plt.title('Multi-Goal Motor RPM')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        figure.savefig(
            output_dir
            / 'multi_goal_motor_rpm.png',
            dpi=180,
        )
        plt.close(figure)

    def finish_mission(
        self,
        completion_status: str,
    ) -> None:
        if self.finished:
            return

        self.finished = True

        if self.goal_timer is not None:
            self.goal_timer.cancel()

        try:
            if not self.samples:
                raise RuntimeError(
                    'Mission ended without samples'
                )

            timestamp = datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )

            output_dir = (
                self.output_root
                / f'multi_goal_{timestamp}'
            )

            output_dir.mkdir(
                parents=True,
                exist_ok=False,
            )

            metrics = self.compute_metrics()
            self.write_csv(output_dir)
            self.write_summary(
                output_dir,
                metrics,
                completion_status,
            )
            self.create_plots(output_dir)

            self.get_logger().info(
                'Mission experiment complete'
            )
            self.get_logger().info(
                f'Results directory: {output_dir}'
            )
            self.get_logger().info(
                'Waypoints reached: '
                f"{metrics['waypoints_reached']}/"
                f"{metrics['waypoint_count']}"
            )
            self.get_logger().info(
                'Actual path length: '
                f"{metrics['actual_path_length_m']:.4f} m"
            )
        except Exception as exception:
            self.get_logger().error(
                f'Failed to save mission: {exception}'
            )
        finally:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiGoalMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finished and node.samples:
            node.finish_mission(
                completion_status='interrupted'
            )
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
