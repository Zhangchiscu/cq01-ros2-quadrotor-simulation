#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pattern = LaunchConfiguration("pattern")
    use_rviz = LaunchConfiguration("use_rviz")
    period_s = LaunchConfiguration("period_s")
    timeout_seconds = LaunchConfiguration("timeout_seconds")
    dynamics_params = PathJoinSubstitution([FindPackageShare("drone_dynamics"), "config", "dynamics.yaml"])
    controller_params = PathJoinSubstitution([FindPackageShare("drone_controller"), "config", "controller.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("drone_pattern_trajectory"), "rviz", "pattern.rviz"])

    executor = Node(
        package="drone_pattern_trajectory",
        executable="pattern_executor",
        name="pattern_trajectory_executor",
        output="screen",
        parameters=[{
            "pattern": pattern,
            "control_rate_hz": 20.0,
            "period_s": ParameterValue(period_s, value_type=float),
            "laps": 1.0,
            "entry_duration_s": 6.0,
            "pre_pattern_hold_s": 1.5,
            "post_pattern_hold_s": 2.0,
            "radius_x_m": 2.0,
            "radius_y_m": 1.4,
            "altitude_m": 1.5,
            "tangent_yaw": False,
        }],
    )

    evaluation = Node(
        package="drone_pattern_trajectory",
        executable="pattern_evaluation",
        name="pattern_trajectory_evaluation",
        output="screen",
        parameters=[{
            "pattern": pattern,
            "timeout_seconds": ParameterValue(timeout_seconds, value_type=float),
            "sample_rate_hz": 20.0,
            "settle_seconds": 0.8,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("pattern", default_value="circle"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("period_s", default_value="36.0"),
        DeclareLaunchArgument("timeout_seconds", default_value="70.0"),
        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            name="quadrotor_dynamics",
            output="screen",
            parameters=[dynamics_params],
        ),
        Node(
            package="drone_controller",
            executable="quadrotor_controller_node",
            name="quadrotor_controller",
            output="screen",
            parameters=[controller_params],
        ),
        Node(
            package="drone_bringup",
            executable="drone_visualization",
            name="drone_visualization",
            output="screen",
        ),
        executor,
        Node(
            package="rviz2",
            executable="rviz2",
            name="pattern_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
        evaluation,
        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluation,
                on_exit=[EmitEvent(event=Shutdown(reason="pattern evaluation finished"))],
            )
        ),
    ])
