import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in {"clean", "nominal", "stress"}:
        raise RuntimeError("profile must be clean, nominal or stress")

    sensor_share = get_package_share_directory("drone_sensor_sim")
    dynamics_share = get_package_share_directory("drone_dynamics")
    controller_share = get_package_share_directory("drone_controller")
    bringup_share = get_package_share_directory("drone_bringup")

    sensor_config = os.path.join(sensor_share, "config", f"sensor_{profile}.yaml")
    experiment_config = os.path.join(sensor_share, "config", "experiment.yaml")
    dynamics_config = os.path.join(dynamics_share, "config", "dynamics.yaml")
    controller_config = os.path.join(controller_share, "config", "controller.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "quadrotor.rviz")

    results_root = str(
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
        / "results"
        / "sensor_noise"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")

    experiment = Node(
        package="drone_sensor_sim",
        executable="sensor_experiment",
        name="sensor_experiment",
        output="screen",
        parameters=[
            experiment_config,
            {
                "profile": profile,
                "results_root": results_root,
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_z": ParameterValue(goal_z, value_type=float),
            },
        ],
    )

    return [
        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            name="quadrotor_dynamics",
            output="screen",
            parameters=[dynamics_config],
        ),
        Node(
            package="drone_sensor_sim",
            executable="sensor_model",
            name="sensor_model",
            output="screen",
            parameters=[sensor_config],
        ),
        Node(
            package="drone_controller",
            executable="quadrotor_controller_node",
            name="quadrotor_controller",
            output="screen",
            parameters=[controller_config],
            remappings=[("/drone/odom", "/drone/odom_sensor")],
        ),
        Node(
            package="drone_bringup",
            executable="drone_visualization",
            name="drone_visualization",
            output="screen",
        ),
        Node(
            package="drone_bringup",
            executable="default_goal",
            name="default_goal_publisher",
            output="screen",
            parameters=[{
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_z": ParameterValue(goal_z, value_type=float),
                "startup_delay_s": 1.0,
                "publish_count": 10,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="sensor_experiment_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
        experiment,
        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment,
                on_exit=[EmitEvent(event=Shutdown(reason="sensor experiment finished"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="nominal"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("goal_x", default_value="2.0"),
        DeclareLaunchArgument("goal_y", default_value="1.0"),
        DeclareLaunchArgument("goal_z", default_value="1.5"),
        OpaqueFunction(function=launch_setup),
    ])
