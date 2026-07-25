import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dynamics_share = get_package_share_directory("drone_dynamics")
    controller_share = get_package_share_directory("drone_controller")
    disturbance_share = get_package_share_directory("drone_disturbance")

    dynamics_params = os.path.join(
        dynamics_share,
        "config",
        "dynamics.yaml",
    )
    controller_params = os.path.join(
        controller_share,
        "config",
        "controller.yaml",
    )
    disturbance_params = os.path.join(
        disturbance_share,
        "config",
        "pulse_world.yaml",
    )
    rviz_config = os.path.join(
        disturbance_share,
        "rviz",
        "disturbance_recovery.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    mode = LaunchConfiguration("mode")
    frame_id = LaunchConfiguration("frame_id")
    start_delay_s = LaunchConfiguration("start_delay_s")
    duration_s = LaunchConfiguration("duration_s")
    force_x_n = LaunchConfiguration("force_x_n")
    force_y_n = LaunchConfiguration("force_y_n")
    force_z_n = LaunchConfiguration("force_z_n")
    torque_x_nm = LaunchConfiguration("torque_x_nm")
    torque_y_nm = LaunchConfiguration("torque_y_nm")
    torque_z_nm = LaunchConfiguration("torque_z_nm")
    experiment_duration_s = LaunchConfiguration("experiment_duration_s")

    experiment_node = Node(
        package="drone_disturbance",
        executable="disturbance_experiment",
        name="disturbance_experiment",
        output="screen",
        parameters=[
            disturbance_params,
            {
                "duration_s": ParameterValue(
                    experiment_duration_s,
                    value_type=float,
                ),
                "expected_disturbance_start_s": ParameterValue(
                    start_delay_s,
                    value_type=float,
                ),
                "expected_disturbance_duration_s": ParameterValue(
                    duration_s,
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("mode", default_value="pulse"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("start_delay_s", default_value="8.0"),
        DeclareLaunchArgument("duration_s", default_value="1.0"),
        DeclareLaunchArgument("force_x_n", default_value="4.0"),
        DeclareLaunchArgument("force_y_n", default_value="0.0"),
        DeclareLaunchArgument("force_z_n", default_value="0.0"),
        DeclareLaunchArgument("torque_x_nm", default_value="0.0"),
        DeclareLaunchArgument("torque_y_nm", default_value="0.0"),
        DeclareLaunchArgument("torque_z_nm", default_value="0.0"),
        DeclareLaunchArgument(
            "experiment_duration_s",
            default_value="20.0",
        ),

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

        Node(
            package="drone_disturbance",
            executable="disturbance_injector",
            name="disturbance_injector",
            output="screen",
            parameters=[
                disturbance_params,
                {
                    "mode": mode,
                    "frame_id": frame_id,
                    "start_delay_s": ParameterValue(
                        start_delay_s,
                        value_type=float,
                    ),
                    "duration_s": ParameterValue(
                        duration_s,
                        value_type=float,
                    ),
                    "force_x_n": ParameterValue(
                        force_x_n,
                        value_type=float,
                    ),
                    "force_y_n": ParameterValue(
                        force_y_n,
                        value_type=float,
                    ),
                    "force_z_n": ParameterValue(
                        force_z_n,
                        value_type=float,
                    ),
                    "torque_x_nm": ParameterValue(
                        torque_x_nm,
                        value_type=float,
                    ),
                    "torque_y_nm": ParameterValue(
                        torque_y_nm,
                        value_type=float,
                    ),
                    "torque_z_nm": ParameterValue(
                        torque_z_nm,
                        value_type=float,
                    ),
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="disturbance_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),

        experiment_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="Disturbance recovery experiment finished"
                        )
                    )
                ],
            )
        ),
    ])
