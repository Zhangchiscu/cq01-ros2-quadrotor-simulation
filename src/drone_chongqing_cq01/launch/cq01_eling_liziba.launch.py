import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    dynamics_share = get_package_share_directory("drone_dynamics")
    controller_share = get_package_share_directory("drone_controller")
    cq01_share = get_package_share_directory("drone_chongqing_cq01")

    dynamics_params = os.path.join(
        dynamics_share, "config", "dynamics.yaml"
    )
    controller_params = os.path.join(
        controller_share, "config", "controller.yaml"
    )
    rviz_config = os.path.join(
        cq01_share, "rviz", "cq01_eling_liziba.rviz"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    timeout_seconds = LaunchConfiguration("timeout_seconds")

    evaluation = Node(
        package="drone_chongqing_cq01",
        executable="cq01_evaluation",
        name="cq01_foothill_ridge_evaluation",
        output="screen",
        parameters=[{
            "timeout_seconds": ParameterValue(
                timeout_seconds,
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "timeout_seconds",
            default_value="180.0",
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
            parameters=[
                controller_params,
                {
                    "maximum_goal_z": 16.0,
                },
            ],
        ),

        Node(
            package="drone_chongqing_cq01",
            executable="cq01_map_server",
            name="cq01_foothill_ridge_map_server",
            output="screen",
        ),

        Node(
            package="drone_planner_3d",
            executable="astar_3d_planner",
            name="astar_3d_planner",
            output="screen",
            parameters=[{
                "minimum_flight_z_m": 0.35,
                "maximum_flight_z_m": 15.0,
                "vertical_cost_factor": 1.15,
                "waypoint_tolerance_m": 0.30,
                "waypoint_hold_seconds": 0.12,
                "maximum_waypoint_spacing_m": 0.70,
                "line_of_sight_step_m": 0.10,
                "fixed_height_test_z_m": 1.50,
            }],
        ),

        Node(
            package="drone_chongqing_cq01",
            executable="cq01_mission",
            name="cq01_foothill_ridge_mission",
            output="screen",
        ),



        Node(
            package="drone_chongqing_cq01",
            executable="cq01_presentation",
            name="cq01_presentation",
            output="screen",
            parameters=[{
                "display_z_scale": 2.5,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="cq01_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),

        evaluation,

        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluation,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="CQ-01 evaluation finished"
                        )
                    )
                ],
            )
        ),
    ])
