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


def generate_launch_description():
    dynamics_share = get_package_share_directory(
        "drone_dynamics"
    )
    controller_share = get_package_share_directory(
        "drone_controller"
    )
    trajectory_share = get_package_share_directory(
        "drone_trajectory"
    )

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
    rviz_config = os.path.join(
        trajectory_share,
        "rviz",
        "trajectory.rviz",
    )

    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")

    evaluation = Node(
        package="drone_trajectory",
        executable="trajectory_evaluation",
        name="trajectory_evaluation",
        output="screen",
        parameters=[{
            "mode": ParameterValue(
                mode,
                value_type=str,
            ),
            "scenario": "overflight_3d",
            "goal_x": 9.0,
            "goal_y": 0.0,
            "goal_z": 1.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="smooth",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
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
            package="drone_world",
            executable="world_server",
            name="world_server",
            output="screen",
            parameters=[{
                "scenario": "overflight_3d",
                "show_labels": False,
                "point_spacing_m": 0.25,
            }],
        ),

        Node(
            package="drone_voxel_map",
            executable="voxel_map_server",
            name="voxel_map_server",
            output="screen",
            parameters=[{
                "scenario": "overflight_3d",
                "resolution_m": 0.30,
                "inflation_radius_m": 0.70,
                "slice_z_m": 1.50,
                "marker_stride": 1,
            }],
        ),

        Node(
            package="drone_planner_3d",
            executable="astar_3d_planner",
            name="astar_3d_planner",
            output="screen",
            remappings=[
                (
                    "/drone/goal",
                    "/drone/goal_unsmoothed",
                )
            ],
        ),

        Node(
            package="drone_trajectory",
            executable="trajectory_executor",
            name="trajectory_executor",
            output="screen",
            parameters=[{
                "mode": ParameterValue(
                    mode,
                    value_type=str,
                ),
            }],
        ),

        Node(
            package="drone_bringup",
            executable="drone_visualization",
            name="drone_visualization",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="trajectory_rviz",
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
                            reason="trajectory evaluation finished"
                        )
                    )
                ],
            )
        ),
    ])
