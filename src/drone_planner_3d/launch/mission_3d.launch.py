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
    planner_share = get_package_share_directory(
        "drone_planner_3d"
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
        planner_share,
        "rviz",
        "planner_3d.rviz",
    )

    scenario = LaunchConfiguration("scenario")
    mission_mode = LaunchConfiguration("mission_mode")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    use_rviz = LaunchConfiguration("use_rviz")
    resolution = LaunchConfiguration("resolution_m")
    inflation = LaunchConfiguration(
        "planner_inflation_radius_m"
    )

    evaluation = Node(
        package="drone_planner_3d",
        executable="planner_3d_evaluation",
        name="planner_3d_evaluation",
        output="screen",
        parameters=[{
            "scenario": ParameterValue(
                scenario,
                value_type=str,
            ),
            "mission_mode": ParameterValue(
                mission_mode,
                value_type=str,
            ),
            "goal_x": ParameterValue(
                goal_x,
                value_type=float,
            ),
            "goal_y": ParameterValue(
                goal_y,
                value_type=float,
            ),
            "goal_z": ParameterValue(
                goal_z,
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="overflight_3d",
        ),
        DeclareLaunchArgument(
            "mission_mode",
            default_value="overflight",
        ),
        DeclareLaunchArgument(
            "goal_x",
            default_value="9.0",
        ),
        DeclareLaunchArgument(
            "goal_y",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "goal_z",
            default_value="1.5",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "resolution_m",
            default_value="0.30",
        ),
        DeclareLaunchArgument(
            "planner_inflation_radius_m",
            default_value="0.70",
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
                "scenario": ParameterValue(
                    scenario,
                    value_type=str,
                ),
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
                "scenario": ParameterValue(
                    scenario,
                    value_type=str,
                ),
                "resolution_m": ParameterValue(
                    resolution,
                    value_type=float,
                ),
                "inflation_radius_m": ParameterValue(
                    inflation,
                    value_type=float,
                ),
                "slice_z_m": 1.50,
                "marker_stride": 1,
            }],
        ),

        Node(
            package="drone_planner_3d",
            executable="astar_3d_planner",
            name="astar_3d_planner",
            output="screen",
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
            name="planner_3d_rviz",
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
                            reason="3D planner mission finished"
                        )
                    )
                ],
            )
        ),
    ])
