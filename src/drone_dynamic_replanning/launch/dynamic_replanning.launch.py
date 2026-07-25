import os

from ament_index_python.packages import (
    get_package_share_directory,
)
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
    dynamic_share = get_package_share_directory(
        "drone_dynamic_replanning"
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
        dynamic_share,
        "rviz",
        "dynamic_replanning.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    obstacle_speed = LaunchConfiguration("obstacle_speed_mps")
    obstacle_start_delay = LaunchConfiguration("obstacle_start_delay_s")
    obstacle_center_dwell = LaunchConfiguration("obstacle_center_dwell_s")
    obstacle_prediction_horizon = LaunchConfiguration(
        "obstacle_prediction_horizon_s"
    )
    planning_prediction_horizon = LaunchConfiguration(
        "planning_prediction_horizon_s"
    )

    evaluation = Node(
        package="drone_dynamic_replanning",
        executable="dynamic_replanning_evaluation",
        name="dynamic_replanning_evaluation",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "obstacle_speed_mps",
            default_value="0.55",
        ),
        DeclareLaunchArgument(
            "obstacle_start_delay_s",
            default_value="3.0",
        ),
        DeclareLaunchArgument(
            "obstacle_center_dwell_s",
            default_value="8.0",
        ),
        DeclareLaunchArgument(
            "obstacle_prediction_horizon_s",
            default_value="6.0",
        ),
        DeclareLaunchArgument(
            "planning_prediction_horizon_s",
            default_value="3.5",
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
                "scenario": "dynamic_crossing_3d",
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
                "scenario": "dynamic_crossing_3d",
                "resolution_m": 0.30,
                "inflation_radius_m": 0.70,
                "slice_z_m": 1.50,
                "marker_stride": 1,
            }],
        ),

        Node(
            package="drone_dynamic_replanning",
            executable="dynamic_obstacle",
            name="dynamic_obstacle",
            output="screen",
            parameters=[{
                "speed_mps": ParameterValue(
                    obstacle_speed,
                    value_type=float,
                ),
                "start_delay_s": ParameterValue(
                    obstacle_start_delay,
                    value_type=float,
                ),
                "center_dwell_s": ParameterValue(
                    obstacle_center_dwell,
                    value_type=float,
                ),
                "prediction_horizon_s": ParameterValue(
                    obstacle_prediction_horizon,
                    value_type=float,
                ),
            }],
        ),

        Node(
            package="drone_dynamic_replanning",
            executable="dynamic_voxel_overlay",
            name="dynamic_voxel_overlay",
            output="screen",
            parameters=[{
                "planning_inflation_radius_m": 0.70,
                "planning_prediction_horizon_s": ParameterValue(
                    planning_prediction_horizon,
                    value_type=float,
                ),
                "start_escape_radius_m": 0.90,
            }],
        ),

        Node(
            package="drone_planner_3d",
            executable="astar_3d_planner",
            name="astar_3d_planner",
            output="screen",
            remappings=[
                (
                    "/drone/voxel_cloud",
                    "/drone/voxel_cloud_dynamic",
                ),
                (
                    "/drone/goal",
                    "/drone/goal_unsmoothed",
                ),
            ],
        ),

        Node(
            package="drone_trajectory",
            executable="trajectory_executor",
            name="trajectory_executor",
            output="screen",
            parameters=[{
                "mode": "raw",
            }],
            remappings=[
                (
                    "/drone/voxel_cloud",
                    "/drone/voxel_cloud_dynamic",
                ),
                (
                    "/drone/goal",
                    "/drone/goal_nominal",
                ),
            ],
        ),

        Node(
            package="drone_dynamic_replanning",
            executable="replanning_supervisor",
            name="replanning_supervisor",
            output="screen",
            parameters=[{
                "maximum_replans": 10,
                "replan_retry_delay_s": 1.00,
                "retry_current_clearance_m": 0.85,
                "prediction_trigger_clearance_m": 1.00,
                "physical_emergency_clearance_m": 0.85,
                "resume_predicted_clearance_m": 1.00,
                "resume_current_clearance_m": 0.85,
                "resume_safe_hold_s": 0.60,
                "prediction_path_lookahead_m": 2.50,
                "resume_path_lookahead_m": 1.50,
                "hold_before_replan_s": 0.10,
                "minimum_replan_interval_s": 0.75,
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
            name="dynamic_replanning_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=[
                "-d",
                rviz_config,
            ],
        ),

        evaluation,

        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluation,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                "dynamic replanning evaluation finished"
                            )
                        )
                    )
                ],
            )
        ),
    ])
