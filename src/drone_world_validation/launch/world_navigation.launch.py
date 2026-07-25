import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        "drone_planner"
    )
    validation_share = get_package_share_directory(
        "drone_world_validation"
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
    planner_params = os.path.join(
        planner_share,
        "config",
        "planner.yaml",
    )
    rviz_config = os.path.join(
        validation_share,
        "rviz",
        "world_navigation.rviz",
    )

    scenario = LaunchConfiguration("scenario")
    use_rviz = LaunchConfiguration("use_rviz")
    planner_inflation = LaunchConfiguration(
        "planner_inflation_radius_m"
    )
    safety_inflation = LaunchConfiguration(
        "safety_inflation_radius_m"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="warehouse",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "planner_inflation_radius_m",
            default_value="0.70",
        ),
        DeclareLaunchArgument(
            "safety_inflation_radius_m",
            default_value="0.45",
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
                "map_topic": "/drone/obstacle_grid_raw",
                "show_labels": False,
                "point_spacing_m": 0.30,
            }],
        ),

        # Planner map: deliberately conservative, absorbs
        # tracking error and waypoint-corner deviation.
        Node(
            package="drone_world_validation",
            executable="occupancy_inflator",
            name="planner_occupancy_inflator",
            output="screen",
            parameters=[{
                "input_topic": "/drone/obstacle_grid_raw",
                "output_topic": "/drone/obstacle_grid",
                "inflation_radius_m": ParameterValue(
                    planner_inflation,
                    value_type=float,
                ),
            }],
        ),

        # Safety-verification map: represents the hard
        # center-distance requirement corresponding to a
        # 0.15 m vehicle radius plus 0.30 m body clearance.
        Node(
            package="drone_world_validation",
            executable="occupancy_inflator",
            name="safety_occupancy_inflator",
            output="screen",
            parameters=[{
                "input_topic": "/drone/obstacle_grid_raw",
                "output_topic": "/drone/obstacle_grid_safety",
                "inflation_radius_m": ParameterValue(
                    safety_inflation,
                    value_type=float,
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
            package="drone_planner",
            executable="astar_planner",
            name="astar_planner",
            output="screen",
            parameters=[planner_params],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="world_navigation_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
    ])
