import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    validation_share = get_package_share_directory(
        "drone_world_validation"
    )

    navigation_launch = os.path.join(
        validation_share,
        "launch",
        "world_navigation.launch.py",
    )

    scenario = LaunchConfiguration("scenario")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    use_rviz = LaunchConfiguration("use_rviz")
    planner_inflation = LaunchConfiguration(
        "planner_inflation_radius_m"
    )
    safety_inflation = LaunchConfiguration(
        "safety_inflation_radius_m"
    )
    minimum_clearance = LaunchConfiguration(
        "minimum_clearance_m"
    )

    mission_node = Node(
        package="drone_world_validation",
        executable="scenario_mission",
        name="scenario_mission",
        output="screen",
        parameters=[{
            "scenario": ParameterValue(
                scenario,
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
            "minimum_clearance_m": ParameterValue(
                minimum_clearance,
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="warehouse",
        ),
        DeclareLaunchArgument(
            "goal_x",
            default_value="6.8",
        ),
        DeclareLaunchArgument(
            "goal_y",
            default_value="4.5",
        ),
        DeclareLaunchArgument(
            "goal_z",
            default_value="1.5",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "planner_inflation_radius_m",
            default_value="0.70",
        ),
        DeclareLaunchArgument(
            "safety_inflation_radius_m",
            default_value="0.45",
        ),
        DeclareLaunchArgument(
            "minimum_clearance_m",
            default_value="0.30",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                navigation_launch
            ),
            launch_arguments={
                "scenario": scenario,
                "use_rviz": use_rviz,
                "planner_inflation_radius_m": planner_inflation,
                "safety_inflation_radius_m": safety_inflation,
            }.items(),
        ),

        mission_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=mission_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                "world scenario mission finished"
                            )
                        )
                    )
                ],
            )
        ),
    ])
