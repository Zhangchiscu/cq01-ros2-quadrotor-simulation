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


def generate_launch_description():
    planner_share = get_package_share_directory(
        "drone_planner"
    )
    multigoal_share = get_package_share_directory(
        "drone_multigoal"
    )

    planner_launch = os.path.join(
        planner_share,
        "launch",
        "avoidance_demo.launch.py",
    )
    mission_params = os.path.join(
        multigoal_share,
        "config",
        "multi_goal_mission.yaml",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    mission_node = Node(
        package="drone_multigoal",
        executable="multi_goal_evaluation",
        name="multi_goal_evaluation",
        output="screen",
        parameters=[mission_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                planner_launch
            ),
            launch_arguments={
                "use_rviz": use_rviz,
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
                                "multi-goal mission finished"
                            )
                        )
                    )
                ],
            )
        ),
    ])
