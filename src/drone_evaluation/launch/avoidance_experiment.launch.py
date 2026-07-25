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
    evaluation_share = get_package_share_directory(
        "drone_evaluation"
    )

    planner_launch = os.path.join(
        planner_share,
        "launch",
        "avoidance_demo.launch.py",
    )
    evaluation_params = os.path.join(
        evaluation_share,
        "config",
        "avoidance_experiment.yaml",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    experiment_node = Node(
        package="drone_evaluation",
        executable="avoidance_experiment",
        name="avoidance_experiment",
        output="screen",
        parameters=[evaluation_params],
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

        experiment_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                "avoidance experiment finished"
                            )
                        )
                    )
                ],
            )
        ),
    ])
