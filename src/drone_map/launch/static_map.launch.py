import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory(
        "drone_map"
    )
    params = os.path.join(
        share,
        "config",
        "obstacles.yaml",
    )
    rviz = os.path.join(
        share,
        "rviz",
        "obstacle_map.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        Node(
            package="drone_map",
            executable="static_obstacle_map",
            name="static_obstacle_map",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="obstacle_map_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz],
        ),
    ])
