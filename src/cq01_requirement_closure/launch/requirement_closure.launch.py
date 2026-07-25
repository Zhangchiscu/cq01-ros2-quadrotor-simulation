import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        "cq01_requirement_closure"
    )
    config = os.path.join(
        package_share,
        "config",
        "requirement_closure.yaml",
    )

    return LaunchDescription([
        Node(
            package="cq01_requirement_closure",
            executable="sensor_noise_bridge",
            name="sensor_noise_bridge",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="cq01_requirement_closure",
            executable="local_perception_filter",
            name="local_perception_filter",
            output="screen",
            parameters=[config],
        ),
    ])
