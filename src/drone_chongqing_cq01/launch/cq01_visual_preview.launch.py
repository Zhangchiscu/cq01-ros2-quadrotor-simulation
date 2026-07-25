import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("drone_chongqing_cq01")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),

        # The standalone preview has no dynamics node, so it must create
        # the map frame itself. Without this transform RViz receives all
        # MarkerArray messages but cannot render them.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="cq01_preview_map_tf",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "map",
            ],
            output="screen",
        ),

        Node(
            package="drone_chongqing_cq01",
            executable="cq01_map_server",
            name="cq01_eling_liziba_map_server",
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
            name="cq01_visual_preview_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=[
                "-d",
                os.path.join(
                    share,
                    "rviz",
                    "cq01_eling_liziba.rviz",
                ),
            ],
        ),
    ])
