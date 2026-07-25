from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    share = Path(get_package_share_directory("drone_chongqing_map"))

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package="drone_chongqing_map",
            executable="chongqing_map_server",
            name="chongqing_yuzhong_map_server",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "chongqing_map_preview.rviz")],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
