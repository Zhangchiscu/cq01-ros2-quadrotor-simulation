from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    display_vertical_scale = LaunchConfiguration("display_vertical_scale")
    show_buildings = LaunchConfiguration("show_buildings")
    show_transport = LaunchConfiguration("show_transport")

    share = Path(get_package_share_directory("drone_chongqing_fullscene"))

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("display_vertical_scale", default_value="2.5"),
        DeclareLaunchArgument("show_buildings", default_value="true"),
        DeclareLaunchArgument("show_transport", default_value="true"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chongqing_scene_center_tf",
            arguments=[
                "--x", "3000",
                "--y", "1905",
                "--z", "300",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "map",
                "--child-frame-id", "scene_center",
            ],
            output="screen",
        ),
        Node(
            package="drone_chongqing_fullscene",
            executable="full_scene_server",
            name="chongqing_full_yuzhong_scene",
            output="screen",
            parameters=[{
                "display_vertical_scale": ParameterValue(
                    display_vertical_scale,
                    value_type=float,
                ),
                "show_buildings": ParameterValue(
                    show_buildings,
                    value_type=bool,
                ),
                "show_transport": ParameterValue(
                    show_transport,
                    value_type=bool,
                ),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(share / "rviz" / "chongqing_full_scene.rviz"),
            ],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
