import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    voxel_share = get_package_share_directory(
        "drone_voxel_map"
    )
    rviz_config = os.path.join(
        voxel_share,
        "rviz",
        "voxel_demo.rviz",
    )

    scenario = LaunchConfiguration("scenario")
    use_rviz = LaunchConfiguration("use_rviz")
    resolution = LaunchConfiguration("resolution_m")
    inflation = LaunchConfiguration(
        "inflation_radius_m"
    )
    slice_z = LaunchConfiguration("slice_z_m")
    marker_stride = LaunchConfiguration(
        "marker_stride"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="overflight_3d",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "resolution_m",
            default_value="0.30",
        ),
        DeclareLaunchArgument(
            "inflation_radius_m",
            default_value="0.45",
        ),
        DeclareLaunchArgument(
            "slice_z_m",
            default_value="1.50",
        ),
        DeclareLaunchArgument(
            "marker_stride",
            default_value="1",
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
                "scenario": ParameterValue(
                    scenario,
                    value_type=str,
                ),
                "resolution_m": ParameterValue(
                    resolution,
                    value_type=float,
                ),
                "inflation_radius_m": ParameterValue(
                    inflation,
                    value_type=float,
                ),
                "slice_z_m": ParameterValue(
                    slice_z,
                    value_type=float,
                ),
                "marker_stride": ParameterValue(
                    marker_stride,
                    value_type=int,
                ),
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="voxel_demo_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
    ])
