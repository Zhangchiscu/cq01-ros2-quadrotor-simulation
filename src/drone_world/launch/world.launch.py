import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    use_rviz = LaunchConfiguration('use_rviz')
    point_spacing = LaunchConfiguration('point_spacing_m')
    show_labels = LaunchConfiguration('show_labels')
    rviz_config = os.path.join(
        get_package_share_directory('drone_world'),
        'rviz',
        'world.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='warehouse'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('point_spacing_m', default_value='0.35'),
        DeclareLaunchArgument('show_labels', default_value='true'),
        Node(
            package='drone_world',
            executable='world_server',
            name='world_server',
            output='screen',
            parameters=[{
                'scenario': scenario,
                'point_spacing_m': point_spacing,
                'show_labels': show_labels,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='drone_world_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
