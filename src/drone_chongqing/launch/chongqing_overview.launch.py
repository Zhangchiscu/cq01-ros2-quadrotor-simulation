import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    show_labels = LaunchConfiguration('show_labels')
    building_count = LaunchConfiguration('building_count')
    rviz_config = os.path.join(
        get_package_share_directory('drone_chongqing'),
        'rviz',
        'chongqing_overview.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('show_labels', default_value='true'),
        DeclareLaunchArgument('building_count', default_value='1900'),
        Node(
            package='drone_chongqing',
            executable='scene_server',
            name='chongqing_scene_server',
            output='screen',
            parameters=[{
                'show_labels': ParameterValue(show_labels, value_type=bool),
                'building_count': ParameterValue(building_count, value_type=int),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='chongqing_overview_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
