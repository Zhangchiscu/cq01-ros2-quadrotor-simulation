from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_ground_station_2d',
            executable='ground_station_2d',
            name='drone_ground_station_2d',
            output='screen',
            parameters=[{'path_history_limit': 3000, 'maximum_occupied_points': 6000}],
        )
    ])
