from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    bringup = os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'quadrotor_sim.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={'use_rviz': use_rviz, 'auto_goal': 'false'}.items(),
        ),
        Node(package='drone_ground_station_2d', executable='ground_station_2d', name='drone_ground_station_2d', output='screen'),
    ])
