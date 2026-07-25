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
    package_share = get_package_share_directory(
        'cq01_v9_mc'
    )
    dynamics_share = get_package_share_directory(
        'drone_dynamics'
    )
    controller_share = get_package_share_directory(
        'drone_controller'
    )

    dynamics_params = os.path.join(
        dynamics_share,
        'config',
        'dynamics.yaml',
    )
    controller_params = os.path.join(
        controller_share,
        'config',
        'controller.yaml',
    )
    rviz_file = os.path.join(
        package_share,
        'rviz',
        'cq01_course_acceptance.rviz',
    )

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_course_world_to_map',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ],
        ),

        Node(
            package='drone_dynamics',
            executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics',
            output='screen',
            parameters=[dynamics_params],
        ),

        Node(
            package='drone_controller',
            executable='quadrotor_controller_node',
            name='quadrotor_controller',
            output='screen',
            parameters=[
                controller_params,
                {
                    'maximum_goal_z': 20.0,
                    'maximum_horizontal_acceleration': 3.0,
                    'maximum_upward_acceleration': 4.0,
                    'maximum_downward_acceleration': 4.0,
                    'maximum_tilt_deg': 30.0,
                },
            ],
        ),

        Node(
            package='cq01_v9_mc',
            executable='course_acceptance_runner',
            name='cq01_course_acceptance_runner',
            output='screen',
        ),

        Node(
            package='cq01_v9_mc',
            executable='course_goal_marker',
            name='cq01_course_goal_marker',
            output='screen',
            parameters=[{
                'goal_topic': '/drone/goal',
                'odom_topic': '/drone/odom',
                'marker_topic':
                    '/cq01_course/current_goal_marker',
                'frame_id': 'world',
            }],
        ),

        Node(
            package='cq01_v9_mc',
            executable='v9_visualizer',
            name='cq01_course_drone_visualizer',
            output='screen',
            parameters=[{
                'odom_topic': '/drone/odom',
                'marker_topic':
                    '/cq01_course/drone_model',
                'trajectory_topic':
                    '/cq01_course/actual_trajectory',
                'frame_id': 'world',
                'minimum_path_step_m': 0.03,
                'max_trajectory_points': 2000,
                'model_publish_rate_hz': 20.0,
                'path_publish_rate_hz': 5.0,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_course_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),
    ])
