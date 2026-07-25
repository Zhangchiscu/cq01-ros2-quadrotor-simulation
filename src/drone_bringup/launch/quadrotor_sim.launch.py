from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import (
    get_package_share_directory,
)
import os


def generate_launch_description():
    dynamics_share = get_package_share_directory(
        'drone_dynamics'
    )
    controller_share = get_package_share_directory(
        'drone_controller'
    )
    bringup_share = get_package_share_directory(
        'drone_bringup'
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
    rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'quadrotor.rviz',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2',
        ),
        DeclareLaunchArgument(
            'auto_goal',
            default_value='true',
            description='Publish a default goal after startup',
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'goal_z',
            default_value='1.5',
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
            parameters=[controller_params],
        ),

        Node(
            package='drone_bringup',
            executable='drone_visualization',
            name='drone_visualization',
            output='screen',
        ),

        Node(
            package='drone_bringup',
            executable='default_goal',
            name='default_goal_publisher',
            output='screen',
            condition=IfCondition(auto_goal),
            parameters=[{
                'goal_x': ParameterValue(goal_x, value_type=float),
                'goal_y': ParameterValue(goal_y, value_type=float),
                'goal_z': ParameterValue(goal_z, value_type=float),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
