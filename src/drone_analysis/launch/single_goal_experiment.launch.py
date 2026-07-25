import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')
    duration_s = LaunchConfiguration('duration_s')
    arrival_tolerance_m = LaunchConfiguration(
        'arrival_tolerance_m'
    )

    experiment_node = Node(
        package='drone_analysis',
        executable='single_goal_experiment',
        name='single_goal_experiment',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_z': goal_z,
            'duration_s': duration_s,
            'arrival_tolerance_m': arrival_tolerance_m,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2 during the experiment',
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
        DeclareLaunchArgument(
            'duration_s',
            default_value='20.0',
        ),
        DeclareLaunchArgument(
            'arrival_tolerance_m',
            default_value='0.30',
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
            condition=IfCondition(use_rviz),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),

        experiment_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Experiment finished'
                        )
                    )
                ],
            )
        ),
    ])
