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
from launch.substitutions import (
    LaunchConfiguration,
)
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
    mission_share = get_package_share_directory(
        'drone_mission'
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
    mission_params = os.path.join(
        mission_share,
        'config',
        'mission.yaml',
    )
    rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'quadrotor.rviz',
    )

    use_rviz = LaunchConfiguration('use_rviz')

    mission_node = Node(
        package='drone_mission',
        executable='multi_goal_experiment',
        name='multi_goal_experiment',
        output='screen',
        parameters=[mission_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2',
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

        mission_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=mission_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                'Multi-goal mission finished'
                            )
                        )
                    )
                ],
            )
        ),
    ])
