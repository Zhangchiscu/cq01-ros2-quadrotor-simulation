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
    dynamics_share = get_package_share_directory(
        "drone_dynamics"
    )
    controller_share = get_package_share_directory(
        "drone_controller"
    )
    planner_share = get_package_share_directory(
        "drone_planner"
    )
    corridor_share = get_package_share_directory(
        "drone_corridor"
    )

    dynamics_params = os.path.join(
        dynamics_share,
        "config",
        "dynamics.yaml",
    )
    controller_params = os.path.join(
        controller_share,
        "config",
        "controller.yaml",
    )
    planner_params = os.path.join(
        planner_share,
        "config",
        "planner.yaml",
    )
    corridor_map_params = os.path.join(
        corridor_share,
        "config",
        "corridor_obstacles.yaml",
    )
    rviz_config = os.path.join(
        planner_share,
        "rviz",
        "avoidance_demo.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),

        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            name="quadrotor_dynamics",
            output="screen",
            parameters=[dynamics_params],
        ),

        Node(
            package="drone_controller",
            executable="quadrotor_controller_node",
            name="quadrotor_controller",
            output="screen",
            parameters=[controller_params],
        ),

        Node(
            package="drone_map",
            executable="static_obstacle_map",
            name="static_obstacle_map",
            output="screen",
            parameters=[corridor_map_params],
        ),

        Node(
            package="drone_bringup",
            executable="drone_visualization",
            name="drone_visualization",
            output="screen",
        ),

        Node(
            package="drone_planner",
            executable="astar_planner",
            name="astar_planner",
            output="screen",
            parameters=[planner_params],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="corridor_demo_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
    ])
