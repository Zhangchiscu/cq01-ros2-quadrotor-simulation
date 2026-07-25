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
    map_share = get_package_share_directory(
        "drone_map"
    )
    planner_share = get_package_share_directory(
        "drone_planner"
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
    map_params = os.path.join(
        map_share,
        "config",
        "obstacles.yaml",
    )
    planner_params = os.path.join(
        planner_share,
        "config",
        "planner.yaml",
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
            parameters=[map_params],
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
            name="avoidance_demo_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
    ])
