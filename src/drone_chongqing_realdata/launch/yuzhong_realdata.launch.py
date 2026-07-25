from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "drone_chongqing_realdata",
                "serve_yuzhong_realdata",
                "--mode",
                "cesium",
            ],
            output="screen",
        ),
    ])
