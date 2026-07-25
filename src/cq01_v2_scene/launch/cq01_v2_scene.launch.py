from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('cq01_v2_scene')
    default_scene = PathJoinSubstitution([pkg_share, 'config', 'cq01_v2_scene.yaml'])
    default_mission = PathJoinSubstitution([pkg_share, 'config', 'cq01_v2_mission.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('scene_file', default_value=default_scene),
        DeclareLaunchArgument('mission_file', default_value=default_mission),
        DeclareLaunchArgument('frame_id', default_value='world'),
        DeclareLaunchArgument('map_topic', default_value='/map_generator/global_cloud'),
        DeclareLaunchArgument('publish_period_s', default_value='1.0'),
        DeclareLaunchArgument('dispatch_goals', default_value='false'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('odom_topic', default_value='/quadrotor/odom'),
        DeclareLaunchArgument('auto_start', default_value='false'),

        Node(
            package='cq01_v2_scene',
            executable='scene_publisher',
            name='cq01_v2_scene_publisher',
            output='screen',
            parameters=[{
                'scene_file': LaunchConfiguration('scene_file'),
                'mission_file': LaunchConfiguration('mission_file'),
                'frame_id': LaunchConfiguration('frame_id'),
                'map_topic': LaunchConfiguration('map_topic'),
                'publish_period_s': LaunchConfiguration('publish_period_s'),
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='mission_dispatcher',
            name='cq01_v2_mission_dispatcher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('dispatch_goals')),
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'auto_start': LaunchConfiguration('auto_start'),
            }],
        ),
    ])
