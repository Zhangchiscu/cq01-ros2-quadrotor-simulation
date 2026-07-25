from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('cq01_v2_scene')
    default_scene = PathJoinSubstitution([pkg_share, 'config', 'cq01_v22_scene.yaml'])
    default_mission = PathJoinSubstitution([pkg_share, 'config', 'cq01_v22_mission.yaml'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'cq01_v22_terrain_focus.rviz'])

    frame_id = LaunchConfiguration('frame_id')

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
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v22_static_tf',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_static_tf')),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', frame_id,
                '--child-frame-id', 'cq01_v22_anchor',
            ],
        ),

        Node(
            package='cq01_v2_scene',
            executable='scene_publisher',
            name='cq01_v22_scene_publisher',
            output='screen',
            parameters=[{
                'scene_file': LaunchConfiguration('scene_file'),
                'mission_file': LaunchConfiguration('mission_file'),
                'frame_id': frame_id,
                'map_topic': LaunchConfiguration('map_topic'),
                'publish_period_s': LaunchConfiguration('publish_period_s'),
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='mission_dispatcher',
            name='cq01_v22_mission_dispatcher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('dispatch_goals')),
            parameters=[{
                'mission_file': LaunchConfiguration('mission_file'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'auto_start': LaunchConfiguration('auto_start'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v22_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
