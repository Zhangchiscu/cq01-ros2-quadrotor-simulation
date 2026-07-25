import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamics_share = get_package_share_directory('drone_dynamics')
    controller_share = get_package_share_directory('drone_controller')
    cq_share = FindPackageShare('cq01_v2_scene')

    scene_file = PathJoinSubstitution([
        cq_share, 'config', 'cq01_v34_scene.yaml',
    ])
    mission_file = PathJoinSubstitution([
        cq_share, 'config', 'cq01_v34e_planning_mission.yaml',
    ])
    rviz_file = PathJoinSubstitution([
        cq_share, 'rviz', 'cq01_v34e_complete_planning.rviz',
    ])

    dynamics_params = os.path.join(
        dynamics_share, 'config', 'dynamics.yaml'
    )
    controller_params = os.path.join(
        controller_share, 'config', 'controller.yaml'
    )

    resolution = LaunchConfiguration('resolution_m')
    inflation = LaunchConfiguration('inflation_radius_m')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start_mission')
    start_index = LaunchConfiguration('mission_start_index')
    end_index = LaunchConfiguration('mission_end_index')
    target_speed = LaunchConfiguration('target_cruise_speed_mps')
    max_lookahead = LaunchConfiguration('maximum_lookahead_m')
    target_time = LaunchConfiguration('target_total_time_s')

    planner_parameters = [{
        'minimum_flight_z_m': 2.0,
        'maximum_flight_z_m': 285.0,
        'vertical_cost_factor': 1.15,
        'waypoint_tolerance_m': 0.80,
        'waypoint_hold_seconds': 0.05,
        'maximum_waypoint_spacing_m': 2.0,
        'line_of_sight_step_m': 0.5,
        'fixed_height_test_z_m': 30.0,
    }]

    return LaunchDescription([
        DeclareLaunchArgument('resolution_m', default_value='2.0'),
        DeclareLaunchArgument('inflation_radius_m', default_value='3.5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start_mission', default_value='false'),
        DeclareLaunchArgument('mission_start_index', default_value='0'),
        DeclareLaunchArgument('mission_end_index', default_value='0'),
        DeclareLaunchArgument(
            'target_cruise_speed_mps', default_value='3.0'
        ),
        DeclareLaunchArgument('maximum_lookahead_m', default_value='10.0'),
        DeclareLaunchArgument('target_total_time_s', default_value='900.0'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v34e_world_to_map',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ],
        ),

        Node(
            package='cq01_v2_scene',
            executable='scene_publisher',
            name='cq01_v34_scene_publisher',
            output='screen',
            parameters=[{
                'scene_file': scene_file,
                'mission_file': mission_file,
                'frame_id': 'world',
                'map_topic': '/map_generator/global_cloud',
                'publish_period_s': 10.0,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='voxel_adapter',
            name='cq01_v34e_voxel_adapter',
            output='screen',
            parameters=[{
                'scene_file': scene_file,
                'frame_id': 'map',
                'resolution_m': ParameterValue(
                    resolution, value_type=float
                ),
                'inflation_radius_m': ParameterValue(
                    inflation, value_type=float
                ),
                'publish_period_s': 3600.0,
                'include_visual_only_boxes': True,
            }],
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

        # The planner only plans. Its dense waypoint control output is disabled.
        Node(
            package='drone_planner_3d',
            executable='astar_3d_planner',
            name='astar_3d_planner',
            output='screen',
            parameters=planner_parameters,
            remappings=[
                ('/drone/goal', '/drone/planner_direct_goal_disabled'),
            ],
        ),

        Node(
            package='cq01_v2_scene',
            executable='complete_path_follower',
            name='cq01_v34e_complete_path_follower',
            output='screen',
            parameters=[{
                'control_rate_hz': 25.0,
                'target_cruise_speed_mps': ParameterValue(
                    target_speed, value_type=float
                ),
                'base_lookahead_m': 5.0,
                'speed_error_lookahead_gain_s': 1.0,
                'minimum_lookahead_m': 2.5,
                'maximum_lookahead_m': ParameterValue(
                    max_lookahead, value_type=float
                ),
                'lookahead_step_m': 1.0,
                'collision_check_spacing_m': 0.45,
                'completion_tolerance_m': 1.25,
                'completion_hold_seconds': 0.35,
                'stall_timeout_seconds': 60.0,
                'minimum_progress_m': 0.5,
                'status_period_seconds': 1.0,
                'bootstrap_takeoff_altitude_m': 5.0,
                'bootstrap_altitude_tolerance_m': 0.35,
                'bootstrap_timeout_seconds': 60.0,
                'emergency_minimum_lookahead_m': 0.50,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='drone_model_visualizer',
            name='cq01_v34e_drone_model_visualizer',
            output='screen',
            parameters=[{
                'odom_topic': '/drone/odom',
                'marker_topic': '/cq01_v34e/drone_model',
                'trajectory_topic': '/cq01_v34e/actual_trajectory',
                'frame_id': 'world',
                'arm_half_span_m': 0.55,
                'body_length_m': 0.65,
                'body_width_m': 0.42,
                'body_height_m': 0.24,
                'rotor_diameter_m': 0.45,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='complete_mission_dispatcher',
            name='cq01_v34e_complete_mission_dispatcher',
            output='screen',
            parameters=[{
                'mission_file': mission_file,
                'goal_topic': '/drone/goal_raw',
                'planner_status_topic': '/drone/planner_3d/status',
                'follower_status_topic': '/cq01_v34e/follower_status',
                'mission_status_topic': '/cq01_v34e/mission_status',
                'start_index': ParameterValue(
                    start_index, value_type=int
                ),
                'end_index': ParameterValue(
                    end_index, value_type=int
                ),
                'auto_start': ParameterValue(
                    auto_start, value_type=bool
                ),
                'goal_delay_s': 0.10,
                'maximum_replans_per_goal': 4,
                'target_total_time_s': ParameterValue(
                    target_time, value_type=float
                ),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v34e_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),
    ])
