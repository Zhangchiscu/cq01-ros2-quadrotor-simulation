import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('cq01_v2_scene')
    dynamics_share = get_package_share_directory('drone_dynamics')
    controller_share = get_package_share_directory('drone_controller')

    scene_file = os.path.join(share, 'config', 'cq01_v34g_scene.yaml')
    mission_file = os.path.join(
        share, 'config', 'cq01_v34g_planning_mission.yaml'
    )
    rviz_file = os.path.join(
        share, 'rviz', 'cq01_v34g_dense_fast_planning.rviz'
    )
    dynamics_params = os.path.join(
        dynamics_share, 'config', 'dynamics.yaml'
    )
    controller_params = os.path.join(
        controller_share, 'config', 'controller.yaml'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start_mission')
    start_index = LaunchConfiguration('mission_start_index')
    end_index = LaunchConfiguration('mission_end_index')
    resolution = LaunchConfiguration('resolution_m')
    inflation = LaunchConfiguration('inflation_radius_m')
    waypoint_spacing = LaunchConfiguration(
        'maximum_waypoint_spacing_m'
    )
    waypoint_tolerance = LaunchConfiguration(
        'waypoint_tolerance_m'
    )
    horizontal_accel = LaunchConfiguration(
        'maximum_horizontal_acceleration'
    )
    vertical_accel = LaunchConfiguration(
        'maximum_vertical_acceleration'
    )
    max_tilt = LaunchConfiguration('maximum_tilt_deg')
    kp_xy = LaunchConfiguration('kp_xy')
    kd_xy = LaunchConfiguration('kd_xy')
    kp_z = LaunchConfiguration('kp_z')
    kd_z = LaunchConfiguration('kd_z')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'auto_start_mission', default_value='false'
        ),
        DeclareLaunchArgument('mission_start_index', default_value='0'),
        DeclareLaunchArgument('mission_end_index', default_value='15'),
        DeclareLaunchArgument('resolution_m', default_value='2.0'),
        DeclareLaunchArgument(
            'inflation_radius_m', default_value='2.75'
        ),
        DeclareLaunchArgument(
            'maximum_waypoint_spacing_m', default_value='6.0'
        ),
        DeclareLaunchArgument(
            'waypoint_tolerance_m', default_value='1.15'
        ),
        DeclareLaunchArgument(
            'maximum_horizontal_acceleration', default_value='6.0'
        ),
        DeclareLaunchArgument(
            'maximum_vertical_acceleration', default_value='6.0'
        ),
        DeclareLaunchArgument(
            'maximum_tilt_deg', default_value='40.0'
        ),
        DeclareLaunchArgument('kp_xy', default_value='2.8'),
        DeclareLaunchArgument('kd_xy', default_value='3.2'),
        DeclareLaunchArgument('kp_z', default_value='5.5'),
        DeclareLaunchArgument('kd_z', default_value='4.2'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v34g_world_to_map',
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
            name='cq01_v34g_scene_publisher',
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
            name='cq01_v34g_voxel_adapter',
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
            parameters=[
                controller_params,
                {
                    # Enforce the already-proven altitude fix at runtime.
                    'maximum_goal_z': 300.0,
                    # Faster but deliberately well-damped position control.
                    'maximum_horizontal_acceleration':
                        ParameterValue(
                            horizontal_accel, value_type=float
                        ),
                    'maximum_upward_acceleration':
                        ParameterValue(
                            vertical_accel, value_type=float
                        ),
                    'maximum_downward_acceleration':
                        ParameterValue(
                            vertical_accel, value_type=float
                        ),
                    'maximum_tilt_deg':
                        ParameterValue(max_tilt, value_type=float),
                    'kp_x': ParameterValue(kp_xy, value_type=float),
                    'kp_y': ParameterValue(kp_xy, value_type=float),
                    'kd_x': ParameterValue(kd_xy, value_type=float),
                    'kd_y': ParameterValue(kd_xy, value_type=float),
                    'kp_z': ParameterValue(kp_z, value_type=float),
                    'kd_z': ParameterValue(kd_z, value_type=float),
                    'ki_x': 0.03,
                    'ki_y': 0.03,
                },
            ],
        ),
        Node(
            package='drone_planner_3d',
            executable='astar_3d_planner',
            name='astar_3d_planner',
            output='screen',
            parameters=[{
                'minimum_flight_z_m': 1.0,
                'maximum_flight_z_m': 285.0,
                'vertical_cost_factor': 1.15,
                'waypoint_tolerance_m': ParameterValue(
                    waypoint_tolerance, value_type=float
                ),
                'waypoint_hold_seconds': 0.02,
                'maximum_waypoint_spacing_m': ParameterValue(
                    waypoint_spacing, value_type=float
                ),
                'line_of_sight_step_m': 0.25,
                'fixed_height_test_z_m': 30.0,
            }],
        ),
        Node(
            package='cq01_v2_scene',
            executable='planning_mission_dispatcher',
            name='cq01_v34g_dense_fast_mission_dispatcher',
            output='screen',
            parameters=[{
                'mission_file': mission_file,
                'goal_topic': '/drone/goal_raw',
                'odom_topic': '/drone/odom',
                'planner_status_topic': '/drone/planner_3d/status',
                'mission_status_topic': '/cq01_v34g/mission_status',
                'start_index': ParameterValue(
                    start_index, value_type=int
                ),
                'end_index': ParameterValue(
                    end_index, value_type=int
                ),
                'auto_start': ParameterValue(
                    auto_start, value_type=bool
                ),
                'goal_delay_s': 0.05,
                'stall_timeout_s': 60.0,
                'minimum_progress_m': 0.30,
                'maximum_replans_per_goal': 2,
            }],
        ),
        Node(
            package='cq01_v2_scene',
            executable='mission_timer',
            name='cq01_v34g_mission_timer',
            output='screen',
            parameters=[{
                'status_topic': '/cq01_v34g/mission_status',
                'target_time_s': 180.0,
            }],
        ),
        Node(
            package='cq01_v2_scene',
            executable='drone_model_visualizer',
            name='cq01_v34g_drone_model_visualizer',
            output='screen',
            parameters=[{
                'odom_topic': '/drone/odom',
                'marker_topic': '/cq01_v34g/drone_model',
                'trajectory_topic': '/cq01_v34g/actual_trajectory',
                'frame_id': 'world',
                'arm_half_span_m': 0.55,
                'body_length_m': 0.65,
                'body_width_m': 0.42,
                'body_height_m': 0.24,
                'rotor_diameter_m': 0.45,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v34g_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),
    ])
