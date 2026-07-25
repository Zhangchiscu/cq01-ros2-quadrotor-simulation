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
    cq01_share = get_package_share_directory(
        'cq01_v2_scene'
    )
    v8b_share = get_package_share_directory(
        'cq01_v8b_regression'
    )
    dynamics_share = get_package_share_directory(
        'drone_dynamics'
    )
    controller_share = get_package_share_directory(
        'drone_controller'
    )

    scene_file = os.path.join(
        cq01_share,
        'config',
        'cq01_v34g_scene.yaml',
    )
    mission_file = os.path.join(
        cq01_share,
        'config',
        'cq01_v34g_planning_mission.yaml',
    )
    rviz_file = os.path.join(
        v8b_share,
        'rviz',
        'cq01_v8b_high_altitude_pattern.rviz',
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

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v8b_world_to_map',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ],
        ),

        Node(
            package='cq01_v2_scene',
            executable='scene_publisher',
            name='cq01_v8b_scene_publisher',
            output='screen',
            parameters=[{
                'scene_file': scene_file,
                'mission_file': mission_file,
                'frame_id': 'world',
                'map_topic':
                    '/map_generator/global_cloud',
                'publish_period_s': 10.0,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='voxel_adapter',
            name='cq01_v8b_voxel_adapter',
            output='screen',
            parameters=[{
                'scene_file': scene_file,
                'frame_id': 'map',
                'resolution_m': 2.0,
                'inflation_radius_m': 2.75,
                'publish_period_s': 3600.0,
                'include_visual_only_boxes': True,
            }],
        ),

        Node(
            package='drone_dynamics',
            executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics',
            output='screen',
            parameters=[
                dynamics_params,
                {
                    'initial_position_x': 309.0,
                    'initial_position_y': -45.0,
                    'initial_position_z': 73.0,
                    'initial_velocity_x': 0.0,
                    'initial_velocity_y': 0.0,
                    'initial_velocity_z': 0.0,
                    'initial_yaw_deg': 0.0,
                },
            ],
        ),

        Node(
            package='drone_controller',
            executable='quadrotor_controller_node',
            name='quadrotor_controller',
            output='screen',
            parameters=[
                controller_params,
                {
                    'maximum_goal_z': 300.0,
                    'maximum_horizontal_acceleration': 6.0,
                    'maximum_upward_acceleration': 6.0,
                    'maximum_downward_acceleration': 6.0,
                    'maximum_tilt_deg': 40.0,
                    'kp_x': 3.0,
                    'kp_y': 3.0,
                    'kd_x': 3.3,
                    'kd_y': 3.3,
                    'kp_z': 6.0,
                    'kd_z': 4.5,
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
            remappings=[
                (
                    '/drone/goal',
                    '/drone/goal_planner',
                ),
            ],
            parameters=[{
                'minimum_flight_z_m': 1.0,
                'maximum_flight_z_m': 285.0,
                'vertical_cost_factor': 1.15,
                'waypoint_tolerance_m': 1.15,
                'waypoint_hold_seconds': 0.02,
                'maximum_waypoint_spacing_m': 6.0,
                'line_of_sight_step_m': 0.30,
                'fixed_height_test_z_m': 30.0,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='planning_mission_dispatcher',
            name='cq01_v8b_route_dispatcher',
            output='screen',
            parameters=[{
                'mission_file': mission_file,
                'goal_topic':
                    '/cq01_v8b/route_goal_raw',
                'odom_topic':
                    '/drone/odom',
                'planner_status_topic':
                    '/drone/planner_3d/status',
                'mission_status_topic':
                    '/cq01_v8b/route_status',
                'start_index': 11,
                'end_index': 15,
                'auto_start': True,
                'goal_delay_s': 0.05,
                'stall_timeout_s': 75.0,
                'minimum_progress_m': 0.30,
                'maximum_replans_per_goal': 2,
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_raw_goal_mux',
            name='cq01_v8b_raw_goal_mux',
            output='screen',
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_high_altitude_wind',
            name='cq01_v8b_high_altitude_wind',
            output='screen',
            parameters=[{
                'mission_status_topic':
                    '/cq01_v8b/route_status',
                'odom_topic':
                    '/drone/odom',
                'raw_goal_topic':
                    '/drone/goal_raw',
                'replan_goal_topic':
                    '/cq01_v8b/h14_replan_goal_raw',
                'path_topic':
                    '/drone/planned_path_3d',
                'planner_status_topic':
                    '/drone/planner_3d/status',
                'rpm_topic':
                    '/drone/motor_rpm',
                'applied_wrench_topic':
                    '/drone/external_wrench_applied',
                'command_wrench_topic':
                    '/drone/external_wrench',
                'active_topic':
                    '/cq01_v8b/h14/active',
                'status_topic':
                    '/cq01_v8b/h14/status',
                'marker_topic':
                    '/cq01_v8b/h14/markers',
                'arm_after_reached_index':
                    12,
                'gate_x_m':
                    334.0,
                'gate_y_m':
                    -31.0,
                'gate_z_m':
                    162.0,
                'gate_radius_xy_m':
                    13.0,
                'gate_tolerance_z_m':
                    22.0,
                'countdown_s':
                    1.2,
                'force_magnitude_n':
                    2.85,
                'duration_s':
                    4.2,
                'minimum_gust_duration_s':
                    1.2,
                'target_deviation_m':
                    5.0,
                'ramp_time_s':
                    0.30,
                'yaw_torque_nm':
                    0.020,
                'soft_tilt_start_deg':
                    39.0,
                'soft_tilt_end_deg':
                    43.4,
                'minimum_force_scale':
                    0.30,
                'gust_frequency_hz':
                    0.9,
                'lateral_sign':
                    -1.0,
                'minimum_replan_deviation_m':
                    1.50,
                'replan_timeout_s':
                    0.80,
                'tracking_tolerance_m':
                    1.10,
                'tracking_hold_s':
                    0.80,
                'evidence_hold_s':
                    5.0,
                'maximum_retracking_wait_s':
                    20.0,
                'safety_tilt_deg':
                    44.0,
                'maximum_rpm':
                    10000.0,
                'safety_rpm_ratio':
                    0.985,
                'minimum_visible_branch_m':
                    1.0,
                'results_root':
                    os.path.expanduser(
                        '~/drone_task/your_drone_sim_ws/'
                        'results/'
                        'cq01_v8b_high_altitude_pattern/'
                        'h14_original'
                    ),
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_summit_pattern_manager',
            name='cq01_v8b_summit_pattern_manager',
            output='screen',
            parameters=[{
                'frame_id':
                    'world',
                'odom_topic':
                    '/drone/odom',
                'goal_topic':
                    '/drone/goal_pattern',
                'status_topic':
                    '/cq01_v8b/pattern/status',
                'reference_path_topic':
                    '/cq01_v8b/pattern/reference_path',
                'marker_topic':
                    '/cq01_v8b/pattern/reference_markers',
                'summit_x_m':
                    410.0,
                'summit_y_m':
                    45.0,
                'summit_z_m':
                    219.878,
                'center_x_m':
                    400.0,
                'center_y_m':
                    45.0,
                'center_z_m':
                    250.0,
                'circle_radius_m':
                    10.0,
                'circle_period_s':
                    16.0,
                'figure8_x_radius_m':
                    10.0,
                'figure8_y_radius_m':
                    6.5,
                'figure8_period_s':
                    22.0,
                'entry_tolerance_m':
                    1.5,
                'summit_hold_s':
                    0.5,
                'rapid_ascent_s':
                    8.0,
                'top_hold_s':
                    0.8,
                'between_pattern_hold_s':
                    0.6,
                'final_hold_s':
                    3.0,
                'publish_rate_hz':
                    20.0,
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_pattern_tracking_evaluator',
            name='cq01_v8b_pattern_evaluator',
            output='screen',
            parameters=[{
                'frame_id':
                    'world',
                'odom_topic':
                    '/drone/odom',
                'status_topic':
                    '/cq01_v8b/pattern/status',
                'rpm_topic':
                    '/drone/motor_rpm',
                'marker_topic':
                    '/cq01_v8b/pattern/evaluator_markers',
                'results_root':
                    os.path.expanduser(
                        '~/drone_task/your_drone_sim_ws/'
                        'results/'
                        'cq01_v8b_high_altitude_pattern/'
                        'pattern_original'
                    ),
                'maximum_duration_s':
                    120.0,
                'sample_rate_hz':
                    20.0,
                'maximum_rpm':
                    10000.0,
                'rpm_saturation_ratio':
                    0.98,
                'shutdown_after_result_s':
                    1.0,
                'shutdown_on_complete':
                    False,
                'result_topic':
                    '/cq01_v8b/pattern/result',
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_goal_mux',
            name='cq01_v8b_goal_mux',
            output='screen',
            parameters=[{
                'planner_goal_topic':
                    '/drone/goal_planner',
                'pattern_goal_topic':
                    '/drone/goal_pattern',
                'output_goal_topic':
                    '/drone/goal',
                'mode_topic':
                    '/cq01_v8b/control_mode',
                'initial_mode':
                    'planner',
                'republish_rate_hz':
                    20.0,
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_summit_orchestrator',
            name='cq01_v8b_summit_orchestrator',
            output='screen',
            parameters=[{
                'route_status_topic':
                    '/cq01_v8b/route_status',
                'pattern_status_topic':
                    '/cq01_v8b/pattern/status',
                'integrated_status_topic':
                    '/cq01_v8b/integrated/status',
                'control_mode_topic':
                    '/cq01_v8b/control_mode',
                'pattern_start_service':
                    '/cq01_v8b_summit_pattern_manager/start',
                'handoff_delay_s':
                    0.5,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='drone_model_visualizer',
            name='cq01_v8b_drone_model_visualizer',
            output='screen',
            parameters=[{
                'odom_topic':
                    '/drone/odom',
                'marker_topic':
                    '/cq01_v8b/drone_model',
                'trajectory_topic':
                    '/cq01_v8b/actual_trajectory',
                'frame_id':
                    'world',
                'arm_half_span_m':
                    0.55,
                'body_length_m':
                    0.65,
                'body_width_m':
                    0.42,
                'body_height_m':
                    0.24,
                'rotor_diameter_m':
                    0.45,
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_camera_focus',
            name='cq01_v8b_camera_focus',
            output='screen',
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_runtime_monitor',
            name='cq01_v8b_runtime_monitor',
            output='screen',
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_marker_text_filter',
            name='cq01_v8b_h14_marker_filter',
            output='screen',
            parameters=[{
                'input_topic':
                    '/cq01_v8b/h14/markers',
                'output_topic':
                    '/cq01_v8b/h14/markers_clean',
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_marker_text_filter',
            name='cq01_v8b_pattern_reference_filter',
            output='screen',
            parameters=[{
                'input_topic':
                    '/cq01_v8b/pattern/reference_markers',
                'output_topic':
                    '/cq01_v8b/pattern/reference_markers_clean',
            }],
        ),

        Node(
            package='cq01_v8b_regression',
            executable='v8b_marker_text_filter',
            name='cq01_v8b_pattern_evaluator_filter',
            output='screen',
            parameters=[{
                'input_topic':
                    '/cq01_v8b/pattern/evaluator_markers',
                'output_topic':
                    '/cq01_v8b/pattern/evaluator_markers_clean',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v8b_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),
    ])
