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
    share = get_package_share_directory('cq01_v2_scene')
    dynamics_share = get_package_share_directory('drone_dynamics')
    controller_share = get_package_share_directory('drone_controller')

    scene_file = os.path.join(
        share,
        'config',
        'cq01_v34g_scene.yaml',
    )
    mission_file = os.path.join(
        share,
        'config',
        'cq01_v34g_planning_mission.yaml',
    )
    rviz_file = os.path.join(
        share,
        'rviz',
        'cq01_v71_single_rviz_short_regression.rviz',
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

    prelude = Node(
        package='cq01_v2_scene',
        executable='v71_prelude_runner',
        name='cq01_v71_prelude_runner',
        output='screen',
    )

    route_dispatcher = Node(
        package='cq01_v2_scene',
        executable='planning_mission_dispatcher',
        name='cq01_v71_route_dispatcher',
        output='screen',
        parameters=[{
            'mission_file': mission_file,
            'goal_topic': '/drone/goal_raw',
            'odom_topic': '/drone/odom',
            'planner_status_topic':
                '/drone/planner_3d/status',
            'mission_status_topic':
                '/cq01_v34h/mission_status',
            'start_index': 0,
            'end_index': 3,
            'auto_start': False,
            'goal_delay_s': 0.05,
            'stall_timeout_s': 60.0,
            'minimum_progress_m': 0.30,
            'maximum_replans_per_goal': 2,
        }],
    )

    h10_manager = Node(
        package='cq01_v2_scene',
        executable='integrated_disturbance_manager',
        name='cq01_v71_h10_manager',
        output='screen',
        parameters=[{
            'trigger_mode': 'manual',
            'manual_countdown_s': 2.0,
            'force_magnitude_n': 2.5,
            'duration_s': 1.8,
            'lateral_sign': 1.0,
            'torque_z_nm': 0.0,
            'fork_center_x_m': 60.70,
            'fork_center_y_m': -85.88,
            'fork_tangent_x': 28.0,
            'fork_tangent_y': 27.0,
            'minimum_route_branch_m': 3.0,
            'trigger_gate_x_m': 52.0,
            'trigger_gate_y_m': -90.0,
            'trigger_gate_z_m': 25.0,
            'trigger_gate_radius_xy_m': 7.0,
            'trigger_gate_tolerance_z_m': 8.0,
            'gate_countdown_s': 0.8,
            'repeat_countdown_s': 1.0,
            'repeat_cooldown_s': 0.4,
            'repeat_zone_radius_xy_m': 55.0,
            'repeat_zone_tolerance_z_m': 18.0,
            'maximum_trigger_count': 3,
            'maximum_queued_repeats': 2,
            'auto_queue_repeats': 2,
            'replan_delay_after_force_s': 0.15,
            'minimum_replan_deviation_m': 2.00,
            'replan_force_timeout_s': 1.00,
            'tracking_tolerance_m': 0.90,
            'tracking_hold_s': 0.70,
            'visual_force_scale_m_per_n': 4.0,
            'marker_publish_rate_hz': 10.0,
            'evidence_hold_s': 20.0,
            'allow_retrigger': True,
            'odom_topic': '/drone/odom',
            'raw_goal_topic': '/drone/goal_raw',
            'path_topic': '/drone/planned_path_3d',
            'planner_status_topic':
                '/drone/planner_3d/status',
            'mission_status_topic':
                '/cq01_v34h/mission_status',
            'disturbance_status_topic':
                '/cq01_v34h/disturbance_status',
            'marker_topic':
                '/cq01_v34h/disturbance_markers',
            'results_root': os.path.expanduser(
                '~/drone_task/your_drone_sim_ws/results/'
                'cq01_v71_short_regression/h10_subresults'
            ),
        }],
    )

    route_handoff = Node(
        package='cq01_v2_scene',
        executable='v712_start_route_and_h10',
        name='cq01_v712_start_route_and_h10',
        output='screen',
    )

    evaluator = Node(
        package='cq01_v2_scene',
        executable='v71_short_regression_evaluator',
        name='cq01_v71_short_regression_evaluator',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v71_world_to_map',
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
            name='cq01_v71_scene_publisher',
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
            name='cq01_v71_voxel_adapter',
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
                    'maximum_goal_z': 300.0,
                    'maximum_horizontal_acceleration': 6.0,
                    'maximum_upward_acceleration': 6.0,
                    'maximum_downward_acceleration': 6.0,
                    'maximum_tilt_deg': 40.0,
                    'kp_x': 2.8,
                    'kp_y': 2.8,
                    'kd_x': 3.2,
                    'kd_y': 3.2,
                    'kp_z': 5.5,
                    'kd_z': 4.2,
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
                'waypoint_tolerance_m': 1.15,
                'waypoint_hold_seconds': 0.02,
                'maximum_waypoint_spacing_m': 6.0,
                'line_of_sight_step_m': 0.25,
                'fixed_height_test_z_m': 30.0,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='drone_model_visualizer',
            name='cq01_v71_drone_model_visualizer',
            output='screen',
            parameters=[{
                'odom_topic': '/drone/odom',
                'marker_topic':
                    '/cq01_v34h/drone_model',
                'trajectory_topic':
                    '/cq01_v34h/actual_trajectory',
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
            executable='v71_unified_camera_focus_manager',
            name='cq01_v71_unified_camera_focus_manager',
            output='screen',
        ),

        Node(
            package='cq01_v2_scene',
            executable='v71_marker_geometry_filter',
            name='cq01_v71_marker_geometry_filter',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v71_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),

        # The original dispatcher stays alive throughout the prelude.
        # It receives the real planner ready state while inactive and ignores
        # all completion events because active=false.
        route_dispatcher,
        h10_manager,
        evaluator,

        prelude,

        RegisterEventHandler(
            OnProcessExit(
                target_action=prelude,
                on_exit=[
                    route_handoff,
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=evaluator,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                'V7.1 original dispatcher/H10 '
                                'short regression finished'
                            )
                        )
                    )
                ],
            )
        ),
    ])
