import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cq01_share = get_package_share_directory('cq01_v2_scene')
    v8a2_share = get_package_share_directory('cq01_v8a2_regression')
    dynamics_share = get_package_share_directory('drone_dynamics')
    controller_share = get_package_share_directory('drone_controller')
    scene_file = os.path.join(cq01_share, 'config', 'cq01_v34g_scene.yaml')
    mission_file = os.path.join(cq01_share, 'config', 'cq01_v34g_planning_mission.yaml')
    rviz_file = os.path.join(v8a2_share, 'rviz', 'cq01_v8a2_dual_location_regression.rviz')
    dynamics_params = os.path.join(dynamics_share, 'config', 'dynamics.yaml')
    controller_params = os.path.join(controller_share, 'config', 'controller.yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    common_h4 = {
        'duration_s': 1.0,
        'ramp_time_s': 0.15,
        'force_x_n': 0.0,
        'force_y_n': 2.5,
        'force_z_n': 0.0,
        'torque_x_nm': 0.0,
        'torque_y_nm': 0.0,
        'torque_z_nm': 0.0,
        'publish_rate_hz': 100.0,
        'recovery_tolerance_m': 0.60,
        'recovery_hold_s': 1.0,
        'maximum_tilt_deg': 45.0,
        'maximum_rpm': 10000.0,
        'saturation_ratio_threshold': 0.98,
        'maximum_saturation_fraction': 0.05,
        'mission_status_topic': '/cq01_v8a2/mission_status',
        'odom_topic': '/drone/odom',
        'path_topic': '/drone/planned_path_3d',
        'rpm_topic': '/drone/motor_rpm',
        'applied_wrench_topic': '/drone/external_wrench_applied',
        'visual_force_scale_m_per_n': 4.0,
        'visual_afterglow_s': 4.0,
        'visual_recovery_hold_s': 20.0,
        'trail_sample_period_s': 0.10,
        'status_heartbeat_s': 0.5,
        'marker_publish_rate_hz': 8.0,
        'visual_evidence_hold_s': 12.0,
        'allow_retrigger': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='cq01_v8a2_world_to_map', output='screen',
            arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0','--frame-id','world','--child-frame-id','map'],
        ),
        Node(
            package='cq01_v2_scene', executable='scene_publisher',
            name='cq01_v8a2_scene_publisher', output='screen',
            parameters=[{'scene_file': scene_file, 'mission_file': mission_file, 'frame_id': 'world', 'map_topic': '/map_generator/global_cloud', 'publish_period_s': 10.0}],
        ),
        Node(
            package='cq01_v2_scene', executable='voxel_adapter',
            name='cq01_v8a2_voxel_adapter', output='screen',
            parameters=[{'scene_file': scene_file, 'frame_id': 'map', 'resolution_m': 2.0, 'inflation_radius_m': 2.75, 'publish_period_s': 3600.0, 'include_visual_only_boxes': True}],
        ),
        Node(package='drone_dynamics', executable='quadrotor_dynamics_node', name='quadrotor_dynamics', output='screen', parameters=[dynamics_params]),
        Node(
            package='drone_controller', executable='quadrotor_controller_node',
            name='quadrotor_controller', output='screen',
            parameters=[controller_params, {'maximum_goal_z': 300.0, 'maximum_horizontal_acceleration': 6.0, 'maximum_upward_acceleration': 6.0, 'maximum_downward_acceleration': 6.0, 'maximum_tilt_deg': 40.0, 'kp_x': 2.8, 'kp_y': 2.8, 'kd_x': 3.2, 'kd_y': 3.2, 'kp_z': 5.5, 'kd_z': 4.2, 'ki_x': 0.03, 'ki_y': 0.03}],
        ),
        Node(
            package='drone_planner_3d', executable='astar_3d_planner',
            name='astar_3d_planner', output='screen',
            parameters=[{'minimum_flight_z_m': 1.0, 'maximum_flight_z_m': 285.0, 'vertical_cost_factor': 1.15, 'waypoint_tolerance_m': 1.15, 'waypoint_hold_seconds': 0.02, 'maximum_waypoint_spacing_m': 6.0, 'line_of_sight_step_m': 0.25, 'fixed_height_test_z_m': 30.0}],
        ),
        Node(
            package='cq01_v2_scene', executable='planning_mission_dispatcher',
            name='cq01_v8a2_route_dispatcher', output='screen',
            parameters=[{'mission_file': mission_file, 'goal_topic': '/drone/goal_raw', 'odom_topic': '/drone/odom', 'planner_status_topic': '/drone/planner_3d/status', 'mission_status_topic': '/cq01_v8a2/mission_status', 'start_index': 0, 'end_index': 4, 'auto_start': True, 'goal_delay_s': 0.05, 'stall_timeout_s': 60.0, 'minimum_progress_m': 0.30, 'maximum_replans_per_goal': 2}],
        ),
        Node(
            package='cq01_v2_scene', executable='drone_model_visualizer',
            name='cq01_v8a2_drone_model_visualizer', output='screen',
            parameters=[{'odom_topic': '/drone/odom', 'marker_topic': '/cq01_v8a2/drone_model', 'trajectory_topic': '/cq01_v8a2/actual_trajectory', 'frame_id': 'world', 'arm_half_span_m': 0.55, 'body_length_m': 0.65, 'body_width_m': 0.42, 'body_height_m': 0.24, 'rotor_diameter_m': 0.45}],
        ),
        Node(
            package='cq01_v8a2_regression', executable='v8a2_low_city_disturbance',
            name='cq01_v8a2_low_disturbance', output='screen',
            remappings=[('/tf', '/cq01_v8a2/low/tf_unused')],
            parameters=[dict(common_h4, **{
                'trigger_mode': 'manual', 'manual_countdown_s': 0.8,
                'disturbance_status_topic': '/cq01_v8a2/low/status',
                'command_wrench_topic': '/cq01_v8a2/low/wrench_cmd',
                'active_topic': '/cq01_v8a2/low/active',
                'marker_topic': '/cq01_v8a2/low/markers',
                'results_root': os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v8a2_dual_location/low_authoritative'),
            })],
        ),
        Node(
            package='cq01_v8a2_regression', executable='v8a2_climb_h4',
            name='cq01_v8a2_climb_disturbance', output='screen',
            remappings=[('/tf', '/cq01_v8a2/climb/tf_unused')],
            parameters=[dict(common_h4, **{
                'trigger_mode': 'waypoint', 'trigger_after_reached_index': 3, 'trigger_delay_s': 3.0,
                'disturbance_status_topic': '/cq01_v8a2/climb/status',
                'command_wrench_topic': '/cq01_v8a2/climb/wrench_cmd',
                'active_topic': '/cq01_v8a2/climb/active',
                'marker_topic': '/cq01_v8a2/climb/markers',
                'results_root': os.path.expanduser('~/drone_task/your_drone_sim_ws/results/cq01_v8a2_dual_location/climb_authoritative'),
            })],
        ),
        Node(package='cq01_v8a2_regression', executable='v8a2_wrench_mux', name='cq01_v8a2_wrench_mux', output='screen'),
        Node(package='cq01_v8a2_regression', executable='v8a2_low_gate_trigger', name='cq01_v8a2_low_gate_trigger', output='screen'),
        Node(package='cq01_v8a2_regression', executable='v8a2_camera_focus', name='cq01_v8a2_camera_focus', output='screen'),
        Node(package='cq01_v8a2_regression', executable='v8a2_runtime_monitor', name='cq01_v8a2_runtime_monitor', output='screen'),
        Node(package='rviz2', executable='rviz2', name='cq01_v8a2_rviz', output='screen', condition=IfCondition(use_rviz), arguments=['-d', rviz_file]),
    ])
