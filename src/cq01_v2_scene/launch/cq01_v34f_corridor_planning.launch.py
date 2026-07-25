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

    scene_file = os.path.join(share, 'config', 'cq01_v34f_scene.yaml')
    mission_file = os.path.join(share, 'config', 'cq01_v34f_planning_mission.yaml')
    rviz_file = os.path.join(share, 'rviz', 'cq01_v34f_corridor_planning.rviz')
    dynamics_params = os.path.join(dynamics_share, 'config', 'dynamics.yaml')
    controller_params = os.path.join(controller_share, 'config', 'controller.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start_mission')
    start_index = LaunchConfiguration('mission_start_index')
    end_index = LaunchConfiguration('mission_end_index')
    resolution = LaunchConfiguration('resolution_m')
    inflation = LaunchConfiguration('inflation_radius_m')
    waypoint_spacing = LaunchConfiguration('maximum_waypoint_spacing_m')
    waypoint_tolerance = LaunchConfiguration('waypoint_tolerance_m')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_start_mission', default_value='false'),
        DeclareLaunchArgument('mission_start_index', default_value='0'),
        DeclareLaunchArgument('mission_end_index', default_value='15'),
        DeclareLaunchArgument('resolution_m', default_value='2.0'),
        DeclareLaunchArgument('inflation_radius_m', default_value='2.5'),
        DeclareLaunchArgument('maximum_waypoint_spacing_m', default_value='2.0'),
        DeclareLaunchArgument('waypoint_tolerance_m', default_value='0.75'),

        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='cq01_v34f_world_to_map', output='screen',
            arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0','--frame-id','world','--child-frame-id','map'],
        ),
        Node(
            package='cq01_v2_scene', executable='scene_publisher',
            name='cq01_v34f_scene_publisher', output='screen',
            parameters=[{
                'scene_file': scene_file,
                'mission_file': mission_file,
                'frame_id': 'world',
                'map_topic': '/map_generator/global_cloud',
                'publish_period_s': 10.0,
            }],
        ),
        Node(
            package='cq01_v2_scene', executable='voxel_adapter',
            name='cq01_v34f_voxel_adapter', output='screen',
            parameters=[{
                'scene_file': scene_file,
                'frame_id': 'map',
                'resolution_m': ParameterValue(resolution, value_type=float),
                'inflation_radius_m': ParameterValue(inflation, value_type=float),
                'publish_period_s': 3600.0,
                'include_visual_only_boxes': True,
            }],
        ),
        Node(
            package='drone_dynamics', executable='quadrotor_dynamics_node',
            name='quadrotor_dynamics', output='screen', parameters=[dynamics_params],
        ),
        Node(
            package='drone_controller', executable='quadrotor_controller_node',
            name='quadrotor_controller', output='screen', parameters=[controller_params],
        ),
        Node(
            package='drone_planner_3d', executable='astar_3d_planner',
            name='astar_3d_planner', output='screen',
            parameters=[{
                'minimum_flight_z_m': 1.0,
                'maximum_flight_z_m': 285.0,
                'vertical_cost_factor': 1.15,
                'waypoint_tolerance_m': ParameterValue(waypoint_tolerance, value_type=float),
                'waypoint_hold_seconds': 0.05,
                'maximum_waypoint_spacing_m': ParameterValue(waypoint_spacing, value_type=float),
                'line_of_sight_step_m': 0.30,
                'fixed_height_test_z_m': 30.0,
            }],
        ),
        Node(
            package='cq01_v2_scene', executable='planning_mission_dispatcher',
            name='cq01_v34f_corridor_mission_dispatcher', output='screen',
            parameters=[{
                'mission_file': mission_file,
                'goal_topic': '/drone/goal_raw',
                'odom_topic': '/drone/odom',
                'planner_status_topic': '/drone/planner_3d/status',
                'mission_status_topic': '/cq01_v34f/mission_status',
                'start_index': ParameterValue(start_index, value_type=int),
                'end_index': ParameterValue(end_index, value_type=int),
                'auto_start': ParameterValue(auto_start, value_type=bool),
                'goal_delay_s': 0.20,
                'stall_timeout_s': 120.0,
                'minimum_progress_m': 0.20,
                'maximum_replans_per_goal': 2,
            }],
        ),
        Node(
            package='cq01_v2_scene', executable='drone_model_visualizer',
            name='cq01_v34f_drone_model_visualizer', output='screen',
            parameters=[{
                'odom_topic': '/drone/odom',
                'marker_topic': '/cq01_v34f/drone_model',
                'trajectory_topic': '/cq01_v34f/actual_trajectory',
                'frame_id': 'world',
                'arm_half_span_m': 0.55,
                'body_length_m': 0.65,
                'body_width_m': 0.42,
                'body_height_m': 0.24,
                'rotor_diameter_m': 0.45,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='cq01_v34f_rviz',
            output='screen', condition=IfCondition(use_rviz), arguments=['-d', rviz_file],
        ),
    ])
