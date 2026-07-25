import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamics_share = get_package_share_directory('drone_dynamics')
    controller_share = get_package_share_directory('drone_controller')
    cq_share = FindPackageShare('cq01_v2_scene')

    scene_file = PathJoinSubstitution([
        cq_share,
        'config',
        'cq01_v34_scene.yaml',
    ])
    mission_file = PathJoinSubstitution([
        cq_share,
        'config',
        'cq01_v34a_planning_mission.yaml',
    ])
    rviz_file = PathJoinSubstitution([
        cq_share,
        'rviz',
        'cq01_v34a_planning.rviz',
    ])

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

    resolution = LaunchConfiguration('resolution_m')
    inflation = LaunchConfiguration('inflation_radius_m')
    use_rviz = LaunchConfiguration('use_rviz')
    use_trajectory = LaunchConfiguration('use_trajectory_executor')
    auto_start = LaunchConfiguration('auto_start_mission')
    start_index = LaunchConfiguration('mission_start_index')
    end_index = LaunchConfiguration('mission_end_index')

    planner_parameters = [{
        'minimum_flight_z_m': 2.0,
        'maximum_flight_z_m': 285.0,
        'vertical_cost_factor': 1.25,
        'waypoint_tolerance_m': 1.2,
        'waypoint_hold_seconds': 0.15,
        'maximum_waypoint_spacing_m': 3.0,
        'line_of_sight_step_m': 1.5,
        'fixed_height_test_z_m': 30.0,
    }]

    return LaunchDescription([
        DeclareLaunchArgument('resolution_m', default_value='3.0'),
        DeclareLaunchArgument('inflation_radius_m', default_value='1.4'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_trajectory_executor',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'auto_start_mission',
            default_value='false',
        ),
        DeclareLaunchArgument('mission_start_index', default_value='0'),
        DeclareLaunchArgument('mission_end_index', default_value='0'),

        # V3.4 visualization uses world; legacy dynamics/planner use map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cq01_v34a_world_to_map',
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
                'publish_period_s': 2.0,
            }],
        ),

        Node(
            package='cq01_v2_scene',
            executable='voxel_adapter',
            name='cq01_v34a_voxel_adapter',
            output='screen',
            parameters=[{
                'scene_file': scene_file,
                'frame_id': 'map',
                'resolution_m': ParameterValue(
                    resolution,
                    value_type=float,
                ),
                'inflation_radius_m': ParameterValue(
                    inflation,
                    value_type=float,
                ),
                'publish_period_s': 5.0,
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

        # Default: planner performs direct waypoint execution.
        Node(
            package='drone_planner_3d',
            executable='astar_3d_planner',
            name='astar_3d_planner',
            output='screen',
            condition=UnlessCondition(use_trajectory),
            parameters=planner_parameters,
        ),

        # Optional smoothing mode: remap planner's internal control output so
        # trajectory_executor is the only /drone/goal publisher.
        Node(
            package='drone_planner_3d',
            executable='astar_3d_planner',
            name='astar_3d_planner',
            output='screen',
            condition=IfCondition(use_trajectory),
            parameters=planner_parameters,
            remappings=[
                ('/drone/goal', '/drone/planner_direct_goal_disabled'),
            ],
        ),

        Node(
            package='drone_trajectory',
            executable='trajectory_executor',
            name='trajectory_executor',
            output='screen',
            condition=IfCondition(use_trajectory),
            parameters=[{
                'mode': 'smooth',
                'control_rate_hz': 20.0,
                'sample_spacing_m': 1.0,
                'maximum_speed_mps': 4.0,
                'maximum_acceleration_mps2': 2.5,
                'maximum_lateral_acceleration_mps2': 2.0,
                'maximum_vertical_speed_mps': 2.0,
                'maximum_jerk_mps3': 3.5,
                'minimum_trajectory_duration_s': 8.0,
                'time_scaling_safety_factor': 1.12,
                'smoothing_iterations': 2,
                'collision_check_spacing_m': 1.0,
                'start_delay_s': 0.5,
                'goal_hold_seconds': 1.0,
            }],
        ),

        Node(
            package='drone_bringup',
            executable='drone_visualization',
            name='drone_visualization',
            output='screen',
        ),

        Node(
            package='cq01_v2_scene',
            executable='planning_mission_dispatcher',
            name='cq01_v34a_planning_mission_dispatcher',
            output='screen',
            parameters=[{
                'mission_file': mission_file,
                'goal_topic': '/drone/goal_raw',
                'odom_topic': '/drone/odom',
                'planner_status_topic': '/drone/planner_3d/status',
                'start_index': ParameterValue(
                    start_index,
                    value_type=int,
                ),
                'end_index': ParameterValue(
                    end_index,
                    value_type=int,
                ),
                'auto_start': ParameterValue(
                    auto_start,
                    value_type=bool,
                ),
                'goal_delay_s': 0.5,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='cq01_v34a_rviz',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_file],
        ),
    ])
