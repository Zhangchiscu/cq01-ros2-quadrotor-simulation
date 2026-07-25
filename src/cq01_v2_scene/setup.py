from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cq01_v2_scene'

setup(
    name=package_name,
    version='7.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description='CQ-01 V7.1.2 keeps the original dispatcher alive and inactive during the prelude, then starts it through its real service to prevent a stale transient-local final_waypoint_reached event from falsely completing P0.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scene_publisher = cq01_v2_scene.scene_publisher:main',
            'mission_dispatcher = cq01_v2_scene.mission_dispatcher:main',
            'voxel_adapter = cq01_v2_scene.voxel_adapter:main',
            'planning_mission_dispatcher = cq01_v2_scene.planning_mission_dispatcher:main',
            'drone_model_visualizer = cq01_v2_scene.drone_model_visualizer:main',
            'fast_path_follower = cq01_v2_scene.fast_path_follower:main',
            'fast_mission_dispatcher = cq01_v2_scene.fast_mission_dispatcher:main',
            'complete_path_follower = cq01_v2_scene.complete_path_follower:main',
            'complete_mission_dispatcher = cq01_v2_scene.complete_mission_dispatcher:main',
            'mission_timer = cq01_v2_scene.mission_timer:main',
            'integrated_disturbance_manager = cq01_v2_scene.integrated_disturbance_manager:main',
            'v71_prelude_runner = cq01_v2_scene.v71_prelude_runner:main',
            'v71_h10_arm_once = cq01_v2_scene.v71_h10_arm_once:main',
            'v712_start_route_and_h10 = cq01_v2_scene.v712_start_route_and_h10:main',
            'v71_unified_camera_focus_manager = cq01_v2_scene.v71_unified_camera_focus_manager:main',
            'v71_marker_geometry_filter = cq01_v2_scene.v71_marker_geometry_filter:main',
            'v71_short_regression_evaluator = cq01_v2_scene.v71_short_regression_evaluator:main',
        ],
    },
)
