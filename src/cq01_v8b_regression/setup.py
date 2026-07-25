from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cq01_v8b_regression'

setup(
    name=package_name,
    version='8.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description=(
        'CQ-01 V8-B high-altitude H14 functional replan plus '
        'P3.1 summit rapid ascent, circle and figure-eight regression.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            (
                'v8b_high_altitude_wind = '
                'cq01_v8b_regression.v8b_high_altitude_wind:main'
            ),
            (
                'v8b_summit_pattern_manager = '
                'cq01_v8b_regression.v8b_summit_pattern_manager:main'
            ),
            (
                'v8b_pattern_tracking_evaluator = '
                'cq01_v8b_regression.v8b_pattern_tracking_evaluator:main'
            ),
            (
                'v8b_goal_mux = '
                'cq01_v8b_regression.v8b_goal_mux:main'
            ),
            (
                'v8b_summit_orchestrator = '
                'cq01_v8b_regression.v8b_summit_orchestrator:main'
            ),
            (
                'v8b_raw_goal_mux = '
                'cq01_v8b_regression.v8b_raw_goal_mux:main'
            ),
            (
                'v8b_camera_focus = '
                'cq01_v8b_regression.v8b_camera_focus:main'
            ),
            (
                'v8b_marker_text_filter = '
                'cq01_v8b_regression.v8b_marker_text_filter:main'
            ),
            (
                'v8b_runtime_monitor = '
                'cq01_v8b_regression.v8b_runtime_monitor:main'
            ),
        ],
    },
)
