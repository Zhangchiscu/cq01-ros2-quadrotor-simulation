from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cq01_v8a3_regression'

setup(
    name=package_name,
    version='8.3.0',
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
        'CQ-01 V8-A3 adaptive 8-10 m low-city strong-wind regression '
        'with closed-loop deviation stopping and a frozen P3-P4 H4 pulse.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            (
                'v8a3_low_city_strong_wind = '
                'cq01_v8a3_regression.v8a3_low_city_strong_wind:main'
            ),
            (
                'v8a3_climb_h4 = '
                'cq01_v8a3_regression.v8a3_climb_h4:main'
            ),
            (
                'v8a3_wrench_mux = '
                'cq01_v8a3_regression.v8a3_wrench_mux:main'
            ),
            (
                'v8a3_camera_focus = '
                'cq01_v8a3_regression.v8a3_camera_focus:main'
            ),
            (
                'v8a3_runtime_monitor = '
                'cq01_v8a3_regression.v8a3_runtime_monitor:main'
            ),
            (
                'v8a3_marker_text_filter = '
                'cq01_v8a3_regression.v8a3_marker_text_filter:main'
            ),
        ],
    },
)
