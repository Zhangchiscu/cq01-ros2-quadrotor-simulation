from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(
        exclude=['test'],
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join(
                'share',
                package_name,
                'rviz',
            ),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description=(
        'Launch and RViz2 visualization package for '
        'the ROS2 quadrotor simulator.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'drone_visualization = '
                'drone_bringup.visualization_node:main'
            ),
            (
                'default_goal = '
                'drone_bringup.default_goal:main'
            ),
        ],
    },
)
