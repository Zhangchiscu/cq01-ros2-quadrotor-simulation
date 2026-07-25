from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drone_analysis'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description=(
        'Automated experiment recording, metrics and plots '
        'for the ROS2 quadrotor simulator.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'single_goal_experiment = '
                'drone_analysis.single_goal_experiment:main'
            ),
            (
                'show_latest_result = '
                'drone_analysis.show_latest_result:main'
            ),
        ],
    },
)
