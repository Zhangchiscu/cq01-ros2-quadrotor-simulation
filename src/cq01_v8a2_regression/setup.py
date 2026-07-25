from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cq01_v8a2_regression'
setup(
    name=package_name,
    version='8.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description='CQ-01 V8-A2.1 fixes the P1-P2 low-city trigger by latching dispatcher progress across transient goal_sent/path_ready status updates.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'v8a2_low_city_disturbance = cq01_v8a2_regression.v8a2_low_city_disturbance:main',
            'v8a2_climb_h4 = cq01_v8a2_regression.v8a2_climb_h4:main',
            'v8a2_wrench_mux = cq01_v8a2_regression.v8a2_wrench_mux:main',
            'v8a2_low_gate_trigger = cq01_v8a2_regression.v8a2_low_gate_trigger:main',
            'v8a2_camera_focus = cq01_v8a2_regression.v8a2_camera_focus:main',
            'v8a2_runtime_monitor = cq01_v8a2_regression.v8a2_runtime_monitor:main',
        ],
    },
)
