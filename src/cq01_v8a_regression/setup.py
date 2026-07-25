from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cq01_v8a_regression'

setup(
    name=package_name,
    version='8.0.1',
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
        'CQ-01 V8-A isolated regression using the audited original '
        'P0-P4 dispatcher chain and the byte-identical V3.4H4 '
        '2.5 N x 1.0 s external-disturbance recovery module.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            (
                'h4_integrated_external_disturbance = '
                'cq01_v8a_regression.h4_integrated_external_disturbance:main'
            ),
            (
                'v8a_runtime_monitor = '
                'cq01_v8a_regression.v8a_runtime_monitor:main'
            ),
        ],
    },
)
