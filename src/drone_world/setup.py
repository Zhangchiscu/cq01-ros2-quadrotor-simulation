from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'drone_world'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Xianzhe Zhang',
    maintainer_email='taccettabellus@gmail.com',
    description='Configurable multi-scenario 3D obstacle world server.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'world_server = drone_world.world_node:main',
            'list_scenarios = drone_world.list_scenarios:main',
        ],
    },
)
