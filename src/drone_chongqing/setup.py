from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'drone_chongqing'

setup(
    name=package_name,
    version='0.3.0',
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
    description='Closed solid Yuzhong Peninsula terrain with an asymmetric shoreline, dense skyline and strong mountain-city relief.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_server = drone_chongqing.scene_server:main',
            'show_scene_manifest = drone_chongqing.show_scene_manifest:main',
        ],
    },
)
