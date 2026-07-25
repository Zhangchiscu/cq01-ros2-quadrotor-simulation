from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_ground_station_2d'
setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhang Xianzhe',
    maintainer_email='taccettabellus@gmail.com',
    description='Advanced native 2D ROS 2 quadrotor mission control station',
    license='Apache-2.0',
    entry_points={'console_scripts': ['ground_station_2d = drone_ground_station_2d.main:main']},
)
