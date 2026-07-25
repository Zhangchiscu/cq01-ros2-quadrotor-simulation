from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_voxel_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Filled and inflated 3D voxel occupancy map "
        "for the ROS2 quadrotor simulator."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "voxel_map_server = "
                "drone_voxel_map.voxel_server:main"
            ),
            (
                "inspect_voxel_map = "
                "drone_voxel_map.inspect_node:main"
            ),
            (
                "validate_voxel_scenarios = "
                "drone_voxel_map.validate_scenarios:main"
            ),
        ],
    },
)
