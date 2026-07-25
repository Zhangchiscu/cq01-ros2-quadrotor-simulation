from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_world_validation"

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
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Safe map inflation and multi-scenario regression "
        "for drone_world navigation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "occupancy_inflator = "
                "drone_world_validation.occupancy_inflator:main"
            ),
            (
                "scenario_mission = "
                "drone_world_validation.scenario_mission:main"
            ),
            (
                "run_world_regression = "
                "drone_world_validation.runner:main"
            ),
            (
                "show_latest_world_regression = "
                "drone_world_validation.show_latest:main"
            ),
        ],
    },
)
