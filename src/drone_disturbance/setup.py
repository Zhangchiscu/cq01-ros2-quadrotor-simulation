from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_disturbance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Configurable wind/external-wrench profiles and automated "
        "closed-loop recovery evaluation for the ROS2 quadrotor simulator."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "disturbance_injector = "
                "drone_disturbance.disturbance_injector:main"
            ),
            (
                "disturbance_experiment = "
                "drone_disturbance.disturbance_experiment:main"
            ),
            (
                "show_latest_disturbance = "
                "drone_disturbance.show_latest:main"
            ),
        ],
    },
)
