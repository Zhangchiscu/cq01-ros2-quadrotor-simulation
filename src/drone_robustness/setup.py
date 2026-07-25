from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_robustness"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Odometry-noise robustness benchmark for "
        "quadrotor corridor avoidance."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "odom_noise_injector = "
                "drone_robustness.odom_noise_injector:main"
            ),
            (
                "run_noise_robustness = "
                "drone_robustness.runner:main"
            ),
            (
                "show_latest_robustness = "
                "drone_robustness.show_latest:main"
            ),
        ],
    },
)
