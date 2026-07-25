from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_trajectory"

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
        "Collision-aware 3D path smoothing, dynamic time "
        "parameterization and raw-versus-smoothed comparison."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "trajectory_executor = "
                "drone_trajectory.trajectory_node:main"
            ),
            (
                "trajectory_evaluation = "
                "drone_trajectory.evaluation_node:main"
            ),
            (
                "run_trajectory_comparison = "
                "drone_trajectory.runner:main"
            ),
            (
                "show_latest_trajectory_comparison = "
                "drone_trajectory.show_latest:main"
            ),
        ],
    },
)
