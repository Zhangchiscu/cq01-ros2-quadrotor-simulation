from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_dynamic_replanning"

setup(
    name=package_name,
    version="0.1.7",
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
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "rviz",
            ),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Moving obstacles, predicted dynamic voxel maps, "
        "safe hold, online 3D replanning, and recovery."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "dynamic_obstacle = "
                "drone_dynamic_replanning.dynamic_obstacle_node:main"
            ),
            (
                "dynamic_voxel_overlay = "
                "drone_dynamic_replanning.dynamic_voxel_overlay:main"
            ),
            (
                "replanning_supervisor = "
                "drone_dynamic_replanning.replanning_supervisor:main"
            ),
            (
                "dynamic_replanning_evaluation = "
                "drone_dynamic_replanning.dynamic_evaluation:main"
            ),
            (
                "run_dynamic_replanning = "
                "drone_dynamic_replanning.runner:main"
            ),
            (
                "show_latest_dynamic_replanning = "
                "drone_dynamic_replanning.show_latest:main"
            ),
            (
                "run_dynamic_robustness = "
                "drone_dynamic_replanning.robustness_runner:main"
            ),
            (
                "show_latest_dynamic_robustness = "
                "drone_dynamic_replanning.show_latest_robustness:main"
            ),
        ],
    },
)
