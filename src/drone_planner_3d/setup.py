from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_planner_3d"

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
        "26-neighbour 3D A* planning and automated "
        "overflight/underpass evaluation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "astar_3d_planner = "
                "drone_planner_3d.planner_node:main"
            ),
            (
                "planner_3d_evaluation = "
                "drone_planner_3d.evaluation_node:main"
            ),
            (
                "run_3d_planner_benchmark = "
                "drone_planner_3d.runner:main"
            ),
            (
                "show_latest_3d_planner = "
                "drone_planner_3d.show_latest:main"
            ),
        ],
    },
)
