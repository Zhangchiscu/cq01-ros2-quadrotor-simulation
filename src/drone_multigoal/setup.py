from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_multigoal"

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
                "config",
            ),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Continuous multi-goal A-star avoidance mission "
        "and automated evaluation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "multi_goal_evaluation = "
                "drone_multigoal.multi_goal_evaluation:main"
            ),
            (
                "run_multi_goal_evaluation = "
                "drone_multigoal.runner:main"
            ),
            (
                "show_latest_multi_goal_result = "
                "drone_multigoal.show_latest:main"
            ),
        ],
    },
)
