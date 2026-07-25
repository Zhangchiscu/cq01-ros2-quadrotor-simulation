from glob import glob
from setuptools import find_packages, setup

package_name = "drone_pattern_trajectory"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhang Xianzhe",
    maintainer_email="taccettabellus@gmail.com",
    description="Circle and figure-eight quadrotor tracking experiments with automatic evaluation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pattern_executor = drone_pattern_trajectory.pattern_executor:main",
            "pattern_evaluation = drone_pattern_trajectory.pattern_evaluation:main",
            "run_pattern_suite = drone_pattern_trajectory.runner:main",
            "show_latest_pattern = drone_pattern_trajectory.show_latest:main",
        ],
    },
)
