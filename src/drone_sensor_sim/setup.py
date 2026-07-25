from glob import glob
import os

from setuptools import find_packages, setup

package_name = "drone_sensor_sim"

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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Deterministic odometry and IMU sensor model with automated "
        "quadrotor robustness evaluation."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_model = drone_sensor_sim.sensor_model:main",
            "sensor_experiment = drone_sensor_sim.sensor_experiment:main",
            "show_latest_sensor = drone_sensor_sim.show_latest:main",
            "run_sensor_comparison = drone_sensor_sim.run_comparison:main",
        ],
    },
)
