from glob import glob
from setuptools import find_packages, setup

package_name = "cq01_requirement_closure"

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
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CQ01 Student",
    maintainer_email="student@example.com",
    description=(
        "Independent add-ons for teacher requirement closure: "
        "configurable sensor noise and local perception-range simulation."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "sensor_noise_bridge = "
                "cq01_requirement_closure.sensor_noise_bridge:main"
            ),
            (
                "local_perception_filter = "
                "cq01_requirement_closure.local_perception_filter:main"
            ),
        ],
    },
)
