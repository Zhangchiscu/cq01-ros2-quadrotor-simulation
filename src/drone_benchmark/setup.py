from setuptools import find_packages, setup

package_name = "drone_benchmark"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Repeatability benchmarks for the ROS2 "
        "quadrotor simulator."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "run_standard_repeatability = "
                "drone_benchmark.repeatability:main"
            ),
            (
                "show_latest_repeatability = "
                "drone_benchmark.show_latest:main"
            ),
        ],
    },
)
