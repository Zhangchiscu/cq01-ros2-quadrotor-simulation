from pathlib import Path
from setuptools import find_packages, setup

package_name = "drone_chongqing_cq01"


def recursive_data_files(directory: str):
    root = Path(directory)
    output = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            destination = Path("share") / package_name / path.parent
            output.append((str(destination), [str(path)]))
    return output


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ]
    + recursive_data_files("launch")
    + recursive_data_files("rviz")
    + recursive_data_files("runtime_data"),
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description="CQ-01 Jialing foothill to Eling ridge 3D UAV mission.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cq01_map_server = drone_chongqing_cq01.map_server:main",
            "cq01_mission = drone_chongqing_cq01.mission_node:main",
            "cq01_evaluation = drone_chongqing_cq01.evaluation_node:main",
            "cq01_presentation = drone_chongqing_cq01.presentation_node:main",
            "show_latest_cq01 = drone_chongqing_cq01.show_latest:main",
        ],
    },
)
