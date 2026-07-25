from pathlib import Path
from setuptools import find_packages, setup

package_name = "drone_chongqing_fullscene"


def recursive_data_files(directory: str):
    root = Path(directory)
    entries = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            destination = Path("share") / package_name / path.parent
            entries.append((str(destination), [str(path)]))
    return entries


setup(
    name=package_name,
    version="0.2.0",
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
    description=(
        "Full Yuzhong peninsula 3D overview for the Chongqing UAV "
        "navigation benchmark."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "full_scene_server = "
                "drone_chongqing_fullscene.full_scene_server:main"
            ),
            (
                "show_full_scene = "
                "drone_chongqing_fullscene.show_full_scene:main"
            ),
        ],
    },
)
