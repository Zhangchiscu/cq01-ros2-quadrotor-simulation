from glob import glob
from setuptools import find_packages, setup

package_name = "drone_chongqing_realdata"

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
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/web",
            glob("web/*.*"),
        ),
        (
            "share/" + package_name + "/web/data",
            glob("web/data/*.*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xianzhe Zhang",
    maintainer_email="taccettabellus@gmail.com",
    description=(
        "Real-data Yuzhong Peninsula 3D validation viewer and "
        "OpenStreetMap fetch pipeline."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "fetch_yuzhong_realdata = "
                "drone_chongqing_realdata.fetch_real_data:main"
            ),
            (
                "serve_yuzhong_realdata = "
                "drone_chongqing_realdata.serve_scene:main"
            ),
            (
                "show_yuzhong_realdata_manifest = "
                "drone_chongqing_realdata.show_manifest:main"
            ),
        ],
    },
)
