import os
from glob import glob

from setuptools import find_packages, setup

package_name = "map_gen"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="youngkim99@kakao.com",
    description="One-shot slam_toolbox mapping reusing nav_restaurant_demo.py's /scan, /odom, TF",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_patrol = map_gen.slam_patrol:main",
        ],
    },
)
