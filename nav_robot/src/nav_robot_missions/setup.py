from setuptools import setup
import os
from glob import glob

package_name = "nav_robot_missions"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yngkim",
    maintainer_email="youngkim99@kakao.com",
    description="Kitchen-to-table fixed-rail FollowPath mission clients for nav_robot.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "return_to_kitchen = nav_robot_missions.return_to_kitchen:main",
            "nav_server_node = nav_robot_missions.nav_server_node:main",
            "hybrid_nav_server_node = nav_robot_missions.hybrid_nav_server_node:main",
            "direct_nav_server_node = nav_robot_missions.direct_nav_server_node:main",
        ],
    },
)
