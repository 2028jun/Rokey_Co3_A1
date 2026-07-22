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
    description="Nav2 NavigateToPose kitchen/table missions over the Occupancy Map.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "go_to_table = nav_robot_missions.go_to_table:main",
            "return_to_kitchen = nav_robot_missions.return_to_kitchen:main",
            "round_trip = nav_robot_missions.round_trip:main",
            "rail_patrol = nav_robot_missions.rail_patrol:main",
            "slam_patrol = nav_robot_missions.slam_patrol:main",
            "nova_go_to_pose = nav_robot_missions.nova_go_to_pose:main",
            "nova_through_poses = nav_robot_missions.nova_through_poses:main",
            "nova_carter_topic_bridge = nav_robot_missions.nova_carter_topic_bridge:main",
        ],
    },
)
