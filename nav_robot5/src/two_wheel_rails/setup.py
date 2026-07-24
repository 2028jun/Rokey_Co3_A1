import os
from glob import glob

from setuptools import find_packages, setup

package_name = "two_wheel_rails"

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
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (
            os.path.join("share", package_name, "maps", "restaurant"),
            glob("maps/restaurant/*"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="youngkim99@kakao.com",
    description="Rail-following Nav2 missions for nav_robot5",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rail_mission = two_wheel_rails.rail_mission:main",
            "autonomous_mission = two_wheel_rails.autonomous_mission:main",
            "topic_bridge = two_wheel_rails.topic_bridge:main",
            "navigation_subsystem = two_wheel_rails.navigation_subsystem_node:main",
            "navigation_initialize = two_wheel_rails.navigation_initialize:main",
        ],
    },
)
