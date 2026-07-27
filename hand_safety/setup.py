from setuptools import find_packages, setup


package_name = "hand_safety"

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
        (
            "share/" + package_name + "/config",
            ["config/hand_safety.yaml"],
        ),
        (
            "share/" + package_name + "/models",
            ["models/YOLOv10n_hands.pt"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="workspace user",
    maintainer_email="user@example.com",
    description="ROS 2 YOLO hand detection from an RGB image topic.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hand_detector_node = hand_safety.hand_detector_node:main",
            "jpeg_compressor_node = hand_safety.jpeg_compressor_node:main",
        ],
    },
)
