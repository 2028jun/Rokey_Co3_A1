from setuptools import find_packages, setup


package_name = 'm0609_isaac_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@example.com',
    description='Isaac Sim trajectory bridge for MoveIt 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_bridge = m0609_isaac_control.trajectory_bridge:main',
            'moveit_joint_test = m0609_isaac_control.moveit_joint_test:main',
            'demo_obstacle = m0609_isaac_control.demo_obstacle:main',
        ],
    },
)
