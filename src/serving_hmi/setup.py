import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'serving_hmi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web_ui'), glob('web_ui/*.*')),
        (os.path.join('share', package_name, 'web_ui/css'), glob('web_ui/css/*.*')),
        (os.path.join('share', package_name, 'web_ui/js'), glob('web_ui/js/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='HMI Dashboard for Pizza Serving Robot and Isaac Sim Monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi_backend_node = serving_hmi.hmi_backend_node:main',
        ],
    },
)
