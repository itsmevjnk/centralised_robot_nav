from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'central_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*_config.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsmevjnk',
    maintainer_email='ngtv0404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_node = central_nav.central_node:main',
            'path_pub_node = central_nav.path_pub_node:main',
            'path_erase_node = central_nav.path_erase_node:main',
            'path_filter_node = central_nav.path_filter_node:main',
            'path_marker_node = central_nav.path_marker_node:main',
            'robot_marker_node = central_nav.robot_marker_node:main',
            'cmdvel_telemetry_node = central_nav.cmdvel_telemetry_node:main',
            'pose_telemetry_node = central_nav.pose_telemetry_node:main',
            'state_telemetry_node = central_nav.state_telemetry_node:main'
        ],
    },
)
