from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_absolute_move'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='Absolute Move behavior for TurtleBot3 on ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'absolute_move_node = turtlebot3_absolute_move.absolute_move_node:main',
            'absolute_move_client = turtlebot3_absolute_move.absolute_move_client:main',
            'pipeline = turtlebot3_absolute_move.pipeline_orchestrator:main',
            'goal_input = turtlebot3_absolute_move.goal_input:main',
            'scan_relay = turtlebot3_absolute_move.scan_relay:main',
        ],
    },
)
