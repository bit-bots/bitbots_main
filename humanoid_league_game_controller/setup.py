import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'humanoid_league_game_controller'

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config",
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    install_requires=[
        'launch',
        'setuptools',
        'construct',
    ],
    zip_safe=True,
    keywords=['ROS'],
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_controller = humanoid_league_game_controller.receiver:main',
        ],
    }
)
