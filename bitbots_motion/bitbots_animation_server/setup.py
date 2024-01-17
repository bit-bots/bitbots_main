import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_animation_server'


setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    scripts=[
        'scripts/animation_hcm_bridge.py',
        'scripts/run_animation.py'
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    zip_safe=True,
    keywords=['ROS'],
    license='MIT',
    entry_points={
        'console_scripts': [
            'animation_node = bitbots_animation_server.animation_node:main',
        ],
    }
)
