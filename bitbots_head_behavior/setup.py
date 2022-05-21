import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_head_behavior'


setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config",
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    scripts=[
        'scripts/test_look_at.py',
        'scripts/testHeadBehavior.py'
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
            'head_node = bitbots_head_behavior.head_node:main',
        ],
    }
)
