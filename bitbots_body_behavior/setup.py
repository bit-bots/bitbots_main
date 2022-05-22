import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_body_behavior'


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
        'scripts/backup_behavior.py',
        'scripts/fake_plan.py',
        'scripts/show_world_model_objects.py'
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
            'body_behavior = bitbots_body_behavior.body_behavior:main',
        ],
    }
)
