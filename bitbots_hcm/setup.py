import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_hcm'


setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    scripts=[
        'scripts/hcm_led.py',
        'scripts/MakeGraph.py',
        'scripts/send_motor_traj.py',
        'scripts/test_publisher.py',
        'scripts/test_subscriber.py'
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
            'hcm_node = bitbots_hcm.humanoid_control_module:main',
            'pause_node = bitbots_hcm.pause:main',
        ],
    }
)
