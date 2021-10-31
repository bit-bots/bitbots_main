import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_vision'


setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config",
            glob.glob('config/*.yaml')),
        ('share/' + package_name + "/config/color_lookup_tables",
            glob.glob('config/color_lookup_tables/*.pickle')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
        *[('share/' + package_name + '/' + os.path.dirname(file), [file]) for file in glob.glob('models/**/**/*.*', recursive=True)],
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
            'vision = bitbots_vision.vision:main',
        ],
    }
)

#https://roboticsbackend.com/ros2-rclpy-parameter-callback/
