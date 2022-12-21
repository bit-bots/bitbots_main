import glob

from setuptools import setup

package_name = 'bitbots_robot_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config",
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='florian',
    maintainer_email='git@flova.de',
    description='A simple model that keeps track of the robots on the field.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter = bitbots_robot_filter.filter:main',
        ],
    },
)
