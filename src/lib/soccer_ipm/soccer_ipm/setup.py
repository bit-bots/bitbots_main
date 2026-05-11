import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'soccer_ipm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            glob.glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Vahl',
    maintainer_email='florian@flova.de',
    description='Inverse perspective mapping for the RoboCup soccer domain',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipm = soccer_ipm.soccer_ipm:main',
        ],
    }
)
