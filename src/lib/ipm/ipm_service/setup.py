from setuptools import find_packages
from setuptools import setup

package_name = 'ipm_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Florian Vahl',
    maintainer_email='git@flova.de',
    description='A ROS2 Inverse Perspective Mapping Service',
    keywords=['ROS'],
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipm_service = ipm_service.ipm:main',
        ],
    }
)
