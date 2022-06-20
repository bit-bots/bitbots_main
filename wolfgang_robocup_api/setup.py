import glob
import os.path

from setuptools import setup
from setuptools import find_packages

package_name = 'wolfgang_robocup_api'

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/config.yaml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch')),
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
            'wolfgang_robocup_api = wolfgang_robocup_api.wolfgang_robocup_api:main',
        ],
    },
)
