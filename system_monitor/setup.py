import glob

from setuptools import setup, find_packages

package_name = 'system_monitor'

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ('share/' + package_name + "/config",
        glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.launch')),
    ],
    install_requires=[
        'setuptools',
    ],
    entry_points={
        'console_scripts': [
            'monitor = system_monitor.monitor:main',
        ],
    }
)