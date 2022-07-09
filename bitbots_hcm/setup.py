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

        ('share/' + package_name + '/config',
            glob.glob('config/*')),

        ('share/' + package_name + '/actions',
            glob.glob(package_name + '/hcm_dsd/actions/*.py')),
        ('share/' + package_name + '/decisions' ,
            glob.glob(package_name + '/hcm_dsd/decisions/*.py')),
        ('share/' + package_name,
            [package_name+'/hcm_dsd/hcm.dsd'])
    ],
    scripts=[
        'scripts/hcm_led.py',
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    include_package_data=True,
    zip_safe=True,
    keywords=['ROS'],
    license='MIT',
)
