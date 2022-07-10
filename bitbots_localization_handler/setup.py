import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'bitbots_localization_handler'


setup(name=package_name,
      packages=find_packages(exclude=['test']),
      data_files=[('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/actions', glob.glob(package_name + '/localization_dsd/actions/*.py')),
                  ('share/' + package_name + '/decisions',
                   glob.glob(package_name + '/localization_dsd/decisions/*.py')),
                  ('share/' + package_name, [package_name + '/localization_dsd/localization.dsd'])],
      install_requires=[
          'launch',
          'setuptools',
      ],
      zip_safe=True,
      keywords=['ROS'],
      license='MIT',
      entry_points={
          'console_scripts': ['localization_handler = bitbots_localization_handler.localization_handler:main',],
      })
