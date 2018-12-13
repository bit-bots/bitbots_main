#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hardware_info'],
    package_dir={'': 'scripts'},
    #scripts=['scripts/motion_viz']
)

setup(**d)
