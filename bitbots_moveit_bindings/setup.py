from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bitbots_moveit_bindings'],
    package_dir={'': 'src'},
)

setup(**d)
