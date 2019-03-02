from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['bitbots_bezier_pathfinding'],
        package_dir={'': 'src'},
)

setup(**d)
