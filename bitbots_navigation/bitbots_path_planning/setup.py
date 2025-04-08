import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import setup

generate_parameter_module(
    "path_planning_parameters",  # python module name for parameter library
    "config/path_planning_parameters.yaml",  # path to input yaml file
)

package_name = "bitbots_path_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    extras_require={
        "dev": ["pytest", "syrupy"],
    },
    zip_safe=True,
    maintainer="Florian Vahl",
    maintainer_email="git@flova.de",
    description="A minimal path plannig system",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_planning = bitbots_path_planning.path_planning:main",
        ],
    },
)
