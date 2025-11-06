import glob
import os

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

generate_parameter_module(
    "vision_parameters",  # python module name for parameter library
    "config/vision_parameters.yaml",  # path to input yaml file
)

package_name = "bitbots_vision"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
        *[
            ("share/" + package_name + "/" + os.path.dirname(file), [file])
            for file in glob.glob("models/**/**/*.*", recursive=True)
        ],
    ],
    install_requires=[
        "launch",
        "setuptools",
    ],
    tests_require=["pytest"],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision = bitbots_vision.vision:main",
        ],
    },
)
