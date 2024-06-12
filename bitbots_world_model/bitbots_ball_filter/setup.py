import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

generate_parameter_module(
    "ball_filter_parameters",  # python module name for parameter library
    "config/ball_filter_parameters.yaml",  # path to input yaml file
)

package_name = "bitbots_ball_filter"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=[
        "launch",
        "setuptools",
    ],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
    entry_points={
        "console_scripts": [
            "ball_filter = bitbots_ball_filter.ball_filter:main",
            "ball_sim = bitbots_ball_filter.ball_sim:main",
        ],
    },
)

# https://roboticsbackend.com/ros2-rclpy-parameter-callback/
