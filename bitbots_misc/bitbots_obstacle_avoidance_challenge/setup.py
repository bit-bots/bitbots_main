import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

package_name = "bitbots_obstacle_avoidance_challenge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="par",
    maintainer_email="paer-wiegmann@gmx.de",
    description="This Package provides a simple vision to detect the obstacles for the obstacle avoidance challenge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bitbots_obstacle_avoidance_challenge = bitbots_obstacle_avoidance_challenge.bitbots_obstacle_avoidance_challenge:main",
            "navigate_to_other_side = bitbots_obstacle_avoidance_challenge.navigate_to_other_side:main",
        ],
    },
)

generate_parameter_module(
    "bitbots_obstacle_avoidance_challenge_params",
    "config/range.yaml",
)