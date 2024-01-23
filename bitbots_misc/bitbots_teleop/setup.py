import glob

from setuptools import find_packages, setup

package_name = "bitbots_teleop"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    scripts=["scripts/teleop_keyboard.py"],
    install_requires=[
        "launch",
        "setuptools",
    ],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
    entry_points={
        "console_scripts": [
            "joy_node = bitbots_teleop.joy_node:main",
        ],
    },
)
