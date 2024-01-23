import glob

from setuptools import find_packages, setup

package_name = "system_monitor"

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=[
        "setuptools",
    ],
    entry_points={
        "console_scripts": [
            "monitor = system_monitor.monitor:main",
        ],
    },
)
