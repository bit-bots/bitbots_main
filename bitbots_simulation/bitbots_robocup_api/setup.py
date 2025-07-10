import glob
import os.path

from setuptools import find_packages, setup

package_name = "bitbots_robocup_api"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), ["config/devices.json"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch")),
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
            f"command_proxy = {package_name}.command_proxy:main",
            f"foot_pressure_proxy = {package_name}.foot_pressure_proxy:main",
        ],
    },
)
