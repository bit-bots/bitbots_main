import glob

from setuptools import find_packages, setup

package_name = "bitbots_ccc"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=[
        "setuptools",
    ],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sip_animator = bitbots_ccc.sip_animator:main",
        ],
    },
)
