import glob

from setuptools import setup

package_name = "bitbots_body_behavior"


setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/config", glob.glob("config/*")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    scripts=[],
    install_requires=[
        "launch",
        "setuptools",
    ],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "body_behavior = bitbots_body_behavior.body_behavior:main",
        ],
    },
)
