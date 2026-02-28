import glob

from setuptools import find_packages, setup



package_name = "bitbots_whistle_detector"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "whistle_detector = bitbots_whistle_detector.whistle_detector:main",
        ],
    },
)

# https://roboticsbackend.com/ros2-rclpy-parameter-callback/
