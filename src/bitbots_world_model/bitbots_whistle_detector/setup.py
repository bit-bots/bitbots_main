import glob

from setuptools import setup

package_name = "bitbots_whistle_detector"


setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
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
