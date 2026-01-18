import glob

from setuptools import find_packages, setup

package_name = "bitbots_auto_test"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch") + glob.glob("launch/*.launch.py")),
    ],
    scripts=["scripts/auto_test_script.py"],
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
            "auto_test = bitbots_auto_test.auto_test:main",
        ],
    },
)
