import glob

from setuptools import find_packages, setup

package_name = "bitbots_localization_handler"


setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/localization_dsd/actions",
            glob.glob(package_name + "/localization_dsd/actions/*.py"),
        ),
        (
            "share/" + package_name + "/localization_dsd/decisions",
            glob.glob(package_name + "/localization_dsd/decisions/*.py"),
        ),
        ("share/" + package_name + "/localization_dsd", [package_name + "/localization_dsd/localization.dsd"]),
    ],
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
            "localization_handler = bitbots_localization_handler.localization_handler:main",
        ],
    },
)
