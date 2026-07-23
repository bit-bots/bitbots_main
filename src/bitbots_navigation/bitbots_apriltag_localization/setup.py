import glob

from setuptools import find_packages, setup

package_name = "bitbots_apriltag_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hamburg Bit-Bots",
    maintainer_email="git@bit-bots.de",
    description="Localizes the robot by detecting AprilTags (and rigid AprilTag bundles) "
    "placed at known poses in the environment.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"localization_node = {package_name}.localization_node:main",
        ],
    },
)
