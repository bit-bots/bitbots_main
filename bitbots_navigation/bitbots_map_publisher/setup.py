import glob

from setuptools import find_packages, setup

package_name = "bitbots_map_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml", recursive=True)),
        ("share/" + package_name + "/config", glob.glob("config/*.png", recursive=True)),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="par",
    maintainer_email="paer.wiegmann@gmx.de",
    description="This package contains a node that publishes information about the lines of soccer fields.",
    license="MIT",
    entry_points={
        "console_scripts": ["bitbots_map_publisher = bitbots_map_publisher.bitbots_map_publisher:main"],
    },
)
