from glob import glob

from setuptools import find_packages, setup

package_name = "bitbots_auto_referee"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"bitbots_auto_referee.ui": ["*.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hamburg Bit-Bots",
    maintainer_email="info@bit-bots.de",
    description="Independent automatic referee infrastructure for the MuJoCo soccer simulation.",
    license="MIT",
    entry_points={"console_scripts": ["auto_referee = bitbots_auto_referee.node:main"]},
)
