from glob import glob

from setuptools import find_packages, setup

package_name = "bitbots_player_state"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hamburg Bit-Bots",
    maintainer_email="info@bit-bots.de",
    description="Aggregates robot state for the RoboCup GameController return message.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "player_state_aggregator = bitbots_player_state.player_state_aggregator:main",
        ],
    },
)
