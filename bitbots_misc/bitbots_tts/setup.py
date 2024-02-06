import glob

from setuptools import find_packages, setup

package_name = "humanoid_league_speaker"

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=[
        "setuptools",
    ],
    scripts=["scripts/send_text.py"],
    entry_points={
        "console_scripts": [
            "speaker = humanoid_league_speaker.speaker:main",
        ],
    },
)
