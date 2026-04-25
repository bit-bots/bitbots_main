import glob

from setuptools import find_packages, setup

package_name = "bitbots_rl_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/models",
            glob.glob("models/*.onnx"),
        ),
        ("share/" + package_name + "/configs", glob.glob("configs/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BitBots",
    maintainer_email="git@bit-bots.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["walk_node = nodes.walk_node:main", "kick_node = nodes.kick_node:main"],
    },
)
