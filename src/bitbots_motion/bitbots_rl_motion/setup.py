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
    maintainer="Hamburg Bit-Bots",
    maintainer_email="git@bit-bots.de",
    description="The bitbots_rl_motion package provides different reinforcement based motions like walking and standing up.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"walk_node = {package_name}.nodes.walk_node:main",
            f"kick_node = {package_name}.nodes.kick_node:main",
            f"mjlab_walk_node = {package_name}.nodes.mjlab_walk_node:main",
            f"mjlab_getup_node = {package_name}.nodes.mjlab_getup_node:main",
            f"phase_from_transform = {package_name}.nodes.phase_from_transform:main",
        ],
    },
)
