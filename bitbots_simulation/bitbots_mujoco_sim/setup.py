import glob

from setuptools import find_packages, setup

package_name = "bitbots_mujoco_sim"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/xml", glob.glob("xml/*.xml")),
        ("share/" + package_name + "/xml/assets", glob.glob("xml/assets/*.png")),
        ("share/" + package_name + "/xml/assets", glob.glob("xml/assets/*.stl")),
        ("share/" + package_name + "/xml/assets/ball", glob.glob("xml/assets/ball/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="22maschal",
    maintainer_email="22maschal@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["sim = bitbots_mujoco_sim.main:main"],
    },
)
