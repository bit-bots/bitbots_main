from setuptools import find_packages, setup

package_name = "dynamic_stack_decider"

setup(
    name=package_name,
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    tests_require=["pytest"],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
    entry_points={},
)
