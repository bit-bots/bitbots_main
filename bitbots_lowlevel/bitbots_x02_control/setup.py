from setuptools import find_packages, setup

package_name = "bitbots_x02_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jasper",
    maintainer_email="jasper.gueldenstein@uni-hamburg.de",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "x02_control = " + package_name + ".x02_control:main",
        ],
    },
)
