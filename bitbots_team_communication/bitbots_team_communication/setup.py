from setuptools import find_packages, setup

package_name = "bitbots_team_communication"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    install_requires=[
        "setuptools",
    ],
    tests_require=["pytest", "syrupy"],
)
