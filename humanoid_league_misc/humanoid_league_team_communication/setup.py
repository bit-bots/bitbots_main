from setuptools import find_packages, setup

package_name = "humanoid_league_team_communication"

setup(
    name=package_name,
    packages=find_packages(exclude=["test"]),
    install_requires=[
        "setuptools",
    ],
    extras_require={
        "dev": ["pytest", "syrupy"],
    },
)
