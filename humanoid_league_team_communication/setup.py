import glob

from setuptools import setup, find_packages

package_name = 'humanoid_league_team_communication'

setup(
    name=package_name,
    packages=find_packages(),
    install_requires=[
        'setuptools',
    ],
)
