import glob

from setuptools import setup, find_packages

package_name = 'humanoid_league_team_communication'

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ('share/' + package_name + "/config",
         glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
         glob.glob('launch/*.launch')),
    ],
    install_requires=[
        'setuptools',
    ],
    scripts=['scripts/show_team_comm.py', 'scripts/teamcomm_test_marker.py', 'scripts/TeamCommTest.py'],
    entry_points={
        'console_scripts': [
            'team_comm = humanoid_league_team_communication.humanoid_league_team_communication:main',
        ],
    }
)