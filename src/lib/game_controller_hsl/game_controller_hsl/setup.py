import glob

from setuptools import find_packages, setup

package_name = 'game_controller_hsl'

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/config",
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch')),
    ],
    install_requires=[
        'launch',
        'setuptools',
        'construct',
    ],
    scripts=['scripts/sim_gamestate.py'],
    zip_safe=True,
    keywords=['ROS'],
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_controller = game_controller_hsl.receiver:main',
        ],
    }
)
