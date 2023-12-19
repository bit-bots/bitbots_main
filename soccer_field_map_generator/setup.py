from setuptools import find_packages, setup

package_name = 'soccer_field_map_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='florian',
    maintainer_email='git@flova.de',
    description='A package to generate a soccer field map using a gui or cli',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = soccer_field_map_generator.gui:main',
            'cli = soccer_field_map_generator.cli:main',
        ],
    },
)
