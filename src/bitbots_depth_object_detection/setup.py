from setuptools import find_packages, setup

package_name = 'bitbots_depth_object_detection'

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
    maintainer='Pär Wiegmann',
    maintainer_email='baz1652@studium.uni-hamburg.de',
    description='This package contains a node for object detection using depth information from a zed stereo camera.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "depth_object_detection = bitbots_depth_object_detection.bitbots_depth_object_detection:main",
        ],
    },
)
