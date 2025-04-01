from setuptools import find_packages, setup
from glob import glob

package_name = 'bitbots_extrinsic_calibration_capture_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.launch"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='par',
    maintainer_email='paer-wiegmann@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "capture_node = bitbots_extrinsic_calibration_capture_data.capture:main",
        ],
    },
)
