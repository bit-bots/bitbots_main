from setuptools import setup, find_packages

package_name = 'system_monitor'

setup(
    name=package_name,
    packages=find_packages(),
    install_requires=[
        'setuptools',
    ],
    entry_points={
        'console_scripts': [
            'monitor = system_monitor.monitor:main',
        ],
    }
)