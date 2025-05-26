import glob
import os
import shutil

from jinja2 import Environment, FileSystemLoader
from setuptools import find_packages, setup

package_name = "bitbots_education"

def generate_data_files(share_path, dir):
    data_files = []

    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith(".")])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ]
    + generate_data_files("share/" + package_name + "/", "templates/")
    + generate_data_files("share/" + package_name + "/", "static/"),
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="wedmann.lea@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "webserver = bitbots_education.app:main",
        ],
    },
)
