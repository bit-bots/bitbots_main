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


def build_html():
    output_path = "out"
    components_path = os.path.join(output_path, "components")
    template_path = "templates"

    env = Environment(loader=FileSystemLoader(template_path))

    # Render all templates excluding the ones in the components folder
    for root, _, files in os.walk(template_path):
        for file in files:
            if os.path.abspath(root) != os.path.abspath(components_path):
                if file.endswith(".html"):
                    template_file = os.path.relpath(os.path.join(root, file), template_path)
                    template = env.get_template(template_file)
                    output = template.render()

                    # Get install path
                    output_file_path = os.path.join(output_path, template_file)
                    os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

                    with open(output_file_path, "w") as f:
                        f.write(output)
                else:
                    # Copy all other files
                    src = os.path.join(root, file)
                    dst = os.path.join(output_path, os.path.relpath(src, template_path))
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src, dst)


build_html()

setup(
    name=package_name,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ]
    + generate_data_files("share/" + package_name + "/", "out/"),
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
        "console_scripts": [],
    },
)
