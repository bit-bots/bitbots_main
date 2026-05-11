from setuptools import find_packages, setup

package_name = "dynamic_stack_decider_visualization"

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ("share/" + package_name + "/resource", ["resource/StackmachineViz.ui"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dsd_gui = " + package_name + ".dsd_visualization_plugin:main",
        ],
    },
)
