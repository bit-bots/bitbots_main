from setuptools import find_packages, setup

package_name = "bitbots_team_data_sim_rqt"

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ("share/" + package_name + "/resource", ["resource/RecordUI.ui"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "animation_gui = " + package_name + ".record_ui:main",
        ],
    },
)
