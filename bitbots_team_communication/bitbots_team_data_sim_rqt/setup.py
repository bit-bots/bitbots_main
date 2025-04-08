from setuptools import find_packages, setup

package_name = "bitbots_team_data_sim_rqt"

setup(
    name=package_name,
    packages=find_packages(),
    data_files=[
        ("share/" + package_name + "/resource", ["resource/RobotTeamDataSimulator.ui"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "team_data_sim_gui = " + package_name + ".team_data_ui:main",
        ],
    },
)
