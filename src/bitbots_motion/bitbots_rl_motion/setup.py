import glob

from setuptools import find_packages, setup

package_name = "bitbots_rl_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/models",
            glob.glob("models/*.onnx"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mark oliver",
    maintainer_email="git@sWintermoor.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_policies = bitbots_rl_motion.policy_nodes:main",
        ],
    },
)
