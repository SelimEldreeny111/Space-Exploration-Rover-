from glob import glob

from setuptools import find_packages, setup

package_name = "rover_sim"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="MISSION X Team",
    maintainer_email="missionx.rover@example.com",
    description="MISSION X rover simulator node (rover_core.sim wrapper)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rover_sim_node = rover_sim.rover_sim_node:main",
        ],
    },
)
