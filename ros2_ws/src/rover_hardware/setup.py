from glob import glob

from setuptools import find_packages, setup

package_name = "rover_hardware"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial", "numpy"],
    zip_safe=True,
    maintainer="MISSION X Team",
    maintainer_email="missionx.rover@example.com",
    description="MISSION X SSP serial bridges (mobility, payload, arm, ground station)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mobility_bridge = rover_hardware.mobility_bridge:main",
            "payload_bridge = rover_hardware.payload_bridge:main",
            "arm_bridge = rover_hardware.arm_bridge:main",
            "ground_station_bridge = rover_hardware.ground_station_bridge:main",
        ],
    },
)
