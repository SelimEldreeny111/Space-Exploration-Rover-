from setuptools import find_packages, setup

package_name = "rover_core"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="MISSION X Team",
    maintainer_email="missionx.rover@example.com",
    description="MISSION X rover core library (SSP protocol, kinematics, simulator)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
