from setuptools import find_packages, setup

package_name = "chassis_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIview Dev",
    maintainer_email="aiview@example.com",
    description="CAN bus motor controller for OID FOC brushless motors.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "chassis_node = chassis_driver.chassis_node:main",
        ],
    },
)
