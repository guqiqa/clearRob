from setuptools import find_packages, setup

package_name = "cleaning_robot_common"

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
    description="Shared config, kinematics, CAN protocol for cleaning robot.",
    license="Apache-2.0",
)
