from setuptools import find_packages, setup

package_name = "master_bridge"

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
    description="Phase 1 master bridge: remote→chassis state machine.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = master_bridge.bridge_node:main",
        ],
    },
)
