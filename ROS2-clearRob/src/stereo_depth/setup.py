from glob import glob
import os

from setuptools import find_packages, setup


package_name = "stereo_depth"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "systemd"), glob("systemd/*.service")),
        (os.path.join("share", package_name, "tools"), glob("tools/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIview Dev",
    maintainer_email="aiview@example.com",
    description="Lightweight stereo depth frontend for SLAM mapping.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stereo_depth_node = stereo_depth.stereo_depth_node:main",
        ],
    },
)
