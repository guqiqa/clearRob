from setuptools import setup

package_name = "s90_h264_input"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/stereo.yaml"]),
        ("share/" + package_name + "/launch", ["launch/stereo.launch.py"]),
        ("share/" + package_name + "/systemd", ["systemd/s90-stereo-ros.service"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "s90_h264_input_node = s90_h264_input.s90_h264_input_node:main",
        ],
    },
)
