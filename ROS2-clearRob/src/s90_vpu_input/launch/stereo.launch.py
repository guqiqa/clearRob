import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("s90_vpu_input")
    params = os.path.join(share, "config", "stereo.yaml")
    return LaunchDescription([
        Node(
            package="s90_vpu_input",
            executable="s90_vpu_input_node",
            name="s90_vpu_input",
            parameters=[params],
            output="screen",
        )
    ])
