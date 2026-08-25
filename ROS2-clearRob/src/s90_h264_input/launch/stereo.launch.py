import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("s90_h264_input"), "config", "stereo.yaml")
    return LaunchDescription(
        [
            Node(
                package="s90_h264_input",
                executable="s90_h264_input_node",
                name="s90_h264_left",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="s90_h264_input",
                executable="s90_h264_input_node",
                name="s90_h264_right",
                parameters=[params],
                output="screen",
            ),
        ]
    )
