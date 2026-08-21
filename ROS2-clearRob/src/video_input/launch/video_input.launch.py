import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("video_input")
    default_params = os.path.join(pkg_share, "config", "video_input.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Video input parameter file",
    )

    node = Node(
        package="video_input",
        executable="video_input_node",
        name="video_input",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([params_arg, node])
