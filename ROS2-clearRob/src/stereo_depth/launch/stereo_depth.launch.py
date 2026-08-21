import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("stereo_depth")
    default_params = os.path.join(pkg_share, "config", "stereo_depth.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Stereo depth parameter file",
    )

    node = Node(
        package="stereo_depth",
        executable="stereo_depth_node",
        name="stereo_depth",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([params_arg, node])
