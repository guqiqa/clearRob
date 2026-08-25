import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("native_stereo_camera")
    default_params = os.path.join(share, "config", "native_stereo_camera.yaml")
    params = DeclareLaunchArgument("params_file", default_value=default_params)
    node = Node(
        package="native_stereo_camera",
        executable="native_stereo_camera_node",
        name="native_stereo_camera",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription([params, node])
