"""
Launch file for vision_detector node (M1).

Usage:
  ros2 launch vision_detector vision_detector.launch.py \
      model_path:=/path/to/best.pt \
      device:=cuda:0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Launch arguments ----------------------------------------------------
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Path to YOLO model file (.pt / .onnx / .engine)",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Inference device: cuda:0 | cpu | tensorrt",
    )
    conf_threshold_arg = DeclareLaunchArgument(
        "conf_threshold",
        default_value="0.45",
        description="Confidence threshold (0.0–1.0)",
    )
    half_precision_arg = DeclareLaunchArgument(
        "half_precision",
        default_value="true",
        description="Enable FP16 inference",
    )

    # Node -----------------------------------------------------------------
    vision_detector_node = Node(
        package="vision_detector",
        executable="vision_detector_node",
        name="vision_detector",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("vision_detector"), "config", "vision_detector.yaml"]
            ),
            {
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "half_precision": LaunchConfiguration("half_precision"),
            },
        ],
        remappings=[
            # In case the camera topic has a different name:
            # ("sensor/camera/image_raw", "/actual/camera/topic"),
        ],
    )

    # Log warning if model_path is empty ----------------------------------
    warn_no_model = LogInfo(
        condition=IfCondition(
            '"" == ""'  # default — harmless
        ),
        message=(
            "WARNING: 'model_path' parameter is empty. "
            "The node will fail to configure until a valid model path is set."
        ),
    )

    return LaunchDescription(
        [
            model_path_arg,
            device_arg,
            conf_threshold_arg,
            half_precision_arg,
            # warn_no_model,
            vision_detector_node,
        ]
    )
