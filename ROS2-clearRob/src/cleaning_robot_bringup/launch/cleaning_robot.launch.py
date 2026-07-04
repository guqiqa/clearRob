"""
Launch file for the cleaning robot system.

Launches all 9 ROS2 nodes in order with proper parameters.

Usage:
    ros2 launch cleaning_robot_bringup cleaning_robot.launch.py
    ros2 launch cleaning_robot_bringup cleaning_robot.launch.py sim_mode:=true
    ros2 launch cleaning_robot_bringup cleaning_robot.launch.py robot_id:=robot_002
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Paths ----
    pkg_bringup = get_package_share_directory("cleaning_robot_bringup")
    default_params = os.path.join(pkg_bringup, "config", "default_params.yaml")

    # ---- Launch arguments ----
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_001",
        description="Unique robot identifier for multi-robot scenarios",
    )
    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="true",
        description="Enable simulation mode (simulated sensors)",
    )
    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Use robot_id as ROS2 namespace for multi-robot support",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS2 logging level (debug/info/warn/error/fatal)",
    )

    # ---- Node definitions ----
    # Each node is launched with the default parameter file + any overrides

    def make_node(pkg, node_name, extra_params=None):
        """Create a Node with standard configuration."""
        params = [default_params]
        if extra_params:
            params.extend(extra_params)
        return Node(
            package=pkg,
            executable=node_name,
            name=node_name.replace("_node", ""),
            parameters=params,
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            emulate_tty=True,
        )

    # 1. Interfaces (no node to launch, just a dependency)
    # 2. Motion controller - need this first for odometry
    motion_node = make_node("motion_controller", "motion_controller_node")

    # 3. Localization engine - needs odometry + lidar
    localization_node = TimerAction(
        period=2.0,
        actions=[make_node("localization_engine", "localization_engine_node")],
    )

    # 4. LiDAR perception - sensor frontend
    lidar_node = TimerAction(
        period=1.0,
        actions=[make_node("lidar_perception", "lidar_perception_node")],
    )

    # 5. Vision detector - sensor frontend
    vision_node = TimerAction(
        period=1.5,
        actions=[make_node("vision_detector", "vision_detector_node")],
    )

    # 6. Fusion engine - needs vision + lidar
    fusion_node = TimerAction(
        period=3.0,
        actions=[make_node("fusion_engine", "fusion_engine_node")],
    )

    # 7. Path planner - needs localization + map
    path_planner_node = TimerAction(
        period=2.5,
        actions=[make_node("path_planner", "path_planner_node")],
    )

    # 8. Control gateway - system entry
    control_gateway_node = make_node("control_gateway", "control_gateway_node")

    # 9. Cleaning actuator - needs strategy + vision
    cleaning_node = TimerAction(
        period=3.5,
        actions=[make_node("cleaning_actuator", "cleaning_actuator_node")],
    )

    # 10. Master controller - system brain, needs all others up first
    master_node = TimerAction(
        period=4.0,
        actions=[make_node("master_controller", "master_controller_node")],
    )

    # ---- Node list ----
    all_nodes = [
        motion_node,
        localization_node,
        lidar_node,
        vision_node,
        fusion_node,
        path_planner_node,
        control_gateway_node,
        cleaning_node,
        master_node,
    ]

    return LaunchDescription([
        # Environment
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        # Arguments
        robot_id_arg,
        sim_mode_arg,
        use_namespace_arg,
        log_level_arg,
        # Single-robot: no namespace (condition: use_namespace is false)
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration("use_namespace")),
            actions=all_nodes,
        ),
        # Multi-robot: wrap in namespace (condition: use_namespace is true)
        GroupAction(
            condition=IfCondition(LaunchConfiguration("use_namespace")),
            actions=[
                PushRosNamespace(LaunchConfiguration("robot_id")),
                *all_nodes,
            ],
        ),
    ])
