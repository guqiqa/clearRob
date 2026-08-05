#!/usr/bin/env python3
"""
Phase 1 chassis bringup launch file.

Launches:
  1. imu_driver    — QMI8658 6-axis IMU
  2. rtk_driver    — NTRIP GNSS positioning
  3. remote_driver — SBUS RC receiver
  4. chassis_driver — CAN bus motor control
  5. master_bridge — STANDBY/MANUAL state machine

Usage:
  ros2 launch cleaning_robot_bringup phase1_chassis.launch.py
  ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("cleaning_robot_bringup")
    params_file = os.path.join(pkg_share, "config", "phase1_params.yaml")

    # ---- Launch arguments ----
    sim_arg = DeclareLaunchArgument(
        "simulate", default_value="false",
        description="Run in simulation mode (no hardware required)"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="",
        description="Robot namespace (empty = no namespace)"
    )
    autopilot_arg = DeclareLaunchArgument(
        "autopilot", default_value="false",
        description="Launch Phase 2 autonomous nodes (path_planner + fusion_engine)"
    )

    sim_val = LaunchConfiguration("simulate")
    ns = LaunchConfiguration("namespace")
    autopilot_val = LaunchConfiguration("autopilot")

    # ---- Node definitions ----
    imu_node = Node(
        package="imu_driver",
        executable="imu_node",
        name="imu_driver",
        namespace=ns,
        output="screen",
        parameters=[params_file, {"simulate": sim_val}],
    )

    remote_node = Node(
        package="remote_driver",
        executable="remote_node",
        name="remote_driver",
        namespace=ns,
        output="screen",
        parameters=[params_file, {"simulate": sim_val}],
    )

    chassis_node = Node(
        package="chassis_driver",
        executable="chassis_node",
        name="chassis_driver",
        namespace=ns,
        output="screen",
        parameters=[
            params_file,
            {"simulate": sim_val},
        ],
    )

    bridge_node = Node(
        package="master_bridge",
        executable="bridge_node",
        name="master_bridge",
        namespace=ns,
        output="screen",
        parameters=[params_file],
    )

    # ---- Phase 2 nodes (autonomous) — off by default, enable with autopilot:=true ----
    path_planner_node = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner",
        namespace=ns,
        output="screen",
        condition=IfCondition(autopilot_val),
        parameters=[params_file],
    )

    fusion_node = Node(
        package="fusion_engine",
        executable="fusion_node",
        name="fusion_engine",
        namespace=ns,
        output="screen",
        condition=IfCondition(autopilot_val),
        parameters=[params_file],
    )

    # RTK — skip in sim mode (no GPS signal indoors)
    rtk_node_sim = Node(
        package="rtk_driver",
        executable="rtk_node",
        name="rtk_driver",
        namespace=ns,
        output="screen",
        condition=IfCondition(sim_val),
        parameters=[params_file, {"simulate": True}],
    )
    rtk_node_real = Node(
        package="rtk_driver",
        executable="rtk_node",
        name="rtk_driver",
        namespace=ns,
        output="screen",
        condition=UnlessCondition(sim_val),
        parameters=[params_file, {"simulate": False}],
    )

    return LaunchDescription([
        sim_arg,
        namespace_arg,
        autopilot_arg,
        LogInfo(msg=["Phase 1 chassis bringup — simulate=", sim_val]),
        # Launch drivers with staggered start to avoid CAN bus contention
        imu_node,
        TimerAction(period=0.5, actions=[chassis_node]),
        TimerAction(period=1.0, actions=[remote_node]),
        TimerAction(period=1.5, actions=[rtk_node_sim, rtk_node_real]),
        TimerAction(period=2.0, actions=[bridge_node]),
        # Phase 2 autonomous nodes (staggered, only if autopilot:=true)
        TimerAction(period=2.5, actions=[fusion_node]),
        TimerAction(period=3.0, actions=[path_planner_node]),
    ])
