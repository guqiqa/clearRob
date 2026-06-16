"""
Launch file for Gazebo simulation of the cleaning robot.

Starts Gazebo with an outdoor/indoor cleaning scenario world
and spawns the cleaning robot model.

Usage:
    ros2 launch cleaning_robot_simulation simulation.launch.py
    ros2 launch cleaning_robot_simulation simulation.launch.py world:=indoor
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory("cleaning_robot_simulation")
    pkg_bringup = get_package_share_directory("cleaning_robot_bringup")

    # World selection
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="outdoor_campus",
        description="World to load: outdoor_campus / indoor_hall / test_room",
    )

    # Headless mode
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI",
    )

    # Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([pkg_sim, "worlds", LaunchConfiguration("world")]) + ".world",
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "cleaning_robot",
            "-file", PathJoinSubstitution([pkg_sim, "models", "cleaning_robot", "model.sdf"]),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
            "-Y", "0.0",
        ],
        output="screen",
    )

    # Robot state publisher (URDF → TF)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": PathJoinSubstitution([
                pkg_sim, "models", "cleaning_robot", "model.urdf"
            ]),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        gazebo,
        spawn_robot,
        robot_state_pub,
    ])
