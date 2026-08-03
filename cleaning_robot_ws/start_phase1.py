#!/usr/bin/env python3
"""Phase 1 launcher — no colcon, no ros2 launch, no setup.py needed.

Starts chassis_driver, remote_driver, master_bridge as subprocesses.
Each node runs via ros2 run through the proper PYTHONPATH.
"""

import os, sys, time, subprocess

WS = "/root/cleaning_robot_ws"

# Add all source dirs to PYTHONPATH
sys.path.insert(0, os.path.join(WS, "src", "cleaning_robot_common"))
sys.path.insert(0, os.path.join(WS, "src"))

os.environ["PYTHONPATH"] = (
    f"{WS}/src/cleaning_robot_common:"
    f"{WS}/src/cleaning_robot_drivers/remote_driver:"
    f"{WS}/src/cleaning_robot_drivers/chassis_driver:"
    f"{WS}/src/cleaning_robot_drivers/imu_driver:"
    f"{WS}/src/cleaning_robot_drivers/rtk_driver:"
    f"{WS}/src/master_bridge:"
    f"{WS}/src/cleaning_robot_bringup:"
    + os.environ.get("PYTHONPATH", "")
)

os.system("pkill -9 -f ros2 2>/dev/null")
os.system("systemctl stop LawnMower 2>/dev/null")
time.sleep(1)

# Helper: run ROS2 node via subprocess
def start_node(name, module_path, class_name, *extra_args):
    # Write a tiny runner script
    runner = f"""
import sys
sys.path.insert(0, "{WS}/src/cleaning_robot_common")
sys.path.insert(0, "{WS}/src")
from {module_path} import {class_name}
import rclpy
rclpy.init(args=["--ros-args", "-r", "__node:={name}"] + list("{' '.join(extra_args)}".split()))
rclpy.spin({class_name}())
"""
    return subprocess.Popen(
        ["python3", "-c", runner],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Starting chassis_driver...")
cd = start_node("chassis_driver", "chassis_driver.chassis_node",
                "ChassisDriverNode",
                "--ros-args", "-p", "simulate:=false", "-p", "left_motor_invert:=true")
time.sleep(2)

print("Starting remote_driver...")
rd = start_node("remote_driver", "remote_driver.remote_node",
                "RemoteDriverNode",
                "--ros-args", "-p", "simulate:=false")
time.sleep(2)

print("Starting master_bridge...")
mb = start_node("master_bridge", "master_bridge.bridge_node",
                "MasterBridgeNode",
                "--ros-args", "-p", "simulate:=false")
time.sleep(3)

print("Waiting for nodes to come up...")
time.sleep(5)

print("\n=== Phase 1 Running ===")
print("check: ros2 topic list | ros2 topic echo /master/state")
print("stop:  kill %d %d %d" % (cd.pid, rd.pid, mb.pid))

try:
    cd.wait()
    rd.wait()
    mb.wait()
except KeyboardInterrupt:
    cd.kill()
    rd.kill()
    mb.kill()
    print("\nStopped.")
