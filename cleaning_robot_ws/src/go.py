#!/usr/bin/env python3
"""One-click motor test — launches nodes + keyboard control.

Usage: python3 /tmp/go.py
W/S = forward/back, A/D = turn, Space = MANUAL/STANDBY, Q = quit
"""

import os, sys, time, select, tty, termios, subprocess

# 1. Ensure ROS2 Python path
ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if ROS2_SITE not in sys.path:
    sys.path.insert(0, ROS2_SITE)

WS = "/root/cleaning_robot_ws"

# 2. Kill old processes
for p in ["chassis_node","bridge_node","remote_node","imu_node","rtk_node","ros2"]:
    os.system(f"kill -9 $(pgrep -f {p} 2>/dev/null) 2>/dev/null")
os.system("systemctl stop LawnMower 2>/dev/null")
time.sleep(1)

# 3. CAN setup
os.system("ip link set can0 type can bitrate 500000 2>/dev/null")
os.system("ip link set can0 up 2>/dev/null")

# 4. Launch nodes via bash (sources ROS2 env internally)
def launch(cmd):
    return subprocess.Popen(
        f"bash -c 'source {WS}/install/setup.bash && exec {cmd}'",
        shell=True, executable="/bin/bash",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Starting chassis_driver...")
launch("ros2 run chassis_driver chassis_node --ros-args -p simulate:=false -p left_motor_invert:=true")
time.sleep(3)

print("Starting master_bridge...")
launch("ros2 run master_bridge bridge_node --ros-args -p simulate:=false")
time.sleep(3)

# 5. Now import ROS2 (nodes are up, PYTHONPATH is set via workspace setup)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class KB(Node):
    def __init__(self):
        super().__init__("kb")
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.v = self.w = 0.0
        self.s = "STANDBY"
        self.create_subscription(Twist, "/cmd_vel",
            lambda m: (setattr(self,'v',m.linear.x), setattr(self,'w',m.angular.z)), 10)
        self.create_subscription(String, "/master/state",
            lambda m: setattr(self,'s',m.data), 10)

    def send(self, th=0.0, st=0.0, bs=0):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [float(st), float(th), 0.0, 0.0]
        m.buttons = [bs, 0, 0, 0]
        self.pub.publish(m)

    def tick(self, d=0.02):
        t = time.time()
        while time.time() - t < d:
            rclpy.spin_once(self, timeout_sec=0.01)

    def show(self, label):
        print(f"  [{self.s:8s}] v={self.v:+.2f} w={self.w:+.2f}  {label}")


rclpy.init(args=["--ros-args", "-r", "__node:=kb"])
k = KB()

# Wait for bridge to subscribe
for _ in range(30):
    k.send(th=0.0, st=0.0, bs=0)
    k.tick(0.05)

# Enter MANUAL
k.send(th=0.0, st=0.0, bs=1)
k.tick(0.15)
k.send(th=0.0, st=0.0, bs=0)
k.tick(0.5)

print(f"\n{'='*50}")
print(f"  W/S=+-Fwd  A/D=Turn  Space=Toggle  Q=Quit")
print(f"  State: {k.s}")
print(f"{'='*50}\n")

old = termios.tcgetattr(sys.stdin)
tty.setraw(sys.stdin.fileno())

try:
    while rclpy.ok():
        if select.select([sys.stdin], [], [], 0.05)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'w':
                k.send(th=0.3); k.tick(); k.show("FORWARD")
            elif key == 's':
                k.send(th=-0.2); k.tick(); k.show("BACK")
            elif key == 'a':
                k.send(st=0.5); k.tick(); k.show("LEFT")
            elif key == 'd':
                k.send(st=-0.5); k.tick(); k.show("RIGHT")
            elif key == ' ':
                k.send(bs=1); k.tick(0.05)
                k.send(bs=0); k.tick(0.2)
                k.show("TOGGLE")
            elif key == 'q':
                print("\nQUIT")
                break
        else:
            k.send(th=0.0, st=0.0)
            k.tick(0.01)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    k.destroy_node(); rclpy.shutdown()
    os.system("kill -9 $(pgrep -f chassis_node) 2>/dev/null")
    os.system("kill -9 $(pgrep -f bridge_node) 2>/dev/null")
    print("Done.")
