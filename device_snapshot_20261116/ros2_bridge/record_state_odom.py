#!/usr/bin/env python3
# 记录 /chassis/state 四轮 rpm + /odom 位姿到 CSV
import sys, time, csv
import rclpy
from navigation_interfaces.msg import ChassisState
from nav_msgs.msg import Odometry

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/state_odom.csv"
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

rclpy.init()
node = rclpy.create_node("state_odom_recorder")
f = open(out, "w", newline="")
w = csv.writer(f)
w.writerow(["t", "x", "y", "yaw_deg", "v", "w_deg", "fl", "rl", "fr", "rr"])
start = time.monotonic()
last = {"x": None, "y": None, "yaw": None, "v": 0.0, "w": 0.0, "rpm": [0.0, 0.0, 0.0, 0.0]}

def on_state(msg):
    last["rpm"] = list(msg.wheel_rpm)
    last["v"] = msg.linear_velocity
    last["w"] = msg.angular_velocity

def on_odom(msg):
    import math
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
    last["x"], last["y"], last["yaw"] = p.x, p.y, yaw
    if last["x"] is not None:
        rpm = last["rpm"]
        w.writerow([round(time.monotonic() - start, 4), last["x"], last["y"], round(last["yaw"], 3),
                    last["v"], last["w"], rpm[0], rpm[1], rpm[2], rpm[3]])
        f.flush()

node.create_subscription(ChassisState, "/chassis/state", on_state, 10)
node.create_subscription(Odometry, "/odom", on_odom, 10)
while time.monotonic() - start < duration:
    rclpy.spin_once(node, timeout_sec=0.05)
f.close()
node.destroy_node()
rclpy.shutdown()
