#!/usr/bin/env python3
import math, sys
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node("yaw_once")
out = []
def cb(msg):
    q = msg.pose.pose.orientation
    out.append(math.degrees(2.0 * math.atan2(q.z, q.w)))
node.create_subscription(Odometry, "/odom", cb, 10)
end = 0
import time
start = time.time()
while time.time() - start < 5.0 and not out:
    rclpy.spin_once(node, timeout_sec=0.1)
if out:
    print("yaw_deg=%.2f" % out[0])
else:
    print("NO_ODOM")
node.destroy_node()
rclpy.shutdown()
