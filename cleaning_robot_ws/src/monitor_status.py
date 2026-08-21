#!/usr/bin/env python3
"""monitor_status.py — sample /chassis/status, print RPM + current cmd + current fb."""
import rclpy, time, sys
from std_msgs.msg import Float32MultiArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

rclpy.init()
n = rclpy.create_node("mon")
samples = []
def cb(msg):
    d = list(msg.data)
    if any(abs(x) > 1 for x in d[7:15]):
        samples.append(d)
n.create_subscription(Float32MultiArray, "/chassis/status", cb, 10)
print(f"monitoring {DUR}s — drive the robot now", flush=True)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
if samples:
    for s in samples[-4:]:
        print("RPM[%s] cmd_cur[%s] fb_cur[%s]" % (
            " ".join("%3d" % int(x) for x in s[0:4]),
            " ".join("%3d" % int(x) for x in s[7:11]),
            " ".join("%3d" % int(x) for x in s[11:15])))
else:
    print("no active samples captured")
