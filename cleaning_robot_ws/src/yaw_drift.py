#!/usr/bin/env python3
"""yaw_drift.py — measure calibrated yaw drift from /chassis/status field[4]."""
import rclpy, time
from std_msgs.msg import Float32MultiArray

rclpy.init()
n = rclpy.create_node("yaw_drift")
yaws = []
def cb(msg):
    d = list(msg.data)
    if len(d) > 4:
        yaws.append(d[4])  # calibrated yaw_deg used by heading hold
n.create_subscription(Float32MultiArray, "/chassis/status", cb, 10)
print("sampling calibrated yaw 8s (car stationary)", flush=True)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
if yaws:
    print(f"samples={len(yaws)} yaw_range={min(yaws):.2f}~{max(yaws):.2f} drift={max(yaws)-min(yaws):.3f} deg")
else:
    print("no status data")
