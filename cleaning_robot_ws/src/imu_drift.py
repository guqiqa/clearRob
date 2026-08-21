#!/usr/bin/env python3
"""imu_drift.py — measure IMU yaw drift while stationary."""
import rclpy, time, math
from sensor_msgs.msg import Imu

rclpy.init()
n = rclpy.create_node("imu_drift")
yaw = 0.0
last_t = None
yaws = []
def cb(msg):
    global yaw, last_t
    now = time.time()
    if last_t is not None:
        dt = now - last_t
        if 0 < dt < 0.5:
            yaw += math.degrees(msg.angular_velocity.z) * dt
    last_t = now
    yaws.append(yaw)
n.create_subscription(Imu, "/imu/data", cb, 10)
print("sampling stationary IMU 8s...", flush=True)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node(); rclpy.shutdown()
if yaws:
    print(f"samples={len(yaws)} yaw_range={min(yaws):.2f}~{max(yaws):.2f} drift={max(yaws)-min(yaws):.3f} deg")
