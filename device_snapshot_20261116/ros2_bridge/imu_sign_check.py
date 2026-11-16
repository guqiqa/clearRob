#!/usr/bin/env python3
# 同时记录 IMU 原始 z 轴角速度与 /odom 发布角度，检查符号一致性
import sys, time, math
import rclpy
from nav_msgs.msg import Odometry

RAW = "/sys/bus/iio/devices/iio:device0/in_anglvel_z_raw"
duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

rclpy.init()
node = rclpy.create_node("imu_sign_check")
state = {"yaw": None}

def cb(msg):
    q = msg.pose.pose.orientation
    state["yaw"] = math.degrees(2.0 * math.atan2(q.z, q.w))

node.create_subscription(Odometry, "/odom", cb, 10)
samples = []
start = time.monotonic()
while time.monotonic() - start < duration:
    rclpy.spin_once(node, timeout_sec=0.02)
    try:
        raw = float(open(RAW).read().strip())
    except Exception:
        raw = 0.0
    if state["yaw"] is not None:
        samples.append((time.monotonic() - start, raw, state["yaw"]))
    time.sleep(0.05)

raw_sum = sum(s[1] for s in samples)
yaw0 = samples[0][2] if samples else None
yaw1 = samples[-1][2] if samples else None
print("samples=%d" % len(samples))
print("raw_sum=%.1f" % raw_sum)
print("yaw_start=%s yaw_end=%s" % (("%.2f" % yaw0) if yaw0 is not None else "None", ("%.2f" % yaw1) if yaw1 is not None else "None"))
if len(samples) > 3:
    xs = [s[1] for s in samples[1:]]
    ys = [s[2] - samples[i][2] for i, s in enumerate(samples[1:])]
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = sum((x - mx) ** 2 for x in xs) ** 0.5
    dy = sum((y - my) ** 2 for y in ys) ** 0.5
    corr = num / (dx * dy) if dx * dy else 0.0
    print("corr=%.3f" % corr)
