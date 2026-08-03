#!/usr/bin/env python3
"""Quick motor test — no launch needed. Run on device directly.
Usage: python3 /tmp/q.py
"""
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Joy

class Q(Node):
    def __init__(self):
        super().__init__("q")
        self.p = self.create_publisher(Joy, "/joy", 10)

    def send(self, th=0.0, st=0.0, bs=0):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "test"
        m.axes = [float(st), float(th), 0.0, 0.0]
        m.buttons = [bs, 0, 0, 0]
        self.p.publish(m)

rclpy.init(args=["--ros-args", "-r", "__node:=q"])
q = Q()

# 1. Enter MANUAL
print("MANUAL...")
q.send(0, 0, 1); time.sleep(0.2); q.send(0, 0, 0); time.sleep(0.5)

# 2. Forward 2s
print("FORWARD...")
for _ in range(20):
    q.send(th=0.3, st=0.0)
    rclpy.spin_once(q, timeout_sec=0.10)
    time.sleep(0.08)

# 3. Stop
print("STOP")
q.send(0, 0, 0); time.sleep(0.3)

q.destroy_node(); rclpy.shutdown()
print("DONE — check if robot moved")
