#!/usr/bin/env python3
"""
continuous_monitor.py — 持续监控转速/电流/遥控信号，直到 Ctrl-C。
采样 /chassis/status (轮速[0-3] + 电流指令[7-10] + 电流反馈[11-14])
和 /joy (axes + buttons)，打印到 stdout。
"""
import rclpy, time, sys
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

rclpy.init()
n = rclpy.create_node("continuous_monitor")
state = {"status": None, "joy": None, "t": time.time()}

def cb_status(msg):
    state["status"] = list(msg.data)

def cb_joy(msg):
    state["joy"] = (list(msg.axes), list(msg.buttons))

n.create_subscription(Float32MultiArray, "/chassis/status", cb_status, 10)
n.create_subscription(Joy, "/joy", cb_joy, 10)

print("持续监控中 — 按 Ctrl-C 停止", flush=True)
last_print = time.time()
while True:
    rclpy.spin_once(n, timeout_sec=0.1)
    now = time.time()
    if now - last_print >= 0.5:  # 2Hz 打印
        last_print = now
        s = state["status"]
        j = state["joy"]
        if s:
            rpm = [int(x) for x in s[0:4]]
            cmd = [int(x) for x in s[7:11]]
            fb = [int(x) for x in s[11:15]]
            print("[%6.1f] RPM[%s] CMD[%s] FB[%s]" % (
                now - state["t"],
                " ".join("%4d" % x for x in rpm),
                " ".join("%4d" % x for x in cmd),
                " ".join("%4d" % x for x in fb)), flush=True)
        if j:
            axes, btns = j
            print("        JOY ax[%s] btn[%s]" % (
                " ".join("%4.2f" % x for x in axes[:4]),
                " ".join(str(int(x)) for x in btns[:4])), flush=True)

try:
    while True:
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    print("\n监控停止")
finally:
    n.destroy_node()
    rclpy.shutdown()
