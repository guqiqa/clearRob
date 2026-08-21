#!/usr/bin/env python3
"""
rc_trace.py — 持续记录遥控信号 + 底盘轮速/电流到文件。
后台运行，用户操作遥控器时自动留痕，之后一次性读取分析。
用法: python3 rc_trace.py [时长秒] [输出文件]
"""
import rclpy, time, sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/root/rc_trace.log"

rclpy.init()
n = rclpy.create_node("rc_trace")
f = open(OUT, "w")
f.write("# t  joy_ax0 joy_ax1 joy_ax2 btn0 btn2 | rpm_fl rpm_rl rpm_fr rpm_rr | cur_fl cur_rl cur_fr cur_rr\n")
f.flush()
last = {"joy": None, "status": None, "t": time.time()}

def cb_joy(msg):
    last["joy"] = (list(msg.axes), list(msg.buttons))

def cb_status(msg):
    last["status"] = list(msg.data)

n.create_subscription(Joy, "/joy", cb_joy, 10)
n.create_subscription(Float32MultiArray, "/chassis/status", cb_status, 10)

print(f"记录 {DUR}s 到 {OUT} — 请操作遥控器", flush=True)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.1)
    now = time.time() - t0
    j = last["joy"]; s = last["status"]
    if j and s:
        ax, bt = j
        rpm = [int(x) for x in s[0:4]]
        cur = [int(x) for x in s[7:11]]
        f.write(f"{now:6.1f} {ax[0]:+.2f} {ax[1]:+.2f} {ax[2]:+.2f} {int(bt[0])} {int(bt[2])} | "
                f"{rpm[0]:3d} {rpm[1]:3d} {rpm[2]:3d} {rpm[3]:3d} | "
                f"{cur[0]:4d} {cur[1]:4d} {cur[2]:4d} {cur[3]:4d}\n")
        f.flush()
f.close()
n.destroy_node(); rclpy.shutdown()
print(f"记录完成: {OUT}", flush=True)
