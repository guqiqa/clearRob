#!/usr/bin/env python3
"""RC 直行监控：记录 /odom 与 /chassis/state，实时打印摘要，结束输出转角/漂移统计。

用法: python3 rc_straight_monitor.py [秒数] [csv路径]
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import ChassisState

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/rc_straight.csv"


def wrap_deg(v):
    while v > 180.0:
        v -= 360.0
    while v < -180.0:
        v += 360.0
    return v


class Mon(Node):
    def __init__(self):
        super().__init__("rc_straight_monitor")
        self.x = self.y = self.yaw = self.v = self.w = None
        self.wheels = None
        self.mode = ""
        self.fault = ""
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(ChassisState, "/chassis/state", self.on_state, 10)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z

    def on_state(self, m):
        self.wheels = [float(v) for v in m.wheel_rpm]
        self.mode = str(m.control_mode)
        self.fault = str(m.fault_reason)

    def run(self):
        t0 = time.monotonic()
        while (self.yaw is None or self.wheels is None) and time.monotonic() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.yaw is None or self.wheels is None:
            print("NO_ODOM_OR_STATE")
            return 2

        f = open(OUT, "w", newline="")
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "yaw_deg", "v", "w", "fl", "rl", "fr", "rr", "mode", "fault"])
        start = time.monotonic()
        next_print = start
        rows = []
        print("RECORDING %.0f s to %s" % (DUR, OUT), flush=True)
        while time.monotonic() - start < DUR:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.wheels is None or self.yaw is None:
                continue
            t = time.monotonic() - start
            row = [t, self.x, self.y, self.yaw, self.v, self.w,
                   self.wheels[0], self.wheels[1], self.wheels[2], self.wheels[3],
                   self.mode, self.fault]
            w.writerow(row)
            rows.append(row)
            now = time.monotonic()
            if now >= next_print:
                print("t=%5.1f x=%7.3f y=%7.3f yaw=%8.2f v=%6.3f w=%7.3f wheels=%s mode=%s"
                      % (t, self.x, self.y, self.yaw, self.v, self.w,
                         ["%.1f" % q for q in self.wheels], self.mode), flush=True)
                next_print += 1.0
            time.sleep(0.02)
        f.close()

        moving = [r for r in rows if abs(r[4]) > 0.01 or max(abs(q) for q in r[6:10]) > 0.5]
        print("ROW_COUNT=%d MOVING=%d" % (len(rows), len(moving)), flush=True)
        if len(moving) > 2:
            first, last = moving[0], moving[-1]
            dyaw = wrap_deg(last[3] - first[3])
            dist = math.hypot(last[1] - first[1], last[2] - first[2])
            n = len(moving)
            fl = sum(r[6] for r in moving) / n
            rl = sum(r[7] for r in moving) / n
            fr = sum(r[8] for r in moving) / n
            rr = sum(r[9] for r in moving) / n
            print("SUMMARY dyaw=%+.2f deg dist=%.3f m drift_per_m=%+.2f deg/m "
                  "fl=%.2f rl=%.2f fr=%.2f rr=%.2f left=%.2f right=%.2f diff_R_minus_L=%.2f"
                  % (dyaw, dist, (dyaw / dist if dist > 0.05 else 0.0),
                     fl, rl, fr, rr, (fl + rl) / 2.0, (fr + rr) / 2.0,
                     (fr + rr - fl - rl) / 2.0), flush=True)
        print("DONE_OUT=%s" % OUT, flush=True)
        return 0


def main():
    rclpy.init()
    node = Mon()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
