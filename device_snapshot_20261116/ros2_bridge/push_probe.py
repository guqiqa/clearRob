#!/usr/bin/env python3
"""手动推车标定探针（不发任何运动指令）。

用法:
  python3 push_probe.py forward 8   # 站车尾后方，把车朝车头方向推 0.3~0.4 m
  python3 push_probe.py rotate 8    # 站车尾后方，把车头朝你的左手边（俯视逆时针）转约 45°

输出运动段（任一 |wheel_rpm|>0.5）的：
  dx/dy/dist、dyaw、四轮均值与左右均值。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import ChassisState

MODE = sys.argv[1] if len(sys.argv) > 1 else "forward"
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0


def wrap_deg(v):
    while v > 180.0:
        v -= 360.0
    while v < -180.0:
        v += 360.0
    return v


class PushProbe(Node):
    def __init__(self):
        super().__init__("push_probe")
        self.x = self.y = self.yaw = None
        self.wheels = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(ChassisState, "/chassis/state", self.on_state, 10)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def on_state(self, m):
        self.wheels = [float(v) for v in m.wheel_rpm]

    def run(self):
        t0 = time.monotonic()
        while (self.yaw is None or self.wheels is None) and time.monotonic() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.yaw is None or self.wheels is None:
            print("NO_ODOM_OR_STATE")
            return 2
        print("=== push_probe mode=%s 开始 %.0f s ===" % (MODE, DUR))
        if MODE == "forward":
            print("请在 8 秒内：站车尾后方，把车朝车头方向平稳推 0.3~0.4 m，然后停")
        else:
            print("请在 8 秒内：站车尾后方，把车头朝你的左手边（俯视逆时针）转约 45°，然后停")
        print("start x=%.3f y=%.3f yaw=%.2f wheels=%s" % (self.x, self.y, self.yaw, self.wheels),
              flush=True)
        samples = []
        t_start = time.monotonic()
        next_print = t_start + 1.0
        while time.monotonic() - t_start < DUR:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.wheels is not None:
                samples.append((time.monotonic() - t_start, self.x, self.y, self.yaw,
                                list(self.wheels)))
                now = time.monotonic()
                if now >= next_print:
                    print("t=%.1f x=%.3f y=%.3f yaw=%.2f wheels=%s"
                          % (samples[-1][0], self.x, self.y, self.yaw, self.wheels),
                          flush=True)
                    next_print += 1.0
            time.sleep(0.05)
        moving = [s for s in samples if max(abs(w) for w in s[4]) > 0.5]
        if not moving:
            print("NO_MOTION：没有检测到轮子转动")
            return 1
        first, last = moving[0], moving[-1]
        dx = last[1] - first[1]
        dy = last[2] - first[2]
        dyaw = wrap_deg(last[3] - first[3])
        n = len(moving)
        fl = sum(s[4][0] for s in moving) / n
        rl = sum(s[4][1] for s in moving) / n
        fr = sum(s[4][2] for s in moving) / n
        rr = sum(s[4][3] for s in moving) / n
        print("MOTION n=%d dx=%+.3f dy=%+.3f dist=%.3f dyaw=%+.2fdeg "
              "fl=%.2f rl=%.2f fr=%.2f rr=%.2f left=%.2f right=%.2f"
              % (n, dx, dy, math.hypot(dx, dy), dyaw, fl, rl, fr, rr,
                 (fl + rl) / 2.0, (fr + rr) / 2.0))
        print("=== DONE ===")
        return 0


def main():
    rclpy.init()
    node = PushProbe()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
