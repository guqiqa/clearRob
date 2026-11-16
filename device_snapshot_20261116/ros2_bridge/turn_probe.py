#!/usr/bin/env python3
"""原地转向方向探针：v=0.0005, w=+0.30, 3 s，打印 odom yaw 变化与四轮平均 rpm。

观察要点（以车自身左右为准）：
  左轮往前滚 + 右轮往后滚 -> 顺时针（右转）
  左轮往后滚 + 右轮往前滚 -> 逆时针（左转）
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from navigation_interfaces.msg import ChassisState, SafeMotionCommand

V = 0.0005
W = 0.30
DUR = 3.0


def wrap_deg(v):
    while v > 180.0:
        v -= 360.0
    while v < -180.0:
        v += 360.0
    return v


class TurnProbe(Node):
    def __init__(self):
        super().__init__("turn_probe")
        self.x = self.y = self.yaw = None
        self.wheels = None
        self.estop = self.feedback = self.can_ok = None
        self.msg = SafeMotionCommand()
        self.msg.ttl_s = 0.10
        self.msg.enable = True
        self.msg.stop_required = False
        self.msg.emergency_stop = False
        self.msg.source = "turn_probe"
        self.msg.reason = "turn_probe"
        self.pub = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(ChassisState, "/chassis/state", self.on_state, 10)
        self.mode = self.create_client(SetBool, "/chassis/set_autonomous_mode")

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def on_state(self, m):
        self.wheels = [float(v) for v in m.wheel_rpm]
        self.estop = bool(m.emergency_stop)
        self.feedback = bool(m.feedback_valid)
        self.can_ok = bool(m.can_healthy)

    def publish(self, v, w):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.sequence = time.monotonic_ns() // 1000
        self.msg.linear_x = float(v)
        self.msg.angular_z = float(w)
        self.pub.publish(self.msg)

    def mode_call(self, data):
        if not self.mode.wait_for_service(timeout_sec=5.0):
            return False
        req = SetBool.Request()
        req.data = bool(data)
        fut = self.mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.done() and fut.result() is not None and fut.result().success)

    def run(self):
        t0 = time.monotonic()
        while (self.yaw is None or self.wheels is None) and time.monotonic() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.yaw is None or self.wheels is None:
            print("NO_ODOM_OR_STATE")
            return 2
        print("PRECHECK estop=%s feedback=%s can=%s yaw=%.2f pos=(%.2f,%.2f) wheels=%s"
              % (self.estop, self.feedback, self.can_ok, self.yaw, self.x, self.y, self.wheels))
        if self.estop or not self.feedback or not self.can_ok:
            print("PRECHECK_FAIL")
            return 3
        if self.count_publishers("/safety/safe_motion_command") > 0:
            print("STRAY_SAFE_CMD_PUBLISHER")
            return 4
        self.pub = self.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
        t0 = time.monotonic()
        while self.pub.get_subscription_count() == 0 and time.monotonic() - t0 < 6.0:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        if self.pub.get_subscription_count() == 0:
            print("NO_SUBSCRIBER")
            return 5
        if not self.mode_call(True):
            print("MODE_ENABLE_FAIL")
            return 6
        y0 = self.yaw
        samples = []
        try:
            start = time.monotonic()
            while time.monotonic() - start < DUR:
                self.publish(V, W)
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.wheels is not None:
                    samples.append(list(self.wheels))
                time.sleep(0.03)
        finally:
            end = time.monotonic() + 0.8
            while time.monotonic() < end:
                self.publish(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.03)
            self.mode_call(False)
        y1 = self.yaw
        dyaw = wrap_deg(y1 - y0)
        if samples:
            fl = sum(s[0] for s in samples) / len(samples)
            rl = sum(s[1] for s in samples) / len(samples)
            fr = sum(s[2] for s in samples) / len(samples)
            rr = sum(s[3] for s in samples) / len(samples)
        else:
            fl = rl = fr = rr = 0.0
        print("RESULT yaw0=%.2f yaw1=%.2f dyaw=%+.2f deg" % (y0, y1, dyaw))
        print("WHEELS fl=%.2f rl=%.2f fr=%.2f rr=%.2f (left_avg=%.2f right_avg=%.2f)"
              % (fl, rl, fr, rr, (fl + rl) / 2.0, (fr + rr) / 2.0))
        print("解读: 左轮往前(+)/右轮往后(-) => 顺时针; 左(-)/右(+) => 逆时针")
        return 0


def main():
    rclpy.init()
    node = TurnProbe()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
