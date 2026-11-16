#!/usr/bin/env python3
"""方向符号验证：先原地 w=+0.30（应左转/逆时针），再前进 v=0.08（应朝车头）。

观察要点：
1) 原地段：车应【向左/逆时针】转；左轮向后滚、右轮向前滚；odom yaw 增大。
2) 直行段：车应朝【车头方向】前进；odom x 增大。
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from navigation_interfaces.msg import ChassisState, SafeMotionCommand


def wrap_deg(v):
    while v > 180.0:
        v -= 360.0
    while v < -180.0:
        v += 360.0
    return v


class SignVerify(Node):
    def __init__(self):
        super().__init__("sign_verify")
        self.x = self.y = self.yaw = None
        self.wheels = None
        self.estop = self.feedback = self.can_ok = None
        self.msg = SafeMotionCommand()
        self.msg.ttl_s = 0.10
        self.msg.enable = True
        self.msg.stop_required = False
        self.msg.emergency_stop = False
        self.msg.source = "sign_verify"
        self.msg.reason = "sign_verify"
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

    def settle(self, dur=0.8):
        end = time.monotonic() + dur
        while time.monotonic() < end:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def run_phase(self, v, w, dur):
        samples = []
        start = time.monotonic()
        while time.monotonic() - start < dur:
            self.publish(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.wheels is not None:
                samples.append(list(self.wheels))
            time.sleep(0.03)
        self.settle()
        if samples:
            fl = sum(s[0] for s in samples) / len(samples)
            rl = sum(s[1] for s in samples) / len(samples)
            fr = sum(s[2] for s in samples) / len(samples)
            rr = sum(s[3] for s in samples) / len(samples)
        else:
            fl = rl = fr = rr = 0.0
        return fl, rl, fr, rr

    def call_mode(self, data):
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
        print("PRECHECK estop=%s feedback=%s can=%s yaw=%.2f pos=(%.2f,%.2f)"
              % (self.estop, self.feedback, self.can_ok, self.yaw, self.x, self.y))
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
        if not self.call_mode(True):
            print("MODE_ENABLE_FAIL")
            return 6
        try:
            yaw0 = self.yaw
            fl, rl, fr, rr = self.run_phase(0.0005, 0.30, 2.0)
            yaw1 = self.yaw
            print("ROTATE w=+0.30  yaw0=%.2f yaw1=%.2f dyaw=%+.2f deg  "
                  "wheels fl=%.2f rl=%.2f fr=%.2f rr=%.2f"
                  % (yaw0, yaw1, wrap_deg(yaw1 - yaw0), fl, rl, fr, rr), flush=True)
            print("  期望: dyaw>0（yaw 增大）；车实际向左/逆时针转", flush=True)
            x0 = self.x
            fl, rl, fr, rr = self.run_phase(0.08, 0.0, 1.5)
            x1 = self.x
            print("FORWARD v=+0.08  x0=%.3f x1=%.3f dx=%+.3f m  "
                  "wheels fl=%.2f rl=%.2f fr=%.2f rr=%.2f"
                  % (x0, x1, x1 - x0, fl, rl, fr, rr), flush=True)
            print("  期望: dx>0；车实际朝车头方向前进", flush=True)
        finally:
            self.settle(1.0)
            self.call_mode(False)
        print("=== DONE ===")
        return 0


def main():
    rclpy.init()
    node = SignVerify()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
