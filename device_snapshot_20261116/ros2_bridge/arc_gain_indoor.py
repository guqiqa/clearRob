#!/usr/bin/env python3
"""室内 15A 弧线转向增益测试（单进程，短弧线，横向位移很小）。

特点：
- 一个进程持续发布 SafeMotionCommand（20Hz），避免多次启动造成的 DDS 发现延迟；
- 阶段间持续发零速保持安全链路，不会触发 SAFE_COMMAND_TIMEOUT 锁存；
- v=0.06 m/s，w=±0.15/±0.30/±0.60，短时 1.5~2.5 s，横向位移预计 < 0.2 m；
- 每段前后读 /odom yaw，段内统计 /chassis/state 角速度与四轮 rpm，
  输出 实际 yaw 率 / 轮速派生角速度（转向效率）。

用法:
    source /root/nav_ws/env.sh
    python3 /root/ros2_bridge/arc_gain_indoor.py

需要：室内前后约 1 m 空间、左右约 0.5 m，人持急停。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from navigation_interfaces.msg import ChassisState, SafeMotionCommand

V = 0.06
SETTLE_S = 1.2
PHASES = [
    ("P015", +0.15, 2.5),
    ("M015", -0.15, 2.5),
    ("P030", +0.30, 2.0),
    ("M030", -0.30, 2.0),
    ("P060", +0.60, 1.5),
    ("M060", -0.60, 1.5),
]
RAD2DEG = 57.29577951308232


def wrap_deg(value):
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


class ArcGainIndoor(Node):
    def __init__(self):
        super().__init__("arc_gain_indoor")
        self.yaw = None
        self.x = None
        self.y = None
        self.state_w = None
        self.wheels = None
        self.estop = None
        self.feedback = None
        self.can_ok = None
        self.pub = None
        self.msg = SafeMotionCommand()
        self.msg.ttl_s = 0.10
        self.msg.enable = True
        self.msg.stop_required = False
        self.msg.emergency_stop = False
        self.msg.source = "arc_gain_indoor"
        self.msg.reason = "arc_gain_indoor"
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(ChassisState, "/chassis/state", self.on_state, 10)
        self.mode_cli = self.create_client(SetBool, "/chassis/set_autonomous_mode")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def on_state(self, msg):
        self.state_w = float(msg.angular_velocity)
        self.wheels = [float(v) for v in msg.wheel_rpm]
        self.estop = bool(msg.emergency_stop)
        self.feedback = bool(msg.feedback_valid)
        self.can_ok = bool(msg.can_healthy)

    def publish(self, v, w):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.sequence = time.monotonic_ns() // 1000
        self.msg.linear_x = float(v)
        self.msg.angular_z = float(w)
        self.pub.publish(self.msg)

    def spin_for(self, duration, v=0.0, w=0.0):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def call_mode(self, data):
        if not self.mode_cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetBool.Request()
        req.data = bool(data)
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.done() and fut.result() is not None and fut.result().success)

    def run(self):
        t0 = time.monotonic()
        while (self.yaw is None or self.state_w is None) and time.monotonic() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.yaw is None or self.state_w is None:
            print("NO_ODOM_OR_STATE：确认 v4 在运行")
            return 2
        print("PRECHECK estop=%s feedback=%s can=%s yaw=%.2f"
              % (self.estop, self.feedback, self.can_ok, self.yaw))
        if self.estop or not self.feedback or not self.can_ok:
            print("PRECHECK_FAIL")
            return 3
        if self.count_publishers("/safety/safe_motion_command") > 0:
            print("STRAY_SAFE_CMD_PUBLISHER：先停掉 nav/safety/arc_cmd")
            return 4
        if self.count_publishers("/cmd_vel") > 0:
            print("STRAY_CMD_VEL_PUBLISHER")
            return 4
        if not self.mode_cli.wait_for_service(timeout_sec=5.0):
            print("NO_MODE_SERVICE")
            return 5
        self.pub = self.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
        t0 = time.monotonic()
        while self.pub.get_subscription_count() == 0 and time.monotonic() - t0 < 6.0:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        if self.pub.get_subscription_count() == 0:
            print("NO_SUBSCRIBER：v4 没有订阅 /safety/safe_motion_command")
            return 6
        if not self.call_mode(True):
            print("MODE_ENABLE_FAIL")
            return 7
        print("MODE enabled, starting phases (v=%.2f m/s)" % V)
        results = []
        interrupted = False
        try:
            for name, w_cmd, duration in PHASES:
                self.spin_for(SETTLE_S, 0.0, 0.0)
                if self.estop or not self.feedback or not self.can_ok:
                    raise RuntimeError("safety state changed: estop=%s feedback=%s can=%s"
                                       % (self.estop, self.feedback, self.can_ok))
                yaw0, x0, y0 = self.yaw, self.x, self.y
                ws = []
                rpms = []
                start = time.monotonic()
                while time.monotonic() - start < duration:
                    self.publish(V, w_cmd)
                    rclpy.spin_once(self, timeout_sec=0.02)
                    if self.state_w is not None:
                        ws.append(self.state_w)
                    if self.wheels is not None:
                        rpms.append(list(self.wheels))
                    time.sleep(0.03)
                tail_end = time.monotonic() + 0.6
                while time.monotonic() < tail_end:
                    self.publish(0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.02)
                    time.sleep(0.03)
                yaw1, x1, y1 = self.yaw, self.x, self.y
                dyaw = wrap_deg(yaw1 - yaw0)
                actual_rate = dyaw / duration
                expected_rad = sum(ws) / len(ws) if ws else 0.0
                expected_deg = expected_rad * RAD2DEG
                ratio = abs(actual_rate) / abs(expected_deg) if abs(expected_deg) > 1e-6 else 0.0
                disp = math.hypot(x1 - x0, y1 - y0) if x0 is not None else 0.0
                if rpms:
                    fl = sum(r[0] for r in rpms) / len(rpms)
                    rl = sum(r[1] for r in rpms) / len(rpms)
                    fr = sum(r[2] for r in rpms) / len(rpms)
                    rr = sum(r[3] for r in rpms) / len(rpms)
                else:
                    fl = rl = fr = rr = 0.0
                line = ("PHASE %s cmd_w=%+.2f dur=%.1fs yaw=%.2f->%.2f dyaw=%+.2fdeg "
                        "actual=%+.2fdeg/s expected=%+.2fdeg/s ratio=%.2f disp=%.3fm "
                        "fl=%.2f rl=%.2f fr=%.2f rr=%.2f"
                        % (name, w_cmd, duration, yaw0, yaw1, dyaw, actual_rate,
                           expected_deg, ratio, disp, fl, rl, fr, rr))
                print(line, flush=True)
                results.append(line)
        except KeyboardInterrupt:
            interrupted = True
            print("INTERRUPTED by Ctrl-C")
        except RuntimeError as exc:
            print("ABORT: %s" % exc)
        finally:
            self.spin_for(1.0, 0.0, 0.0)
            ok = self.call_mode(False)
            print("MODE disabled ok=%s" % ok)
        print("=== SUMMARY ===")
        for line in results:
            print(line)
        print("=== DONE interrupted=%s ===" % interrupted)
        return 0


def main():
    rclpy.init()
    node = ArcGainIndoor()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
