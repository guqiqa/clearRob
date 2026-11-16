#!/usr/bin/env python3
"""室内 2 m x 1 m 顺时针矩形运动（闭环：odom 距离 + IMU yaw）。

顺序：
  前进 LONG -> 右转 90° -> 前进 SHORT -> 右转 90°
  -> 前进 LONG -> 右转 90° -> 前进 SHORT -> 右转 90° 归位

- 前进：v=0.08 m/s，用 IMU yaw 做小航向保持；
- 右转：v=0.0005（触发 v4 rotate 分支，15A）、w=-0.45，闭环到 -90°；
- 直接发布 /safety/safe_motion_command（绕过 safety_supervisor，和之前测试一致）；
- 每次运动前检查急停/反馈/CAN；Ctrl-C 或异常会自动零速并关闭自主模式。

用法：
    source /root/nav_ws/env.sh
    python3 /root/ros2_bridge/rect_cw_2x1.py [LONG] [SHORT] [V_FWD]

空间要求：前后 >= LONG+0.6 m，左右 >= SHORT+0.7 m（含车体宽度），人持急停。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from navigation_interfaces.msg import ChassisState, SafeMotionCommand

LONG = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
SHORT = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
V_FWD = float(sys.argv[3]) if len(sys.argv) > 3 else 0.08
V_TURN = 0.0005
W_TURN = -0.45
FWD_TOL = 0.02
TURN_TARGET = -90.0
TURN_TOL = 2.0
HOLD_KP = 0.03
MAX_W_HOLD = 0.15
SETTLE_S = 0.8


def wrap_deg(value):
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


class RectCW(Node):
    def __init__(self):
        super().__init__("rect_cw_2x1")
        self.x = None
        self.y = None
        self.yaw = None
        self.state_w = None
        self.estop = None
        self.feedback = None
        self.can_ok = None
        self.pub = None
        self.msg = SafeMotionCommand()
        self.msg.ttl_s = 0.10
        self.msg.enable = True
        self.msg.stop_required = False
        self.msg.emergency_stop = False
        self.msg.source = "rect_cw_2x1"
        self.msg.reason = "rect_cw_2x1"
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
        self.estop = bool(msg.emergency_stop)
        self.feedback = bool(msg.feedback_valid)
        self.can_ok = bool(msg.can_healthy)

    def publish(self, v, w):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.sequence = time.monotonic_ns() // 1000
        self.msg.linear_x = float(v)
        self.msg.angular_z = float(w)
        self.pub.publish(self.msg)

    def check_safety(self):
        if self.estop or not self.feedback or not self.can_ok:
            raise RuntimeError("safety: estop=%s feedback=%s can=%s"
                               % (self.estop, self.feedback, self.can_ok))

    def settle(self, duration=SETTLE_S):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def forward(self, length):
        x0, y0, yaw0 = self.x, self.y, self.yaw
        t0 = time.monotonic()
        timeout = length / max(V_FWD, 1e-3) * 3.0 + 10.0
        dist = 0.0
        while True:
            err = wrap_deg(self.yaw - yaw0)
            w = max(-MAX_W_HOLD, min(MAX_W_HOLD, -HOLD_KP * err))
            self.publish(V_FWD, w)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.check_safety()
            dist = math.hypot(self.x - x0, self.y - y0)
            if dist >= length - FWD_TOL:
                break
            if time.monotonic() - t0 > timeout:
                raise RuntimeError("forward timeout length=%.2f dist=%.3f" % (length, dist))
            time.sleep(0.03)
        self.settle()
        return dist, wrap_deg(self.yaw - yaw0)

    def turn_right(self):
        yaw0 = self.yaw
        t0 = time.monotonic()
        last_log = t0
        timeout = 45.0
        dyaw = 0.0
        while True:
            self.publish(V_TURN, W_TURN)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.check_safety()
            dyaw = wrap_deg(self.yaw - yaw0)
            now = time.monotonic()
            if now - last_log >= 1.0:
                print("TURN dyaw=%+.1f" % dyaw, flush=True)
                last_log = now
            if dyaw <= TURN_TARGET + TURN_TOL:
                break
            if dyaw > 15.0:
                raise RuntimeError("turn direction wrong dyaw=%.1f" % dyaw)
            if now - t0 > 3.0 and abs(dyaw) < 1.0:
                raise RuntimeError("turn not responding after 3s dyaw=%.1f" % dyaw)
            if now - t0 > timeout:
                raise RuntimeError("turn timeout dyaw=%.1f" % dyaw)
            time.sleep(0.03)
        self.settle()
        return wrap_deg(self.yaw - yaw0)

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
            print("NO_SUBSCRIBER")
            return 6
        if not self.call_mode(True):
            print("MODE_ENABLE_FAIL")
            return 7
        print("MODE enabled; start pose x=%.3f y=%.3f yaw=%.2f" % (self.x, self.y, self.yaw))
        home = (self.x, self.y, self.yaw)
        ok = True
        try:
            d, dy = self.forward(LONG)
            print("LEG1 forward LONG done dist=%.3f dyaw=%+.2f" % (d, dy), flush=True)
            dy = self.turn_right()
            print("TURN1 right done dyaw=%+.2f" % dy, flush=True)
            d, dy = self.forward(SHORT)
            print("LEG2 forward SHORT done dist=%.3f dyaw=%+.2f" % (d, dy), flush=True)
            dy = self.turn_right()
            print("TURN2 right done dyaw=%+.2f" % dy, flush=True)
            d, dy = self.forward(LONG)
            print("LEG3 forward LONG done dist=%.3f dyaw=%+.2f" % (d, dy), flush=True)
            dy = self.turn_right()
            print("TURN3 right done dyaw=%+.2f" % dy, flush=True)
            d, dy = self.forward(SHORT)
            print("LEG4 forward SHORT done dist=%.3f dyaw=%+.2f" % (d, dy), flush=True)
            dy = self.turn_right()
            print("TURN4 right done dyaw=%+.2f" % dy, flush=True)
        except KeyboardInterrupt:
            print("INTERRUPTED")
            ok = False
        except RuntimeError as exc:
            print("ABORT: %s" % exc)
            ok = False
        finally:
            self.settle(1.0)
            self.call_mode(False)
        dx = self.x - home[0]
        dy = self.y - home[1]
        closure = math.hypot(dx, dy)
        yaw_err = wrap_deg(self.yaw - home[2])
        print("CLOSURE dx=%+.3f dy=%+.3f dist=%.3f yaw_err=%+.2f ok=%s"
              % (dx, dy, closure, yaw_err, ok))
        print("=== DONE ok=%s ===" % ok)
        return 0


def main():
    rclpy.init()
    node = RectCW()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
