#!/usr/bin/env python3
"""v4 安全校验行为测试：零半径旋转拒绝、超限拒绝、命令超时锁存与清除（轮子离地，不产生运动指令）。"""
import sys, time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from navigation_interfaces.msg import SafeMotionCommand, ChassisState
from nav_msgs.msg import Odometry

PASS = []; FAIL = []
def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(("PASS " if ok else "FAIL ") + name + ((" | " + detail) if detail else ""), flush=True)

class T(Node):
    def __init__(self):
        super().__init__("v4_rejection_test")
        self.pub = self.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
        self.odom = None
        self.state = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(ChassisState, "/chassis/state", self._state, 10)
    def _odom(self, m): self.odom = m
    def _state(self, m): self.state = m
    def seq(self): return time.monotonic_ns() // 1000
    def send(self, lin, ang):
        m = SafeMotionCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.sequence = self.seq(); m.ttl_s = 0.10
        m.linear_x = lin; m.angular_z = ang
        m.enable = True; m.stop_required = False; m.emergency_stop = False
        m.source = "rejection_test"; m.reason = "test"
        self.pub.publish(m)
    def spin_for(self, seconds, lin=None, ang=None):
        end = time.time() + seconds
        while time.time() < end:
            if lin is not None: self.send(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.05)
    def call(self, srv, value):
        cli = self.create_client(SetBool, srv)
        if not cli.wait_for_service(timeout_sec=2.0): return None
        r = SetBool.Request(); r.data = value
        f = cli.call_async(r); rclpy.spin_until_future_complete(self, f, timeout_sec=2.0)
        return f.result()
    def pose(self):
        if not self.odom: return None
        p = self.odom.pose.pose.position
        return (p.x, p.y)

def main():
    rclpy.init()
    t = T(); time.sleep(0.8)
    for _ in range(20): rclpy.spin_once(t, timeout_sec=0.05)
    r = t.call("/chassis/set_autonomous_mode", True)
    check("启用 chassis 自主模式", r is not None and r.success, str(r))
    t.spin_for(0.5)
    p0 = t.pose()

    t.spin_for(1.2, 0.0, 0.10)
    p1 = t.pose()
    d1 = ((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2) ** 0.5 if p0 and p1 else -1
    check("零半径旋转被拒绝(无运动)", 0 <= d1 < 0.01, "odom_delta=%.5f" % d1)
    check("零半径旋转未锁存", t.state is not None and t.state.fault_reason not in ("SAFE_COMMAND_OVER_LIMIT", "ZERO_RADIUS_ROTATION_REJECTED"), "fault=%r" % (t.state.fault_reason if t.state else None))

    t.spin_for(1.2, 0.20, 0.0)
    p2 = t.pose()
    d2 = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) ** 0.5 if p1 and p2 else -1
    check("超限线速度被拒绝(无运动)", 0 <= d2 < 0.01, "odom_delta=%.5f" % d2)
    check("超限命令未锁存", t.state is not None and t.state.fault_reason not in ("SAFE_COMMAND_OVER_LIMIT", "OVER_SPEED_LIMIT"), "fault=%r" % (t.state.fault_reason if t.state else None))

    t.spin_for(0.3, 0.0, 0.0)
    t.spin_for(1.2)  # silence -> TTL latch
    st = t.state
    check("命令超时锁存", st is not None and st.fault_reason == "SAFE_COMMAND_TIMEOUT", "fault=%r" % (st.fault_reason if st else None))
    check("超时锁存后运动不允许", st is not None and not st.motion_allowed, "allow=%r" % (st.motion_allowed if st else None))

    t.call("/chassis/set_autonomous_mode", False)
    t.call("/chassis/set_autonomous_mode", True)
    t.spin_for(0.6)
    st2 = t.state
    check("关→开自主模式清除超时锁存", st2 is not None and st2.fault_reason != "SAFE_COMMAND_TIMEOUT", "fault=%r" % (st2.fault_reason if st2 else None))
    t.call("/chassis/set_autonomous_mode", False)

    print("\nREJECTION RESULT: PASS=%d FAIL=%d" % (len(PASS), len(FAIL)), flush=True)
    t.destroy_node(); rclpy.shutdown()
    return 1 if FAIL else 0

if __name__ == "__main__":
    sys.exit(main())
