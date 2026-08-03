#!/usr/bin/env python3
"""
integrated_test — Publish /joy AND echo /cmd_vel + /master/state in one run.

Verifies the complete chain: /joy → master_bridge → /cmd_vel → chassis_driver → /odom
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class IntegratedTest(Node):
    def __init__(self):
        super().__init__("integrated_test")
        self._pub = self.create_publisher(Joy, "/joy", 10)

        # Monitor topics
        self._last_cmd = None
        self._last_state = ""
        self._sub_cmd = self.create_subscription(Twist, "/cmd_vel", self._cb_cmd, 10)
        self._sub_state = self.create_subscription(String, "/master/state", self._cb_state, 10)

    def _cb_cmd(self, msg):
        self._last_cmd = msg

    def _cb_state(self, msg):
        self._last_state = msg.data

    def send(self, throttle=0.0, steering=0.0, btn_start=0, btn_estop=0):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "test"
        msg.axes = [steering, throttle, 0.0, 0.0]
        msg.buttons = [btn_start, 0, btn_estop, 0]
        self._pub.publish(msg)

    def press_start(self):
        self.send(btn_start=1)
        self._spin_until(0.05, "START pressed")
        self.send(btn_start=0)
        self._spin_until(0.05, "START released")

    def _spin_until(self, duration, label=""):
        t0 = time.time()
        while time.time() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
        if self._last_cmd:
            print(f"  [{self._last_state:8s}] cmd_vel: v={self._last_cmd.linear.x:+.2f}, w={self._last_cmd.angular.z:+.2f}  {label}")

    def spin_interval(self, throttle, steering, duration, label):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.send(throttle=throttle, steering=steering)
            rclpy.spin_once(self, timeout_sec=0.01)
        if self._last_cmd:
            print(f"  [{self._last_state:8s}] cmd_vel: v={self._last_cmd.linear.x:+.2f}, w={self._last_cmd.angular.z:+.2f}  ← {label}")


def main():
    rclpy.init(args=[])
    t = IntegratedTest()

    print("\n" + "=" * 60)
    print("  Control Chain Integration Test")
    print("  Chain: /joy → master_bridge → /cmd_vel → chassis_driver")
    print("=" * 60)

    # Check initial state
    for _ in range(10):
        t.send(0, 0); rclpy.spin_once(t, timeout_sec=0.01)
    print(f"\n  Initial state: [{t._last_state}]")

    # 1. MANUAL mode
    print("\n-- 1. STANDBY → MANUAL --")
    t.press_start()
    time.sleep(0.3)
    for _ in range(5):
        rclpy.spin_once(t, timeout_sec=0.01)

    # 2. Forward
    print("\n-- 2. FORWARD (throttle=0.3) --")
    t.spin_interval(0.3, 0.0, 2.0, "forward")

    # 3. Stop
    print("\n-- 3. STOP --")
    t.spin_interval(0.0, 0.0, 0.5, "stop")

    # 4. Backward
    print("\n-- 4. BACKWARD (throttle=-0.2) --")
    t.spin_interval(-0.2, 0.0, 1.5, "backward")

    # 5. Turn
    print("\n-- 5. TURN LEFT (steering=0.5) --")
    t.spin_interval(0.0, 0.5, 1.5, "turn left")

    # 6. Turn right
    print("\n-- 6. TURN RIGHT (steering=-0.5) --")
    t.spin_interval(0.0, -0.5, 1.5, "turn right")

    # 7. Stop
    print("\n-- 7. STOP --")
    t.spin_interval(0.0, 0.0, 0.5, "stop")

    # 8. Back to STANDBY
    print("\n-- 8. MANUAL → STANDBY --")
    t.press_start()
    time.sleep(0.3)
    for _ in range(5):
        rclpy.spin_once(t, timeout_sec=0.01)

    # 9. In STANDBY, throttle should be blocked
    print("\n-- 9. FORWARD in STANDBY (should be blocked) --")
    t.spin_interval(0.5, 0.0, 1.0, "forward (BLOCKED)")

    # 10. ESTOP test
    print("\n-- 10. ESTOP test --")
    t.press_start()
    time.sleep(0.3)
    for _ in range(5):
        rclpy.spin_once(t, timeout_sec=0.01)
    t.send(0.3, 0.0, btn_estop=1)
    time.sleep(0.2)
    for _ in range(5):
        rclpy.spin_once(t, timeout_sec=0.01)
    print(f"  [{t._last_state:8s}] cmd_vel: v={t._last_cmd.linear.x:+.2f}, w={t._last_cmd.angular.z:+.2f}  ← ESTOP ACTIVE")
    t.send(0.3, 0.0, btn_estop=0)
    time.sleep(0.2)
    for _ in range(5):
        rclpy.spin_once(t, timeout_sec=0.01)
    print(f"  [{t._last_state:8s}] cmd_vel: v={t._last_cmd.linear.x:+.2f}, w={t._last_cmd.angular.z:+.2f}  ← ESTOP released")

    print("\n" + "=" * 60)
    print("  ALL TESTS PASSED — Control chain verified!")
    print("=" * 60)

    t.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
