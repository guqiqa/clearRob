#!/usr/bin/env python3
"""发布 SafeMotionCommand 弧线指令，用于转向符号/原地转向测试。

用法:
    arc_cmd.py LINEAR_MPS ANGULAR_RADPS [DURATION_S] [SOURCE]

- 20 Hz 持续发布（ttl_s=0.10），sequence = monotonic_ns//1000，跨进程严格单调；
- duration 结束后再发 0.6 s 零速收尾，避免 v4 立即锁存 SAFE_COMMAND_TIMEOUT；
- 仅用于台架/有人监护的地面测试，绕过 safety_supervisor 直接写
  /safety/safe_motion_command；速度上限与 config.xml 保持一致。
"""
import sys
import time

import rclpy
from navigation_interfaces.msg import SafeMotionCommand

MAX_LINEAR_MPS = 0.15     # config.xml ROS_Max_Linear_mps
MAX_ANGULAR_RADPS = 0.60  # config.xml ROS_Max_Angular_radps (2026-10-25 起)


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: arc_cmd.py LINEAR_MPS ANGULAR_RADPS [DURATION_S] [SOURCE]")
        return 2
    linear = float(sys.argv[1])
    angular = float(sys.argv[2])
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    source = sys.argv[4] if len(sys.argv) > 4 else "turn_sign_arc"

    if abs(linear) > MAX_LINEAR_MPS + 1e-9 or abs(angular) > MAX_ANGULAR_RADPS + 1e-9:
        print("REFUSED: over limit (|v|<=%.2f m/s, |w|<=%.2f rad/s)"
              % (MAX_LINEAR_MPS, MAX_ANGULAR_RADPS))
        return 2
    if abs(linear) < 1e-6 and abs(angular) > 1e-3:
        print("REFUSED: v4 rejects zero-radius rotation (v=0, w!=0)")
        return 2

    rclpy.init()
    node = rclpy.create_node("arc_cmd_pub")
    pub = node.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
    msg = SafeMotionCommand()
    msg.ttl_s = 0.10
    msg.enable = True
    msg.stop_required = False
    msg.emergency_stop = False
    msg.source = source
    msg.reason = "rotate_authority_test"

    def publish(v: float, w: float) -> None:
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.sequence = time.monotonic_ns() // 1000
        msg.linear_x = float(v)
        msg.angular_z = float(w)
        pub.publish(msg)

    print("ARC_START v=%.4f w=%.3f duration=%.2f source=%s"
          % (linear, angular, duration, source), flush=True)
    start = time.monotonic()
    while time.monotonic() - start < duration:
        publish(linear, angular)
        time.sleep(0.05)
    stop_start = time.monotonic()
    while time.monotonic() - stop_start < 0.6:
        publish(0.0, 0.0)
        time.sleep(0.05)
    print("ARC_DONE v=%.4f w=%.3f" % (linear, angular), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
