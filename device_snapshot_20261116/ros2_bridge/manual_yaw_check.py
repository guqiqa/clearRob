#!/usr/bin/env python3
"""手动 90° 转体测试：验证 /odom yaw 符号（ROS 约定：逆时针/向左 = yaw 增大）。

不需要任何电机/速度指令。用法:
    python3 manual_yaw_check.py [DURATION_S]

步骤：运行后 DURATION_S 秒内，把整车【从上方看逆时针（向左）】转约 90°，然后保持不动。
输出 YAW_START / YAW_END / DELTA / RAW_SUM / VERDICT。
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

RAW_PATH = "/sys/bus/iio/devices/iio:device0/in_anglvel_z_raw"
DEFAULT_DURATION = 25.0


def read_raw() -> float:
    try:
        with open(RAW_PATH) as handle:
            return float(handle.read().strip())
    except Exception:
        return 0.0


def wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION
    rclpy.init()
    node = rclpy.create_node("manual_yaw_check")
    state = {"yaw": None}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        state["yaw"] = math.degrees(2.0 * math.atan2(q.z, q.w))

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    print("=== 手动 yaw 符号测试 ===", flush=True)
    print("现在开始 %.0f 秒窗口：请把整车【从上方看逆时针（向左）】转约 90°，然后停住不动。"
          % duration, flush=True)
    samples = []
    start = time.monotonic()
    next_print = start + 2.0
    while time.monotonic() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        raw = read_raw()
        if state["yaw"] is not None:
            samples.append((time.monotonic() - start, raw, state["yaw"]))
        now = time.monotonic()
        if now >= next_print:
            if samples:
                print("  t=%4.1fs yaw=%+7.2f° raw=%+6.0f"
                      % (samples[-1][0], samples[-1][2], raw), flush=True)
            next_print += 2.0
        time.sleep(0.05)

    if not samples:
        print("NO_ODOM：没有收到 /odom，确认 v4 在运行")
        return 1

    y0 = samples[0][2]
    y1 = samples[-1][2]
    delta = wrap_deg(y1 - y0)
    yaws = [s[2] for s in samples]
    print("--- summary ---")
    print("samples=%d duration=%.1fs" % (len(samples), samples[-1][0] - samples[0][0]))
    print("YAW_START=%.2f YAW_END=%.2f DELTA=%+.2f deg" % (y0, y1, delta))
    print("YAW_MIN=%.2f YAW_MAX=%.2f RAW_SUM=%.0f"
          % (min(yaws), max(yaws), sum(s[1] for s in samples)))
    if delta > 20.0:
        print("VERDICT=IMU_YAW_SIGN_OK (逆时针转体 -> /odom yaw 增大 %.1f deg)" % delta)
    elif delta < -20.0:
        print("VERDICT=IMU_YAW_SIGN_REVERSED (逆时针转体 -> /odom yaw 减小 %.1f deg；需翻转 IMU_Yaw_Sign)" % delta)
    else:
        print("VERDICT=INCONCLUSIVE (yaw 只变化 %+.1f deg；确认转体方向/幅度，或 IMU 未更新)" % delta)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
