#!/usr/bin/env python3
"""实验二：脚本内单独实现 IMU 航向 PID，A 组 --pid off / B 组 --pid on 对比直行效果。"""

import sys

# ===== 实验参数区 =====
V_MPS = 0.10
DIST_M = 2.0
TIMEOUT_S = 15.0
KP = 1.2
KI = 0.0
KD = 0.1
U_MAX_RPM = 6.0
W_MAX = 0.30
# ===== 参数区结束 =====


def fmt_help():
    print("用法: python3 edu_heading_pid_lab.py [--pid on|off] [组号]")
    print("      python3 edu_heading_pid_lab.py --check")
    print("A 组: --pid off；B 组: --pid on")
    print("PID: u=-(KP*e+KI*int(e)-KD*yaw_rate)，u 限幅 %.1f RPM" % U_MAX_RPM)
    print("angular_z = -2*u/(B*k)，限幅 %.2f rad/s" % W_MAX)


def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        fmt_help()
        return 0
    if "--check" in args:
        print("V_MPS=%.3f DIST_M=%.3f TIMEOUT_S=%.1f" % (V_MPS, DIST_M, TIMEOUT_S))
        print("KP=%.3f KI=%.3f KD=%.3f U_MAX_RPM=%.1f W_MAX=%.2f"
              % (KP, KI, KD, U_MAX_RPM, W_MAX))
        print("A: --pid off (u=0)  B: --pid on")
        return 0

    pid_on = True
    label = "group"
    i = 0
    while i < len(args):
        if args[i] == "--pid":
            if i + 1 >= len(args):
                raise SystemExit("--pid 需要 on/off")
            pid_on = args[i + 1].lower() != "off"
            i += 2
        else:
            label = args[i]
            i += 1
    if V_MPS <= 0 or DIST_M <= 0 or TIMEOUT_S <= 0 or U_MAX_RPM < 0 or W_MAX <= 0:
        raise SystemExit("参数必须为正数")

    import math
    import re
    import time
    import rclpy
    import edu_lab_common as c
    from navigation_interfaces.msg import SafeMotionCommand

    tag = re.sub(r"[^A-Za-z0-9_.-]", "_", label)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    samples = []

    rclpy.init()
    node = c.LabNode()
    pub = None
    try:
        c.check_environment()
        node.wait_ready()
        node.check_no_other_publishers()
        print("PID=%s  V=%.3f m/s  目标距离=%.2f m" % ("ON" if pid_on else "OFF", V_MPS, DIST_M))
        c.confirm_teacher("教师确认本次 A/B 参数和场地条件")
        if not node.set_auto(True):
            raise RuntimeError("启用自主模式失败")
        pub = node.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
        time.sleep(0.5)

        target_yaw = None
        x0 = y0 = 0.0
        t0 = last_t = time.monotonic()
        integral = 0.0
        u = 0.0
        w = 0.0
        seq = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            od = node.odom
            if od is None:
                continue
            sample = c.sample_from_node(node)
            if sample is not None:
                samples.append(sample)
            q = od.pose.pose.orientation
            yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
            x = od.pose.pose.position.x
            y = od.pose.pose.position.y
            now = time.monotonic()
            if target_yaw is None:
                target_yaw = yaw
                x0, y0 = x, y
                t0 = last_t = now
                print("目标航向=%.2f 度" % target_yaw)
                continue
            dist = math.hypot(x - x0, y - y0)
            if dist >= DIST_M:
                break
            if now - t0 > TIMEOUT_S:
                print("超时，按当前距离停车")
                break
            if node.state is not None and node.state.emergency_stop:
                raise RuntimeError("底盘急停")
            dt = max(1e-3, now - last_t)
            last_t = now
            e = c.wrap_deg(target_yaw - yaw)
            integral += e * dt
            if KI > 0.0:
                ilim = U_MAX_RPM / KI
                integral = max(-ilim, min(ilim, integral))
            rate_dps = od.twist.twist.angular.z * 180.0 / math.pi
            if pid_on:
                u = -(KP * e + KI * integral - KD * rate_dps)
                u = max(-U_MAX_RPM, min(U_MAX_RPM, u))
            else:
                u = 0.0
            w = -2.0 * u / (c.TRACK * c.RPM_PER_MPS)
            w = max(-W_MAX, min(W_MAX, w))

            msg = SafeMotionCommand()
            msg.header.stamp = node.get_clock().now().to_msg()
            seq = max(seq + 1, time.monotonic_ns() // 1000)
            msg.sequence = seq
            msg.ttl_s = 0.10
            msg.linear_x = V_MPS
            msg.angular_z = w
            msg.enable = True
            msg.stop_required = False
            msg.emergency_stop = False
            msg.source = "edu_heading_pid_lab"
            msg.reason = "edu_heading_pid"
            pub.publish(msg)
            time.sleep(0.05)

        # 先连续零速，再退出自主模式
        zero = SafeMotionCommand()
        zero.header.stamp = node.get_clock().now().to_msg()
        zero.ttl_s = 0.10
        zero.linear_x = 0.0
        zero.angular_z = 0.0
        zero.enable = True
        zero.stop_required = False
        zero.emergency_stop = False
        zero.source = "edu_heading_pid_lab"
        zero.reason = "stop"
        for _ in range(12):
            zero.sequence = max(seq + 1, time.monotonic_ns() // 1000)
            seq = zero.sequence
            zero.header.stamp = node.get_clock().now().to_msg()
            pub.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)

        node.set_auto(False)
        m = c.metrics_from_rows(samples)
        lines = [
            "=== 实验二结果 %s PID=%s %s ===" % (tag, "ON" if pid_on else "OFF", stamp),
            "速度=%.3f m/s 目标距离=%.2f m" % (V_MPS, DIST_M),
            "实际距离=%.3f m 累积转角=%+.2f 度 漂移率=%+.2f 度/m"
            % (m["dist"], m["dyaw"], m["drift"]),
            "横向偏差=%.3f m 左右轮差=%+.2f RPM 航向过零=%d 次"
            % (m["lateral"], m["diff"], m["zero_cross"]),
        ]
        print("\n".join(lines))
        return 0
    except KeyboardInterrupt:
        print("已中断，执行零速清理")
        return 1
    finally:
        try:
            if pub is not None:
                z = SafeMotionCommand()
                z.ttl_s = 0.10
                z.linear_x = 0.0
                z.angular_z = 0.0
                z.enable = True
                z.stop_required = False
                z.emergency_stop = False
                z.source = "edu_heading_pid_lab"
                z.reason = "cleanup"
                z.sequence = time.monotonic_ns() // 1000
                pub.publish(z)
        except Exception:
            pass
        c.force_manual()
        try:
            node.set_auto(False)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
