#!/usr/bin/env python3
"""实验一：四轮目标 RPM -> 左右轮组 -> v/omega，通过正式 SafeMotionCommand 链路离地运行。"""

# ===== 学生参数区：只修改以下 6 行 =====
RUN_LABEL = "group01_A"
FL_RPM = 6
RL_RPM = 6
FR_RPM = 6
RR_RPM = 6
DURATION_S = 4
# ===== 参数区结束 =====

import math
import re
import subprocess
import sys

R_WHEEL = 0.1034
TRACK = 0.45
POLE_PAIRS = 10
GEAR_RATIO = 4.5
MAX_RPM = 10.0
SAME_SIDE_TOL = 0.5
V_MAX = 0.15
W_MAX = 0.60
ROTATE_EPS = 0.0005


def theory(fl, rl, fr, rr):
    n_l = (fl + rl) / 2.0
    n_r = (fr + rr) / 2.0
    v_l = 2.0 * math.pi * R_WHEEL * n_l / 60.0
    v_r = 2.0 * math.pi * R_WHEEL * n_r / 60.0
    v = (v_l + v_r) / 2.0
    w = (v_r - v_l) / TRACK
    return n_l, n_r, v, w


def check_params(fl, rl, fr, rr):
    for name, rpm in (("FL", fl), ("RL", rl), ("FR", fr), ("RR", rr)):
        if abs(rpm) > MAX_RPM:
            raise SystemExit("%s_RPM 超过教学上限 %.1f" % (name, MAX_RPM))
    if abs(fl - rl) > SAME_SIDE_TOL or abs(fr - rr) > SAME_SIDE_TOL:
        raise SystemExit("同侧前后轮目标差超过 %.1f RPM，拒绝运行" % SAME_SIDE_TOL)
    _, _, v, w = theory(fl, rl, fr, rr)
    if abs(v) > V_MAX or abs(w) > W_MAX:
        raise SystemExit("理论 v/omega 超限: v=%.3f omega=%.3f" % (v, w))
    if abs(v) < 1e-4 and abs(w) < 1e-4:
        raise SystemExit("四个 RPM 都为 0，没有运动")
    if DURATION_S <= 0 or DURATION_S > 5.0:
        raise SystemExit("DURATION_S 必须在 0~5 s 内")
    return v, w


def main():
    check_only = "--check" in sys.argv
    v, w = check_params(FL_RPM, RL_RPM, FR_RPM, RR_RPM)
    n_l, n_r, _, _ = theory(FL_RPM, RL_RPM, FR_RPM, RR_RPM)
    if check_only:
        print("=== CHECK %s ===" % RUN_LABEL)
        print("目标 RPM: FL=%.1f RL=%.1f FR=%.1f RR=%.1f" % (FL_RPM, RL_RPM, FR_RPM, RR_RPM))
        print("轮组: n_L=%.3f n_R=%.3f RPM" % (n_l, n_r))
        print("理论: v=%.4f m/s omega=%.4f rad/s" % (v, w))
        print("eRPM: FL=%.1f RL=%.1f FR=%.1f RR=%.1f"
              % (FL_RPM * POLE_PAIRS * GEAR_RATIO, RL_RPM * POLE_PAIRS * GEAR_RATIO,
                 FR_RPM * POLE_PAIRS * GEAR_RATIO, RR_RPM * POLE_PAIRS * GEAR_RATIO))
        return 0

    import time
    import rclpy
    import edu_lab_common as c

    stamp = time.strftime("%Y%m%d_%H%M%S")
    linear_pub = v if abs(v) >= 1e-4 else ROTATE_EPS
    samples = []

    rclpy.init()
    node = c.LabNode()
    arc = None
    try:
        c.check_environment()
        node.wait_ready()
        node.check_no_other_publishers()
        print("工况 %s: n_L=%.3f n_R=%.3f v=%.4f omega=%.4f" % (RUN_LABEL, n_l, n_r, v, w))
        if abs(v) < 1e-4:
            print("注意：原地旋转 v4 会拒绝严格 v=0，内部用 %.4f m/s 触发 rotate 分支" % ROTATE_EPS)
        c.confirm_teacher("教师确认本次参数和离地条件")
        if not node.set_auto(True):
            raise RuntimeError("启用自主模式失败")
        arc = subprocess.Popen(
            ["python3", c.ARC_CMD, "%.6f" % linear_pub, "%.6f" % w, "%.2f" % DURATION_S],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t_start = time.monotonic()
        while time.monotonic() - t_start < DURATION_S + 0.8 and arc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            sample = c.sample_from_node(node)
            if sample is not None:
                samples.append(sample)
            time.sleep(0.02)
        try:
            arc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            arc.kill()
        node.set_auto(False)
        m = c.metrics_from_rows(samples)
        v_l = 2.0 * math.pi * R_WHEEL * m["left"] / 60.0
        v_r = 2.0 * math.pi * R_WHEEL * m["right"] / 60.0
        v_fb = (v_l + v_r) / 2.0
        w_fb = (v_r - v_l) / TRACK
        lines = [
            "=== 实验一结果 %s %s ===" % (RUN_LABEL, stamp),
            "目标 RPM: FL=%.1f RL=%.1f FR=%.1f RR=%.1f" % (FL_RPM, RL_RPM, FR_RPM, RR_RPM),
            "理论: n_L=%.3f n_R=%.3f v=%.4f omega=%.4f" % (n_l, n_r, v, w),
            "反馈: 左=%.2f 右=%.2f RPM  v=%.4f omega=%.4f" % (m["left"], m["right"], v_fb, w_fb),
            "反馈/位移: dist=%.3f m dyaw=%+.2f deg 左右轮差=%+.2f RPM"
            % (m["dist"], m["dyaw"], m["diff"]),
        ]
        print("\n".join(lines))
        return 0
    except KeyboardInterrupt:
        print("已中断，执行零速清理")
        return 1
    finally:
        try:
            if arc is not None and arc.poll() is None:
                arc.kill()
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
