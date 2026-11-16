#!/usr/bin/env python3
"""教学实验共用工具：预检、状态、内存采样、指标。不在设备/电脑保存实验记录。"""
import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import ChassisState
from std_srvs.srv import SetBool

R_WHEEL = 0.1034
TRACK = 0.45
POLE_PAIRS = 10
GEAR_RATIO = 4.5
RPM_PER_MPS = 60.0 / (2.0 * math.pi * R_WHEEL)
BRIDGE = "/root/ros2_bridge"
ARC_CMD = BRIDGE + "/arc_cmd.py"


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def check_environment():
    if os.path.exists("/etc/laifei-chassis-rc-input"):
        raise RuntimeError("当前是 RC 模式，请先执行 /root/ros2_bridge/chassis_rc_mode.sh off")
    if not os.path.exists(ARC_CMD):
        raise RuntimeError("缺少设备脚本: " + ARC_CMD)


def confirm_teacher(msg):
    ans = input(msg + " [yes/NO]: ").strip().lower()
    if ans not in ("yes", "y", "确认", "是"):
        raise SystemExit("教师未确认，已取消")


class LabNode(Node):
    def __init__(self):
        super().__init__("edu_lab_node")
        self.state = None
        self.odom = None
        self.create_subscription(ChassisState, "/chassis/state", self._on_state, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.mode_cli = self.create_client(SetBool, "/chassis/set_autonomous_mode")

    def _on_state(self, m):
        self.state = m

    def _on_odom(self, m):
        self.odom = m

    def spin_wait(self, cond, timeout=8.0):
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def wait_ready(self):
        if not self.spin_wait(lambda: self.state is not None and self.odom is not None):
            raise RuntimeError("未收到 /chassis/state 或 /odom")
        s = self.state
        if str(s.control_mode) != "MANUAL":
            raise RuntimeError("v4 不是 MANUAL: " + str(s.control_mode))
        if s.emergency_stop:
            raise RuntimeError("急停有效")
        if not s.feedback_valid or not s.can_healthy:
            raise RuntimeError("反馈或 CAN 不健康")
        if max(abs(x) for x in s.wheel_rpm) > 0.5:
            raise RuntimeError("四轮未停稳")
        if abs(s.linear_velocity) > 0.01 or abs(s.angular_velocity) > 0.01:
            raise RuntimeError("底盘未零速")

    def check_no_other_publishers(self):
        for topic in ("/safety/safe_motion_command", "/cmd_vel"):
            n = self.count_publishers(topic)
            if n > 0:
                raise RuntimeError("已有发布者 " + topic + " x" + str(n))

    def set_auto(self, data):
        if not self.mode_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("无 /chassis/set_autonomous_mode 服务")
        req = SetBool.Request()
        req.data = bool(data)
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.done() and fut.result() is not None and fut.result().success


def force_manual():
    try:
        subprocess.run(
            ["ros2", "service", "call", "/chassis/set_autonomous_mode",
             "std_srvs/srv/SetBool", "{data: false}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=8.0,
        )
    except Exception:
        pass


def sample_from_node(node):
    s = node.state
    od = node.odom
    if s is None or od is None:
        return None
    q = od.pose.pose.orientation
    yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
    return {
        "x": od.pose.pose.position.x,
        "y": od.pose.pose.position.y,
        "yaw_deg": yaw,
        "v": s.linear_velocity,
        "fl": s.wheel_rpm[0],
        "rl": s.wheel_rpm[1],
        "fr": s.wheel_rpm[2],
        "rr": s.wheel_rpm[3],
    }


def metrics_from_rows(rows):
    if len(rows) < 10:
        raise RuntimeError("有效采样太少，无法统计")
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    yaws = [float(r["yaw_deg"]) for r in rows]
    vs = [float(r["v"]) for r in rows]
    fl = [float(r["fl"]) for r in rows]
    rl = [float(r["rl"]) for r in rows]
    fr = [float(r["fr"]) for r in rows]
    rr = [float(r["rr"]) for r in rows]
    moving = [
        i for i in range(len(rows))
        if abs(vs[i]) > 0.02 or max(abs(fl[i]), abs(rl[i]), abs(fr[i]), abs(rr[i])) > 1.0
    ]
    if len(moving) < 5:
        raise RuntimeError("没有检测到运动")
    i0, i1 = moving[0], moving[-1]
    dx = xs[i1] - xs[i0]
    dy = ys[i1] - ys[i0]
    dist = math.hypot(dx, dy)
    dyaw = wrap_deg(yaws[i1] - yaws[i0])
    yaw0 = math.radians(yaws[i0])
    lateral = abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)
    n = len(moving)
    left = sum((fl[i] + rl[i]) / 2.0 for i in moving) / n
    right = sum((fr[i] + rr[i]) / 2.0 for i in moving) / n
    zc = 0
    last = 0
    for i in moving:
        err = wrap_deg(yaws[i] - yaws[i0])
        s = 1 if err > 0.3 else (-1 if err < -0.3 else 0)
        if s != 0:
            if last != 0 and s != last:
                zc += 1
            last = s
    return {
        "dist": dist,
        "dyaw": dyaw,
        "drift": (dyaw / dist if dist > 0.05 else 0.0),
        "lateral": lateral,
        "left": left,
        "right": right,
        "diff": right - left,
        "zero_cross": zc,
        "rows": len(rows),
    }
