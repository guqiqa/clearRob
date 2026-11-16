#!/usr/bin/env python3
"""/chassis/state /odom /safe_motion_command 等接口的 v4 hardware 模式审计（不启动电机）。"""
import sys, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from navigation_interfaces.msg import SafeMotionCommand, ChassisState
from nav_msgs.msg import Odometry

PASS = []; FAIL = []

def check(name, ok, detail=""):
    (PASS if ok else FAIL).append((name, detail))
    print(("PASS " if ok else "FAIL ") + name + ((" | " + detail) if detail else ""), flush=True)

class Audit(Node):
    def __init__(self):
        super().__init__("v4_interface_audit")

    def pubs(self, name):
        try:
            return [i.node_name for i in self.get_publishers_info_by_topic(name)]
        except Exception as e:
            return ["ERR:" + str(e)]

    def sample(self, name, cls, duration):
        got = []
        sub = self.create_subscription(cls, name, lambda m: got.append(m), 10)
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        return got

    def call_bool(self, srv_name, value, timeout=2.0):
        cli = self.create_client(SetBool, srv_name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        req = SetBool.Request(); req.data = value
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()


def main():
    rclpy.init()
    n = Audit()
    time.sleep(1.0)
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.05)

    # 1) publishers & services
    ch_pubs = n.pubs("/chassis/state")
    odom_pubs = n.pubs("/odom")
    safe_pubs = n.pubs("/safety/safe_motion_command")
    cmd_pubs = n.pubs("/cmd_vel")
    check("/chassis/state 唯一发布者", [p.lstrip("/") for p in ch_pubs] == ["laifei_chassis_driver"], str(ch_pubs))
    check("/odom 唯一发布者", [p.lstrip("/") for p in odom_pubs] == ["laifei_chassis_driver"], str(odom_pubs))
    check("/safety/safe_motion_command 唯一发布者", [p.lstrip("/") for p in safe_pubs] == ["safety_supervisor"], str(safe_pubs))
    check("/cmd_vel 0 发布者", len(cmd_pubs) == 0, str(cmd_pubs))
    svcs = dict(n.get_service_names_and_types())
    for name, typ in (("/chassis/set_autonomous_mode", "std_srvs/srv/SetBool"),
                      ("/set_autonomous_mode", "std_srvs/srv/SetBool"),
                      ("/control/set_autonomous_mode", "std_srvs/srv/SetBool")):
        check("服务 " + name, name in svcs and typ in svcs[name], str(svcs.get(name)))

    # 2) passive sampling
    safe = n.sample("/safety/safe_motion_command", SafeMotionCommand, 2.5)
    ch = n.sample("/chassis/state", ChassisState, 2.5)
    odom = n.sample("/odom", Odometry, 2.5)
    seqs = [m.sequence for m in safe]
    check("safe_motion_command 采样", len(safe) >= 20, "n=%d" % len(safe))
    check("safe_motion_command 频率>=20Hz", len(safe) / 2.5 >= 10.0, "%.1fHz" % (len(safe) / 2.5))
    check("sequence 严格递增", all(b > a for a, b in zip(seqs, seqs[1:])), "n=%d first=%s last=%s" % (len(seqs), seqs[0] if seqs else None, seqs[-1] if seqs else None))
    check("ttl_s 合法", all(0 < m.ttl_s <= 0.5 for m in safe), "min=%.3f max=%.3f" % (min([m.ttl_s for m in safe], default=0), max([m.ttl_s for m in safe], default=0)))
    check("chassis/state 采样", len(ch) >= 20, "n=%d" % len(ch))
    check("chassis/state 频率>=20Hz", len(ch) / 2.5 >= 10.0, "%.1fHz" % (len(ch) / 2.5))
    if ch:
        m = ch[-1]
        check("chassis/state 字段完整",
              m.control_mode in ("MANUAL", "AUTONOMOUS") and isinstance(m.motion_allowed, (bool, int)) and
              isinstance(m.can_healthy, (bool, int)) and isinstance(m.feedback_valid, (bool, int)) and
              len(m.wheel_rpm) == 4 and abs(m.linear_velocity) < 1e-6,
              "mode=%s allow=%s can=%s fb=%s fault=%r" % (m.control_mode, m.motion_allowed, m.can_healthy, m.feedback_valid, m.fault_reason))
        check("chassis/state 静止线速度≈0", abs(m.linear_velocity) < 1e-6, "v=%.6f" % m.linear_velocity)
    check("odom 采样", len(odom) >= 20, "n=%d" % len(odom))
    check("odom 频率>=20Hz", len(odom) / 2.5 >= 10.0, "%.1fHz" % (len(odom) / 2.5))
    if odom:
        m = odom[-1]
        check("odom frame", m.header.frame_id == "odom" and m.child_frame_id == "base_link", "%r/%r" % (m.header.frame_id, m.child_frame_id))

    # 3) estop observability (chassis autonomous, no motion source)
    r = n.call_bool("/chassis/set_autonomous_mode", True)
    check("启用 chassis 自主模式", r is not None and r.success, str(r))
    estop_pub = n.create_publisher(Bool, "/safety/emergency_stop", 10)
    time.sleep(0.3)
    estop_pub.publish(Bool(data=True))
    time.sleep(1.0)
    ch_e = n.sample("/chassis/state", ChassisState, 0.6)
    st_e = n.sample("/safety/status", String, 0.6)
    check("急停可观测(chassis_state.emergency_stop)", bool(ch_e) and bool(ch_e[-1].emergency_stop),
          "fault=%r" % (ch_e[-1].fault_reason if ch_e else None))
    check("急停可观测(safety/status)", bool(st_e) and ("ESTOP" in st_e[-1].data.upper() or "EMERGENCY" in st_e[-1].data.upper()), st_e[-1].data if st_e else "")
    estop_pub.publish(Bool(data=False))
    time.sleep(0.5)
    n.call_bool("/chassis/set_autonomous_mode", False)
    n.call_bool("/chassis/set_autonomous_mode", True)
    time.sleep(0.5)
    ch_c = n.sample("/chassis/state", ChassisState, 0.6)
    check("急停锁存清除(关→开自主模式)", bool(ch_c) and not ch_c[-1].emergency_stop and ch_c[-1].fault_reason not in ("SAFE_COMMAND_ESTOP", "EMERGENCY_STOP"),
          "fault=%r" % (ch_c[-1].fault_reason if ch_c else None))
    n.call_bool("/chassis/set_autonomous_mode", False)

    print("\nAUDIT RESULT: PASS=%d FAIL=%d" % (len(PASS), len(FAIL)), flush=True)
    if FAIL:
        print("FAILED:", FAIL, flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if FAIL else 0

if __name__ == "__main__":
    sys.exit(main())
