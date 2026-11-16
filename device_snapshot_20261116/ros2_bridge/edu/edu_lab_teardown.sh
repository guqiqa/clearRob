#!/bin/bash
# 教学实验统一清理：禁用自主模式、停教学进程、验证 MANUAL + 零速。
set +u
source /root/nav_ws/env.sh
timeout 6 ros2 service call /chassis/set_autonomous_mode std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
pkill -f "[e]du_wheel_rpm_lab.py" 2>/dev/null
pkill -f "[e]du_heading_pid_lab.py" 2>/dev/null
pkill -f "[a]rc_cmd.py" 2>/dev/null
pkill -f "[r]ecord_state_odom.py" 2>/dev/null
sleep 1

python3 - <<'PY'
import sys
import time
import rclpy
from navigation_interfaces.msg import ChassisState

rclpy.init()
node = rclpy.create_node("edu_lab_teardown_check")
box = {}

def cb(msg):
    box["s"] = msg

node.create_subscription(ChassisState, "/chassis/state", cb, 10)
t0 = time.monotonic()
while "s" not in box and time.monotonic() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)

s = box.get("s")
if s is None:
    print("TEARDOWN FAIL: no /chassis/state")
    sys.exit(1)

max_rpm = max(abs(x) for x in s.wheel_rpm)
ok = (
    str(s.control_mode) == "MANUAL"
    and not s.emergency_stop
    and max_rpm <= 0.5
    and abs(s.linear_velocity) <= 0.01
    and abs(s.angular_velocity) <= 0.01
)
print("state: mode=%s estop=%s v=%.3f w=%.3f wheels=%s"
      % (s.control_mode, s.emergency_stop, s.linear_velocity,
         s.angular_velocity, [round(x, 2) for x in s.wheel_rpm]))
print("TEARDOWN PASS" if ok else "TEARDOWN FAIL")
sys.exit(0 if ok else 1)
PY

RC=$?
if [ $RC -eq 0 ]; then echo "TEARDOWN PASS"; else echo "TEARDOWN FAIL"; fi
exit $RC
