#!/usr/bin/env python3
"""
ramp_test.py — 缓慢加速电机直到触发限制。
throttle 从 start 逐步升到 1.0，每档保持 hold_s，记录每档电流/转速。
用法: python3 ramp_test.py [gear] [start] [step] [hold_s]
"""
import rclpy, json, time, sys
from std_msgs.msg import String, Float32MultiArray

GEAR = sys.argv[1] if len(sys.argv) > 1 else "MID"
START = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
STEP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
HOLD = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

rclpy.init()
n = rclpy.create_node("ramp_test")
pub = n.create_publisher(String, "/chassis/intent", 10)
last_status = {"data": None}
def cb(msg):
    last_status["data"] = list(msg.data)
n.create_subscription(Float32MultiArray, "/chassis/status", cb, 10)
time.sleep(0.5)

throttle = START
while throttle <= 1.01:
    payload = json.dumps({"throttle": round(throttle, 2), "steering": 0.0,
                          "gear": GEAR, "mower": 0, "estop": 0})
    t0 = time.time()
    rpm = [0]*4
    cur = [0]*4
    # hold this throttle, sample status
    while time.time() - t0 < HOLD:
        pub.publish(String(data=payload))
        rclpy.spin_once(n, timeout_sec=0.05)
        d = last_status["data"]
        if d:
            for i in range(4):
                rpm[i] = abs(d[i])   # wheel RPM
                cur[i] = abs(d[7+i]) # current cmd
    avg_rpm = int(sum(rpm)/4)
    max_rpm = max(rpm)
    avg_cur = int(sum(cur)/4)
    max_cur = max(cur)
    print(f"thr={throttle:.2f} rpm_avg={avg_rpm} rpm_max={max_rpm} cur_avg={avg_cur} cur_max={max_cur}", flush=True)
    # stop between steps
    for _ in range(10):
        pub.publish(String(data=json.dumps({"throttle": 0.0, "steering": 0.0, "gear": GEAR, "mower": 0, "estop": 0})))
        time.sleep(0.05)
    if max_rpm > 170:  # approaching overspeed
        print("APPROACHING OVERSPEED — stopping ramp", flush=True)
        break
    throttle += STEP

# final stop
for _ in range(20):
    pub.publish(String(data=json.dumps({"throttle": 0.0, "steering": 0.0, "gear": GEAR, "mower": 0, "estop": 0})))
    time.sleep(0.05)
n.destroy_node(); rclpy.shutdown()
print("ramp test done")
