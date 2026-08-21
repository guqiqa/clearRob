#!/usr/bin/env python3
"""arc_test.py — send a sustained arc-turn intent and sample wheel RPM+currents."""
import rclpy, time, json
from std_msgs.msg import String, Float32MultiArray

rclpy.init()
n = rclpy.create_node("arc2")
pub = n.create_publisher(String, "/chassis/intent", 10)
samples = []
def cb(m):
    samples.append(list(m.data))
n.create_subscription(Float32MultiArray, "/chassis/status", cb, 10)
time.sleep(0.3)

payload = json.dumps({"throttle":0.6,"steering":0.6,"gear":"MID","mower":0,"estop":0})
print("sending arc (th=0.6 s=0.6) for 10s...")
start = time.time()
while time.time()-start < 10:
    pub.publish(String(data=payload))
    rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.02)
n.destroy_node()
rclpy.shutdown()

if samples:
    step = max(1, len(samples)//12)
    print(f"captured {len(samples)} samples")
    for s in samples[::step][:12]:
        print(f"RPM[FL{int(s[0]):3d} RL{int(s[1]):3d} FR{int(s[2]):3d} RR{int(s[3]):3d}] "
              f"cur[FL{int(s[7]):4d} RL{int(s[8]):4d} FR{int(s[9]):4d} RR{int(s[10]):4d}]")
else:
    print("no samples")
