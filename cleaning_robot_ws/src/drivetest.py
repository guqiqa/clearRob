#!/usr/bin/env python3
"""
drivetest.py — send a chassis motion intent directly via rclpy.
Usage:  python3 drivetest.py forward [seconds]
        python3 drivetest.py backward [seconds]
        python3 drivetest.py left [seconds]
        python3 drivetest.py right [seconds]
        python3 drivetest.py arc [seconds]
        python3 drivetest.py fwd_fast [seconds]
        python3 drivetest.py stop
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

INTENT_TOPIC = "/chassis/intent"

ACTIONS = {
    "forward":  {"throttle": 0.4, "steering": 0.0, "gear": "LOW", "mower": 0, "estop": 0},
    "backward": {"throttle": -0.4, "steering": 0.0, "gear": "LOW", "mower": 0, "estop": 0},
    "fwd_fast": {"throttle": 0.6, "steering": 0.0, "gear": "HIGH", "mower": 0, "estop": 0},
    "left":     {"throttle": 0.0, "steering": 1.0, "gear": "MID", "mower": 0, "estop": 0},
    "right":    {"throttle": 0.0, "steering": -1.0, "gear": "MID", "mower": 0, "estop": 0},
    "arc":      {"throttle": 0.4, "steering": 0.6, "gear": "MID", "mower": 0, "estop": 0},
    "stop":     {"throttle": 0.0, "steering": 0.0, "gear": "MID", "mower": 0, "estop": 1},
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ACTIONS:
        print(__doc__)
        return
    action = ACTIONS[sys.argv[1]]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    import json
    payload = json.dumps(action)

    rclpy.init()
    node = Node("drivetest")
    pub = node.create_publisher(String, INTENT_TOPIC, 10)
    time.sleep(0.5)

    rate_hz = 10
    steps = max(1, int(secs * rate_hz))
    print(f">>> {sys.argv[1]} for {secs}s: {payload}")
    for _ in range(steps):
        msg = String()
        msg.data = payload
        pub.publish(msg)
        time.sleep(1.0 / rate_hz)
    print(">>> done")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
