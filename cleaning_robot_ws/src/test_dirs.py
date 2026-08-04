#!/usr/bin/env python3
"""
test_dirs.py — light-touch direction test while wheels are jacked up.
Sends a small throttle (0.15) to avoid overspeed latch, then reads the
wheel-rpm feedback sign from /chassis/status to verify each wheel's PHYSICAL
direction.

With correct motor_dirs, during FORWARD every wheel reads positive rpm.
During LEFT (rotate), FL/RL read one sign, FR/RR the opposite.

Usage:  python3 test_dirs.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import time

CORE_ORDER = [2, 1, 4, 3]
NAMES = {2: "FL", 1: "RL", 4: "FR", 3: "RR"}
LIGHT = 0.15  # low enough to avoid the 120rpm overspeed latch on free wheels


def main():
    rclpy.init()
    node = Node("test_dirs")
    pub = node.create_publisher(String, "/chassis/intent", 10)
    last = {"rpm": None, "t": 0.0}

    def on_status(msg):
        last["rpm"] = list(msg.data[:4])
        last["t"] = time.time()

    node.create_subscription(Float32MultiArray, "/chassis/status", on_status, 10)
    time.sleep(0.5)

    def run(label, throttle, steering, dur=1.6):
        print(f"\n=== {label}: throttle={throttle} steering={steering} ===")
        payload = json.dumps({"throttle": throttle, "steering": steering,
                              "gear": "LOW", "mower": 0, "estop": 0})
        for _ in range(int(dur * 10)):
            msg = String()
            msg.data = payload
            pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.3)
        rpm = last["rpm"]
        if rpm:
            line = "  ".join(f"{NAMES[cid]}({cid})={rpm[i]:+.0f}" for i, cid in enumerate(CORE_ORDER))
            print(f"    wheel rpm: {line}")
        return rpm

    # Reset any safety latch first
    pub.publish(String(data=json.dumps({"throttle": 0, "steering": 0,
                                        "gear": "LOW", "mower": 0, "estop": 0})))
    time.sleep(0.5)

    fwd = run("FORWARD", LIGHT, 0.0)
    time.sleep(0.5)
    bwd = run("BACKWARD", -LIGHT, 0.0)
    time.sleep(0.5)
    left = run("LEFT (rotate)", 0.0, LIGHT)
    time.sleep(0.5)
    run("STOP", 0.0, 0.0)

    node.destroy_node()
    rclpy.shutdown()

    if not fwd or not bwd:
        print("\nNo feedback captured — is chassis_driver running?")
        return

    print("\n=== VERDICT ===")
    ok = True
    for i, cid in enumerate(CORE_ORDER):
        f = fwd[i]
        b = bwd[i]
        # forward must be positive, backward negative, for a correct dir
        good = (f > 0) and (b < 0)
        ok = ok and good
        print(f"  {NAMES[cid]}(CAN{cid}): fwd={f:+.0f} bwd={b:+.0f} -> {'OK' if good else 'REVERSED'}")
    print("\n  motor_dirs should be: "
          + str([-1 if fwd[i] < 0 else 1 for i in range(4)])
          + "  (in FL,RL,FR,RR order)")
    print("  " + ("ALL CORRECT" if ok else "SOME REVERSED — apply the value above in phase1_params.yaml"))


if __name__ == "__main__":
    main()
