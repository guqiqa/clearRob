#!/usr/bin/env python3
"""
calibrate_dirs.py — AUTO-CALIBRATE the 4 motor directions while the robot is
jacked up (wheels off the ground).

Principle: with the wheels free-spinning, command throttle=+1 (forward) and
read the RAW erpm from each motor.  The erpm sign (× pole_pairs × gear_ratio)
tells which PHYSICAL direction each wheel actually turns.  If a wheel's erpm
sign disagrees with the direction the chassis logic expects for "forward", its
motor_dirs entry is flipped.

This is safe on jacked-up wheels: no load, motors free-spin, and the C++ core
runs its normal startup ramp + speed loop.

Expected result for this robot (from the Python-port era, verified on ground):
  FL=2 -> -1, RL=1 -> -1, FR=4 -> +1, RR=3 -> +1   (i.e. [-1, -1, 1, 1])

The script only REPORTS the detected directions and writes a corrected
motor_dirs to stdout.  It does NOT modify any file — apply the value in
phase1_params.yaml (chassis_core.motor_dirs) and restart.

Usage:
  python3 calibrate_dirs.py [seconds]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import sys
import math

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
CORE_ORDER = [2, 1, 4, 3]  # FL, RL, FR, RR (same as C++ core motor order)
LEFT_IDS = [2, 1]
RIGHT_IDS = [4, 3]
PP = 10
GR = 5.2


def collect_feedback(node, topic, duration):
    """Collect chassis/status samples for `duration` s and return wheel rpm[4]."""
    got = None
    deadline = time.time() + duration + 0.5
    while time.time() < deadline:
        msg = None
        try:
            msg = rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:
            pass
        # spin_once doesn't return the msg; use a subscriber via callback instead
        time.sleep(0.02)
    return got


def main():
    rclpy.init()
    node = Node("calibrate_dirs")
    pub = node.create_publisher(String, "/chassis/intent", 10)
    rpm_data = {"values": None}

    def on_status(msg):
        rpm_data["values"] = list(msg.data[:4])

    node.create_subscription(Float32MultiArray, "/chassis/status", on_status, 10)
    time.sleep(0.5)

    def send(throttle, steering):
        import json
        payload = json.dumps({"throttle": throttle, "steering": steering,
                              "gear": "LOW", "mower": 0, "estop": 0})
        for _ in range(int(DURATION * 10)):
            msg = String()
            msg.data = payload
            pub.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(node, timeout_sec=0.05)

    # --- Phase 1: forward ---
    print(f">>> commanding FORWARD (throttle=+1) for {DURATION}s ...")
    send(1.0, 0.0)
    time.sleep(0.3)
    fwd = rpm_data["values"]
    print(f"    forward wheel rpm (FL,RL,FR,RR) = {fwd}")

    # --- stop ---
    send(0.0, 0.0)
    time.sleep(0.5)

    # --- Phase 2: backward ---
    print(f">>> commanding BACKWARD (throttle=-1) for {DURATION}s ...")
    send(-1.0, 0.0)
    time.sleep(0.3)
    bwd = rpm_data["values"]
    print(f"    backward wheel rpm (FL,RL,FR,RR) = {bwd}")

    send(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()

    if not fwd or not bwd:
        print("\nERROR: no wheel feedback received — is chassis_driver running?")
        return

    # --- Determine correct dirs ---
    # For throttle=+1 (forward), each wheel must turn in its "physical forward"
    # direction.  The sign convention: erpm>0 (after ×dir) = measured forward.
    # The C++ core's measured_rpm = erpm * dir / (pp*gr).  If during forward
    # the wheel actually spins forward, its measured_rpm should be > 0.
    # If a wheel's measured rpm is < 0 during forward, its dir is flipped.
    print("\n=== RESULT ===")
    new_dirs = []
    for i, cid in enumerate(CORE_ORDER):
        old = "?"  # we don't know old dir here; caller applies to yaml
        rpm = fwd[i]
        # The measured rpm during forward tells us the CURRENT effective dir.
        # We want ALL wheels to read positive during forward.
        new = -1 if rpm < 0 else 1
        new_dirs.append(new)
        status = "REVERSED" if rpm < 0 else "OK"
        print(f"  CAN{cid} ({'FL' if cid==2 else 'RL' if cid==1 else 'FR' if cid==4 else 'RR'})"
              f" forward_rpm={rpm:+.1f} -> dir={new:+d}  {status}")
    print(f"\nSet chassis_core.motor_dirs in phase1_params.yaml to:")
    print(f"  motor_dirs: {new_dirs}")
    print("Then restart: ros2 launch cleaning_robot_bringup phase1_chassis.launch.py simulate:=false")


if __name__ == "__main__":
    main()
