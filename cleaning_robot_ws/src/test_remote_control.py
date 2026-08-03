#!/usr/bin/env python3
"""
test_remote_control — Automated /joy sequence for testing control chain.

Runs on device. Publishes a sequence of /joy messages to test:
  1. STANDBY → MANUAL (press START)
  2. Forward (0.3 m/s) × 2s
  3. Turn left × 1s
  4. Stop
  5. MANUAL → STANDBY

Monitor with:
  ros2 topic echo /cmd_vel
  ros2 topic echo /odom
  ros2 topic echo /master/state
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TestRemote(Node):
    def __init__(self):
        super().__init__("test_remote")
        self._pub = self.create_publisher(Joy, "/joy", 10)

    def send(self, throttle=0.0, steering=0.0, btn_start=0, btn_estop=0):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "test"
        msg.axes = [steering, throttle, 0.0, 0.0]
        msg.buttons = [btn_start, 0, btn_estop, 0]
        self._pub.publish(msg)

    def press_start(self):
        self.send(btn_start=1)
        time.sleep(0.1)
        self.send(btn_start=0)
        time.sleep(0.1)


def main():
    rclpy.init(args=[])
    node = TestRemote()

    print("=" * 50)
    print("  Control Chain Test — Simulated Remote")
    print("=" * 50)

    # Step 1: STANDBY → MANUAL
    print("\n[1/5] STANDBY → MANUAL (press CH5)...")
    node.press_start()
    time.sleep(0.5)

    # Step 2: Forward at 0.3 m/s for 2s
    print("[2/5] FORWARD 0.3 m/s × 2s")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        node.send(throttle=0.3, steering=0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Step 3: Turn left for 1s
    print("[3/5] TURN LEFT × 1s")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        node.send(throttle=0.0, steering=0.5)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Step 4: Stop
    print("[4/5] STOP (throttle=0, steering=0)")
    for _ in range(10):
        node.send(throttle=0.0, steering=0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Step 5: MANUAL → STANDBY
    print("[5/5] MANUAL → STANDBY (press CH5)")
    node.press_start()

    print("\n" + "=" * 50)
    print("  Test sequence complete!")
    print("  Check results:")
    print("    ros2 topic echo /cmd_vel")
    print("    ros2 topic echo /odom")
    print("    ros2 topic echo /master/state")
    print("=" * 50)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
