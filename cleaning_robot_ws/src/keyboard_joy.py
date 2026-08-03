#!/usr/bin/env python3
"""
keyboard_joy — Keyboard-based /joy publisher for remote control testing.

Controls:
  W/S       → forward / backward (linear velocity)
  A/D       → left / right (angular velocity)
  Space     → toggle MANUAL / STANDBY (CH5 button)
  E         → ESTOP toggle (CH7 button)
  Q         → quit

Publishes /joy with 4 axes + 4 buttons at 50 Hz.
"""

import sys
import tty
import termios
import select
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class KeyboardJoyNode(Node):
    def __init__(self):
        super().__init__("keyboard_joy")
        self._pub = self.create_publisher(Joy, "/joy", 10)
        self._timer = self.create_timer(0.02, self._publish)  # 50 Hz

        self._throttle = 0.0   # -1.0 .. +1.0
        self._steering = 0.0   # -1.0 .. +1.0
        self._btn_start = 0    # CH5: toggle MANUAL/STANDBY
        self._btn_estop = 0    # CH7: emergency stop
        self._running = True

        self.get_logger().info("Keyboard Joystick ready")
        self._print_help()

    def _print_help(self):
        print("\n" + "=" * 50)
        print("  Keyboard Remote Control Simulator")
        print("=" * 50)
        print("  W/S     → forward / backward (throttle)")
        print("  A/D     → turn left / right (steering)")
        print("  SPACE   → toggle MANUAL / STANDBY")
        print("  E       → ESTOP (emergency stop)")
        print("  Q       → quit")
        print("  H       → print this help")
        print("=" * 50)
        print("  Current: STANDBY (press SPACE to enter MANUAL)")
        print()

    def set_throttle(self, val: float):
        self._throttle = max(-1.0, min(1.0, val))

    def set_steering(self, val: float):
        self._steering = max(-1.0, min(1.0, val))

    def press_start(self):
        self._btn_start = 1
        time.sleep(0.05)
        self._btn_start = 0

    def toggle_estop(self):
        self._btn_estop = 1 - self._btn_estop
        if self._btn_estop:
            self.get_logger().warn("ESTOP PRESSED")
        else:
            self.get_logger().info("ESTOP released")

    def stop(self):
        self._running = False

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "keyboard"

        # Axes: [steering, throttle, 0, 0]
        msg.axes = [self._steering, self._throttle, 0.0, 0.0]

        # Buttons: [CH5(START), 0, CH7(ESTOP), 0]
        msg.buttons = [self._btn_start, 0, self._btn_estop, 0]

        self._pub.publish(msg)


def get_key(timeout=0.05):
    """Non-blocking read of a single keypress. Returns '' if no key."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return ''


def main():
    # Set terminal to raw mode
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    rclpy.init(args=[])
    node = KeyboardJoyNode()

    try:
        while node._running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(0.02)
            if not key:
                continue

            k = key.lower()
            if k == 'w':
                node.set_throttle(0.5)
                node.set_steering(0.0)
                print(f"  → FORWARD  throttle=0.5")
            elif k == 's':
                node.set_throttle(-0.3)
                node.set_steering(0.0)
                print(f"  → BACKWARD throttle=-0.3")
            elif k == 'a':
                node.set_steering(1.0)
                node.set_throttle(0.0)
                print(f"  → TURN LEFT")
            elif k == 'd':
                node.set_steering(-1.0)
                node.set_throttle(0.0)
                print(f"  → TURN RIGHT")
            elif k == ' ':
                node.press_start()
                print(f"  → START toggled (MANUAL/STANDBY)")
            elif k == 'e':
                node.toggle_estop()
                print(f"  → ESTOP toggled")
            elif k == 'h':
                node._print_help()
            elif k == 'q':
                print("\n  Quit.")
                node.stop()
            else:
                # Release all on any other key
                node.set_throttle(0.0)
                node.set_steering(0.0)
                print(f"  → NEUTRAL (key released)")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
