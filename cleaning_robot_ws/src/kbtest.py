#!/usr/bin/env python3
"""Keyboard motor test — directly publishes /joy, monitors /cmd_vel."""
import rclpy, time, sys, tty, termios, select
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class KeyboardMotor(Node):
    def __init__(self):
        super().__init__("keyboard_motor")
        self._pub = self.create_publisher(Joy, "/joy", 10)
        self._sub_cmd = self.create_subscription(Twist, "/cmd_vel", self._cb_cmd, 10)
        self._sub_state = self.create_subscription(String, "/master/state", self._cb_state, 10)
        self._last_cmd = None
        self._last_state = "?"
        self._started = False

    def _cb_cmd(self, msg): self._last_cmd = msg
    def _cb_state(self, msg): self._last_state = msg.data

    def send(self, throttle=0.0, steering=0.0, btn_start=0, btn_estop=0):
        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.axes = [float(steering), float(throttle), 0.0, 0.0]
        m.buttons = [btn_start, 0, btn_estop, 0]
        self._pub.publish(m)

    def spin(self, duration):
        t0=time.time()
        while time.time()-t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

    def status(self, label):
        v = self._last_cmd.linear.x if self._last_cmd else 0.0
        w = self._last_cmd.angular.z if self._last_cmd else 0.0
        print(f"  [{self._last_state:8s}] v={v:+.2f} w={w:+.2f}  {label}")

def main():
    rclpy.init(args=[])
    k = KeyboardMotor()
    print("\n=== KEYBOARD MOTOR TEST ===")
    print("W=forward S=back A=left D=right SPACE=toggle Q=quit")
    print(f"State: {k._last_state}")

    # Wait for master_bridge + chassis_driver to come up
    for _ in range(50):
        k.send(0.0, 0.0, btn_start=0)
        k.spin(0.02)

    old = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'w':
                    k.send(0.3, 0.0); k.spin(0.02)
                    k.status("FORWARD")
                elif key == 's':
                    k.send(-0.2, 0.0); k.spin(0.02)
                    k.status("BACK")
                elif key == 'a':
                    k.send(0.0, 0.5); k.spin(0.02)
                    k.status("LEFT")
                elif key == 'd':
                    k.send(0.0, -0.5); k.spin(0.02)
                    k.status("RIGHT")
                elif key == ' ':
                    k.send(0.0, 0.0, btn_start=1); k.spin(0.05)
                    k.send(0.0, 0.0, btn_start=0); k.spin(0.05)
                    print(f"  TOGGLE -> [{k._last_state}]")
                elif key == 'q':
                    break
            else:
                k.send(0.0, 0.0); k.spin(0.01)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        q = Twist()
        k._pub_cmd = k.create_publisher(Twist, "/cmd_vel", 10) if False else None
        k.destroy_node()
        rclpy.shutdown()
        print("\nDone.")

if __name__ == "__main__":
    main()
