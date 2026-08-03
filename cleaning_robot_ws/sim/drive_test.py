#!/usr/bin/env python3
"""drive_test.py — Low-speed motor test for the WheelPID V2 port.

Publishes a small /cmd_vel for N seconds while sampling /odom twist, to
verify the full chain: cmd_vel → chassis_driver → CAN → motors → feedback → odom.

Usage:  python3 /tmp/drive_test.py <linear_x> [duration_s=2.0]
Example: python3 /tmp/drive_test.py 0.05
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveTest(Node):
    def __init__(self):
        super().__init__("drive_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)

    def cb_odom(self, msg):
        self.odom = msg

    def spin(self, duration):
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)


def main():
    lin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    node = DriveTest()
    node.get_logger().info(f"Publishing /cmd_vel linear.x={lin} for {dur}s")

    # brief settle to catch any stale odom
    node.spin(0.5)

    # publish cmd_vel at 20 Hz while sampling odom
    period = 0.05
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < dur:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = 0.0
        node.pub.publish(msg)
        node.spin(period)

    # stop
    for _ in range(5):
        node.pub.publish(Twist())
        node.spin(0.1)

    # final odom sample
    node.spin(0.3)
    if node.odom:
        v = node.odom.twist.twist.linear.x
        w = node.odom.twist.twist.angular.z
        node.get_logger().info(f"FINAL odom: v={v:.4f} m/s  w={w:.4f} rad/s")
    else:
        node.get_logger().warn("No /odom received")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
