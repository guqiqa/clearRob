import csv, math, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import ChassisState

rclpy.init()
out = open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/trace.csv", "w")
writer = csv.writer(out)
writer.writerow(["t", "x", "y", "yaw_deg", "v", "w", "fl", "fr", "allowed", "state"])
node = rclpy.create_node("trace_recorder")
t0 = node.get_clock().now()
rpm = [0.0, 0.0]
allowed = 0
last = {}


def on_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    t = (node.get_clock().now() - t0).nanoseconds / 1e9
    writer.writerow([round(t, 3), round(p.x, 4), round(p.y, 4), round(math.degrees(yaw), 2),
                     round(msg.twist.twist.linear.x, 4), round(msg.twist.twist.angular.z, 4),
                     round(rpm[0], 2), round(rpm[1], 2), allowed, last.get("s", "")])
    out.flush()


def on_state(msg):
    global rpm, allowed
    rpm = [float(msg.wheel_rpm[0]), float(msg.wheel_rpm[2])]
    allowed = 1 if msg.motion_allowed else 0
    last["s"] = msg.safety_state


node.create_subscription(Odometry, "/odom", on_odom, 10)
node.create_subscription(ChassisState, "/chassis/state", on_state, 10)
rclpy.spin(node)