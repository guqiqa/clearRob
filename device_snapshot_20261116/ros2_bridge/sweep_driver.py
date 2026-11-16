import sys, time
import rclpy
from navigation_interfaces.msg import SafeMotionCommand

def read_speed():
    try:
        return float(open("/tmp/sweep_speed").read().strip())
    except Exception:
        return 0.0

def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    node = rclpy.create_node("bench_sweep_driver")
    pub = node.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
    start = time.monotonic()
    while time.monotonic() - start < duration:
        msg = SafeMotionCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.sequence = time.monotonic_ns() // 1000
        msg.ttl_s = 0.10
        msg.linear_x = read_speed()
        msg.angular_z = 0.0
        msg.enable = True
        msg.stop_required = False
        msg.emergency_stop = False
        msg.source = "bench_sweep_driver"
        msg.reason = "speed_sweep"
        pub.publish(msg)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
