import sys, time
import rclpy
from navigation_interfaces.msg import SafeMotionCommand

def publish_cmd(pub, node, speed):
    msg = SafeMotionCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    # steady_clock 派生的单调微秒数：跨进程也严格递增，满足 v4 的 sequence 校验
    msg.sequence = time.monotonic_ns() // 1000
    msg.ttl_s = 0.10
    msg.linear_x = float(speed)
    msg.angular_z = 0.0
    msg.enable = True
    msg.stop_required = False
    msg.emergency_stop = False
    msg.source = "bench_safe_cmd_pub"
    msg.reason = "speed_test"
    pub.publish(msg)

def main():
    speed = float(sys.argv[1])
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    rclpy.init()
    node = rclpy.create_node("bench_safe_cmd_pub")
    pub = node.create_publisher(SafeMotionCommand, "/safety/safe_motion_command", 10)
    print("PUB_START speed=%.3f duration=%.1f" % (speed, duration), flush=True)
    start = time.monotonic()
    while time.monotonic() - start < duration:
        publish_cmd(pub, node, speed)
        time.sleep(0.05)
    stop_start = time.monotonic()
    while time.monotonic() - stop_start < 0.6:
        publish_cmd(pub, node, 0.0)
        time.sleep(0.05)
    print("PUB_DONE speed=%.3f" % speed, flush=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
