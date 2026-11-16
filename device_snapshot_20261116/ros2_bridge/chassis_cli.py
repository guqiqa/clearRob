#!/usr/bin/env python3
"""ROS 2 底盘控制命令行（当前版本，autonomous 模式）。

用法:
  python3 chassis_cli.py forward 5        # 前进 5 秒（默认 0.2 m/s 幅值，实际按 MID 档速度）
  python3 chassis_cli.py back 3
  python3 chassis_cli.py left 2
  python3 chassis_cli.py right 2
  python3 chassis_cli.py stop
  python3 chassis_cli.py enable           # 开启 autonomous
  python3 chassis_cli.py disable          # 切回 manual
  python3 chassis_cli.py status           # 打印 /odom 一次
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class ChassisCli:
    def __init__(self):
        self.node = rclpy.create_node('chassis_cli')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_client = self.node.create_client(SetBool, '/set_autonomous_mode')
        self.odom = None
        self.node.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.odom = msg

    def spin_wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)

    def publish_twist(self, linear_x=0.0, angular_z=0.0, duration=0.0, rate=20.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        if duration <= 0:
            for _ in range(3):
                self.pub.publish(msg)
                self.spin_wait(0.05)
            return
        end = time.time() + duration
        period = 1.0 / rate
        while time.time() < end:
            self.pub.publish(msg)
            self.spin_wait(period)
        # 停车
        for _ in range(5):
            self.pub.publish(Twist())
            self.spin_wait(0.05)

    def set_mode(self, enable):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            print('服务 /set_autonomous_mode 不可用')
            return False
        req = SetBool.Request()
        req.data = enable
        fut = self.mode_client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if fut.done() and fut.result() is not None:
            print('set_autonomous_mode(%s): %s' % (enable, fut.result().message))
            return True
        print('set_autonomous_mode 调用超时')
        return False

    def status(self):
        self.spin_wait(1.0)
        if self.odom is None:
            print('/odom 尚未收到数据')
            return
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear.x
        print('odom: x=%.3f y=%.3f v=%.3f m/s' % (p.x, p.y, v))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['forward', 'back', 'left', 'right', 'stop', 'enable', 'disable', 'status'])
    parser.add_argument('duration', nargs='?', type=float, default=0.0)
    parser.add_argument('--speed', type=float, default=0.2)
    parser.add_argument('--turn', type=float, default=0.5)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    cli = ChassisCli()
    try:
        if args.action == 'enable':
            cli.set_mode(True)
        elif args.action == 'disable':
            cli.set_mode(False)
        elif args.action == 'status':
            cli.status()
        elif args.action == 'stop':
            cli.publish_twist(0.0, 0.0, 0.0)
        elif args.action == 'forward':
            cli.publish_twist(linear_x=args.speed, duration=args.duration, rate=args.rate)
        elif args.action == 'back':
            cli.publish_twist(linear_x=-args.speed, duration=args.duration, rate=args.rate)
        elif args.action == 'left':
            cli.publish_twist(angular_z=args.turn, duration=args.duration, rate=args.rate)
        elif args.action == 'right':
            cli.publish_twist(angular_z=-args.turn, duration=args.duration, rate=args.rate)
    finally:
        cli.node.destroy_node()
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    sys.exit(main())
