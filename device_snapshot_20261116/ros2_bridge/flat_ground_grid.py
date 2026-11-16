#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bench/ground fallback sensor: publishes an all-free local occupancy grid.

Used only while no real perception is connected. The grid is axis-aligned in
odom, 80x80 cells at 0.25 m, centred on the pose captured at startup, and
published at 5 Hz. It never publishes goals.
"""

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


class FlatGroundGrid(Node):
    def __init__(self):
        super().__init__("flat_ground_grid")
        self.declare_parameter("width", 80)
        self.declare_parameter("height", 80)
        self.declare_parameter("resolution_m", 0.25)
        self.declare_parameter("rate_hz", 5.0)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution_m").value)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.origin = None
        self.pub = self.create_publisher(OccupancyGrid, "/navigation/occupancy_grid", 1)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(1.0 / rate, self.publish_grid)
        self.get_logger().info("flat_ground_grid ready (no goals, all-free local grid)")

    def on_odom(self, msg):
        if self.origin is None:
            half_w = 0.5 * self.width * self.resolution
            half_h = 0.5 * self.height * self.resolution
            self.origin = (msg.pose.pose.position.x - half_w,
                           msg.pose.pose.position.y - half_h)
            self.get_logger().info("grid origin fixed at %.2f,%.2f" % self.origin)

    def publish_grid(self):
        if self.origin is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "odom"
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin[0]
        grid.info.origin.position.y = self.origin[1]
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self.width * self.height)
        self.pub.publish(grid)


def main():
    rclpy.init()
    node = FlatGroundGrid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
