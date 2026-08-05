#!/usr/bin/env python3
"""
path_planner/path_planner_node.py — Coverage path planner for the real robot.

Integrates the horizon1 boustrophedon planner with our V3 intent-driven chassis:
  - Listens for planning triggers ("GO" / "GO_ROAD") on `control/trigger_planning`
  - Generates waypoints: zigzag (AUTO_SWEEP) or road JSON (ROAD_SWEEP)
  - Executes them through WaypointExecutor → /chassis/intent (no Nav2)

Subscribes:
  - control/trigger_planning  (std_msgs/String): "GO" / "GO_ROAD"
Publishes:
  - /chassis/intent          (std_msgs/String): motion commands via executor
  - planner/status           (std_msgs/String): "idle" / "planning" / "executing"
"""

import json
import math
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from path_planner.zigzag import generate_zigzag_path, smart_sweep_direction
from path_planner.waypoint_executor import WaypointExecutor


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ---- params ----
        self.declare_parameter('zone', [0.0, 0.0, 5.0, 8.0])      # x0,y0,x1,y1 zone
        self.declare_parameter('sweep_spacing', 0.6)              # cleaning width
        self.declare_parameter('road_path', '/root/road_path.json')
        self.declare_parameter('intent_topic', '/chassis/intent')
        self.declare_parameter('odom_topic', '/odom')

        z = self.get_parameter('zone').value
        # zone as 4 corners [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
        self.zone = [(z[0], z[1]), (z[2], z[1]), (z[2], z[3]), (z[0], z[3])]
        self.sweep_spacing = float(self.get_parameter('sweep_spacing').value)
        self.road_path = self.get_parameter('road_path').value

        # ---- state ----
        self.active_task = None
        self.planning = False
        self._latest_odom = None  # cached pose from /odom

        # ---- waypoint executor (lazy init — Humble walks Node attrs) ----
        self._executor_cfg = {
            "intent_topic": self.get_parameter('intent_topic').value,
            "odom_topic": self.get_parameter('odom_topic').value,
        }
        self._executor = None

        # ---- pub/sub ----
        self.trigger_sub = self.create_subscription(
            String, 'control/trigger_planning', self.trigger_cb, 10)
        self.status_pub = self.create_publisher(String, 'planner/status', 10)
        # Also subscribe to odom directly so _robot_pos works before executor inits
        self._odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                                    self._odom_cb, 10)

        self.get_logger().info(
            f"path_planner ready: zone={self.zone} sweep={self.sweep_spacing}m "
            f"road={self.road_path}")

    def _set_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ---- trigger ----

    def trigger_cb(self, msg: String):
        if self.planning:
            self.get_logger().warn("path_planner: already busy, ignoring trigger")
            return
        if msg.data == "GO":
            self.active_task = "SQUARE"
            self.get_logger().info("🔫 收到广场指令 (AUTO_SWEEP)")
            self._start_planning()
        elif msg.data == "GO_ROAD":
            self.active_task = "ROAD"
            self.get_logger().info("🔫 收到马路指令 (ROAD_SWEEP)")
            self._start_planning()
        else:
            self.get_logger().warn(f"path_planner: unknown trigger '{msg.data}'")

    def _start_planning(self):
        self.planning = True
        self._set_status("planning")
        # run in a thread so we don't block the ROS executor
        import threading
        thread = threading.Thread(target=self._plan_and_execute, daemon=True)
        thread.start()

    # ---- planning ----

    def _plan_and_execute(self):
        try:
            if self.active_task == "SQUARE":
                waypoints = self._plan_square()
            else:
                waypoints = self._plan_road()
            self.active_task = None

            if not waypoints:
                self.get_logger().error("path_planner: no valid waypoints")
                self._set_status("idle")
                self.planning = False
                return

            self.get_logger().info(
                f"✨ 生成 {len(waypoints)} 个航点: {waypoints[:5]}{'...' if len(waypoints) > 5 else ''}")

            # Lazy-init executor (Humble walks Node attrs; only safe after super().__init__)
            if self._executor is None:
                self._executor = WaypointExecutor(
                    self,
                    intent_topic=self._executor_cfg["intent_topic"],
                    odom_topic=self._executor_cfg["odom_topic"],
                )

            self._set_status("executing")
            self._executor.reset()
            ok = self._executor.execute(waypoints)
            self.get_logger().info(f"执行完成: {'成功' if ok else '中止/失败'}")
        except Exception as e:
            self.get_logger().error(f"path_planner: {e}")
        finally:
            self._set_status("idle")
            self.planning = False

    def _odom_cb(self, msg):
        self._latest_odom = msg

    def _robot_pos(self):
        """Best-effort robot (x,y) from latest odom."""
        if self._latest_odom is None:
            return 0.0, 0.0
        p = self._latest_odom.pose.pose
        return p.position.x, p.position.y

    def _plan_square(self):
        robot_x, robot_y = self._robot_pos()
        from_top = smart_sweep_direction(self.zone, robot_y)
        raw = generate_zigzag_path(self.zone, robot_x,
                                   sweep_spacing=self.sweep_spacing,
                                   start_from_top=from_top)
        self.get_logger().info(
            f"📍 弓字形生成 {len(raw)} 个原始航点 (from_top={from_top})")
        return raw

    def _plan_road(self):
        if not os.path.exists(self.road_path):
            self.get_logger().error(f"❌ 找不到马路文件: {self.road_path}")
            return []
        with open(self.road_path, 'r') as f:
            raw = json.load(f)
        if len(raw) < 2:
            return []

        robot_x, robot_y = self._robot_pos()
        d0 = math.hypot(robot_x - raw[0][0], robot_y - raw[0][1])
        d1 = math.hypot(robot_x - raw[-1][0], robot_y - raw[-1][1])
        if d1 < d0:
            self.get_logger().info("📍 离终点更近，反向回扫")
            raw.reverse()
        else:
            self.get_logger().info("📍 离起点更近，正向清扫")
        return raw


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPlannerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
