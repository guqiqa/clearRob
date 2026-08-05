#!/usr/bin/env python3
"""
fusion_engine/fusion_node.py — Semantic alert engine (Phase 2 skeleton).

Ported from horizon1 fusion_engine: the semantic-alert pattern that lets a
perception module signal the master state machine to pause.

Current scope (V3, no LiDAR):
  - Subscribes to vision detection results (DetectionArray) — placeholder
    until vision_detector is implemented on BPU.
  - Publishes `fusion/semantic_alert` with PEDESTRIAN_AHEAD / CLEAR so the
    master_bridge can enter OBSTACLE_PAUSE.
  - Debounce / hold-time logic ported from horizon1.

This node has NO hardware dependency — it is the interface contract for
Phase-2 perception.  When vision_detector / radar_driver land, they publish
DetectionArray / obstacle lists and this node does the fusion.

Subscribes:
  - vision/detect/list        (cleaning_robot_interfaces/msg/DetectionArray)
  - control/cmd/unified       (std_msgs/String) — STOP clears alerts
Publishes:
  - fusion/semantic_alert     (std_msgs/String): PEDESTRIAN_AHEAD / CLEAR
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cleaning_robot_common.config import (
    TOPIC_SEMANTIC_ALERT, TOPIC_UNIFIED_CMD,
)

# Which detection classes count as a "pedestrian / must-stop" hazard.
# Phase 2 will expand: pedestrian(5), obstacle(6) from the 8-class vision set.
HAZARD_CLASSES = {"pedestrian", "obstacle"}
# Solid-waste classes (garbage) — for future "clean here" logic, not a stop.
GARBAGE_CLASSES = {"recyclable", "kitchen_waste", "other_waste", "green_waste"}


class FusionEngineNode(Node):
    def __init__(self):
        super().__init__('fusion_engine')

        # alert debounce / hold
        self._last_alert_time = 0.0
        self._alert_hold_s = 3.0
        self._alert_active = False

        # publishers / subscribers
        self._alert_pub = self.create_publisher(String, TOPIC_SEMANTIC_ALERT, 10)
        self._cmd_sub = self.create_subscription(String, TOPIC_UNIFIED_CMD, self._cmd_cb, 10)

        # Try to import DetectionArray (only if interfaces package is built).
        self._vision_sub = None
        try:
            from cleaning_robot_interfaces.msg import DetectionArray  # noqa: F401
            self._vision_sub = self.create_subscription(
                DetectionArray, 'vision/detect/list', self._vision_cb, 10)
            self.get_logger().info("fusion_engine: subscribed to vision/detect/list")
        except ImportError:
            self.get_logger().warn(
                "fusion_engine: cleaning_robot_interfaces/DetectionArray unavailable — "
                "vision fusion disabled until interfaces are built")

        self.get_logger().info("fusion_engine started (semantic alert gateway)")

    def _cmd_cb(self, msg: String):
        if msg.data == "STOP":
            self._clear_alert()

    def _vision_cb(self, msg):
        """Placeholder: when detection results arrive, decide alert state."""
        now = time.time()
        hazard = False
        for det in getattr(msg, 'detections', []):
            name = getattr(det, 'class_name', '') or ''
            if name in HAZARD_CLASSES:
                hazard = True
                break

        if hazard:
            # debounce: only re-alert after the hold window
            if not self._alert_active and now - self._last_alert_time > self._alert_hold_s:
                self._last_alert_time = now
                self._alert_active = True
                self.get_logger().warn("🚨 检测到行人/障碍物，发布 PEDESTRIAN_AHEAD")
                self._alert_pub.publish(String(data="PEDESTRIAN_AHEAD"))
        else:
            self._clear_alert()

    def _clear_alert(self):
        if self._alert_active:
            self._alert_active = False
            self.get_logger().info("✅ 障碍清除，发布 CLEAR")
            self._alert_pub.publish(String(data="CLEAR"))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FusionEngineNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
