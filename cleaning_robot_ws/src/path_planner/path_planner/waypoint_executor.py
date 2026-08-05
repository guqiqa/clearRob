#!/usr/bin/env python3
"""
path_planner/waypoint_executor.py — Waypoint follower that drives the real
chassis through /chassis/intent (NO Nav2).

The horizon1 planner delegated waypoint execution to Nav2's
navigate_through_poses action.  Our V3 chassis has no Nav2 / LiDAR SLAM, so
this executor replaces that dependency: it reads /odom for position + yaw and
issues intent commands (rotate-in-place to align heading, then drive forward).

Pipeline per waypoint:
  1. Read current pose from /odom (x, y, yaw).
  2. Compute target heading = atan2(dy, dx) and distance = hypot(dx, dy).
  3. If heading error > align_threshold: rotate in place (steering intent)
     until |error| < threshold.
  4. Drive forward (throttle intent) until odom distance covered >= target.
  5. Move to next waypoint.
"""

import json
import math
import threading
import time

from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def _yaw_from_quat(q):
    """Extract yaw (rad) from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointExecutor:
    """Drives the chassis through a list of (x, y) waypoints via intent.

    Not a ROS node itself — it takes a Node for pub/sub and a topic name for
    the intent publisher, so path_planner_node can embed it.
    """

    def __init__(self, node: Node, intent_topic: str = "/chassis/intent",
                 odom_topic: str = "/odom",
                 align_deg: float = 5.0,
                 drive_throttle: float = 0.4,
                 rotate_steering: float = 0.8,
                 arrival_tol_m: float = 0.15,
                 max_wait_s: float = 60.0):
        self._node = node
        self._intent_pub = node.create_publisher(String, intent_topic, 10)
        self._odom = None
        self._odom_sub = node.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self._align_deg = align_deg
        self._drive_throttle = drive_throttle
        self._rotate_steering = rotate_steering
        self._arrival_tol_m = arrival_tol_m
        self._max_wait_s = max_wait_s

        self._stop_event = threading.Event()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    # ---- helpers ----

    def _publish_intent(self, throttle: float, steering: float) -> None:
        payload = json.dumps({
            "throttle": float(throttle),
            "steering": float(steering),
            "gear": "MID",
            "mower": 0,
            "estop": 0,
        })
        msg = String()
        msg.data = payload
        self._intent_pub.publish(msg)

    def _pose(self):
        """Return (x, y, yaw_rad) from latest odom, or None."""
        if self._odom is None:
            return None
        p = self._odom.pose.pose
        return (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _wait_pose(self, timeout=None):
        """Block until odom pose is available."""
        deadline = time.time() + (timeout or 10.0)
        while time.time() < deadline:
            pose = self._pose()
            if pose is not None:
                return pose
            time.sleep(0.02)
        return None

    def _send_intent_until(self, throttle, steering, stop_cond, timeout_s):
        """Publish intent repeatedly until stop_cond() or timeout."""
        deadline = time.time() + timeout_s
        while time.time() < deadline and not self._stop_event.is_set():
            self._publish_intent(throttle, steering)
            if stop_cond():
                return True
            time.sleep(0.1)
        # safety: always stop at end
        self._publish_intent(0.0, 0.0)
        return False

    # ---- public API ----

    def stop(self):
        self._stop_event.set()
        self._publish_intent(0.0, 0.0)

    def reset(self):
        self._stop_event = threading.Event()

    def execute(self, waypoints):
        """Follow a list of [(x, y), ...] waypoints. Returns True if all reached."""
        pose = self._wait_pose()
        if pose is None:
            self._node.get_logger().error("waypoint_executor: no /odom — aborting")
            return False

        for idx, (tx, ty) in enumerate(waypoints):
            if self._stop_event.is_set():
                self._publish_intent(0.0, 0.0)
                return False

            self._node.get_logger().info(f"[executor] waypoint {idx+1}/{len(waypoints)}: ({tx:.2f}, {ty:.2f})")

            # ---- 1. rotate to align heading ----
            ok = self._align_to(tx, ty)
            if not ok:
                self._node.get_logger().warn(f"[executor] waypoint {idx+1}: align failed")
                return False

            # ---- 2. drive forward until arrival ----
            ok = self._drive_to(tx, ty)
            if not ok:
                self._node.get_logger().warn(f"[executor] waypoint {idx+1}: drive failed")
                return False

        self._publish_intent(0.0, 0.0)
        self._node.get_logger().info("[executor] all waypoints reached")
        return True

    def _align_to(self, tx, ty):
        """Rotate in place until the robot faces (tx, ty). Returns True if aligned."""
        pose = self._wait_pose()
        if pose is None:
            return False
        start = time.time()

        while time.time() - start < self._max_wait_s and not self._stop_event.is_set():
            pose = self._wait_pose(timeout=0.5)
            if pose is None:
                return False
            x, y, yaw = pose
            target_yaw = math.atan2(ty - y, tx - x)
            err = target_yaw - yaw
            # wrap to [-180, 180]
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            err_deg = math.degrees(err)

            if abs(err_deg) <= self._align_deg:
                self._publish_intent(0.0, 0.0)
                return True

            # rotate direction: positive steering = left (per our intent mapping)
            steering = self._rotate_steering if err > 0 else -self._rotate_steering
            self._publish_intent(0.0, steering)
            time.sleep(0.1)

        self._publish_intent(0.0, 0.0)
        return False

    def _drive_to(self, tx, ty):
        """Drive forward toward (tx, ty) until within arrival_tol_m. Returns True."""
        pose = self._wait_pose()
        if pose is None:
            return False
        start = time.time()

        while time.time() - start < self._max_wait_s and not self._stop_event.is_set():
            pose = self._wait_pose(timeout=0.5)
            if pose is None:
                return False
            x, y, _ = pose
            dist = math.hypot(tx - x, ty - y)
            if dist <= self._arrival_tol_m:
                self._publish_intent(0.0, 0.0)
                return True

            # keep small heading correction while driving
            target_yaw = math.atan2(ty - y, tx - x)
            yaw = _yaw_from_quat(self._odom.pose.pose.orientation)
            err = target_yaw - yaw
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            steer = 0.0
            if abs(err) > math.radians(self._align_deg):
                steer = 0.3 * (1.0 if err > 0 else -1.0)

            self._publish_intent(self._drive_throttle, steer)
            time.sleep(0.1)

        self._publish_intent(0.0, 0.0)
        return False
