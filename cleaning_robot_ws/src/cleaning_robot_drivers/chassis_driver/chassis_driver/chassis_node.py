#!/usr/bin/env python3
"""
chassis_driver/chassis_node.py — CAN bus motor controller (ROS2 node).

Uses cleaning_robot_common for all constants, kinematics, and CAN protocol.
This file contains only the ROS2 lifecycle (subs/pubs/timers) and runtime state.

Subscribes:  /cmd_vel (geometry_msgs/Twist)
Publishes:   /odom (nav_msgs/Odometry), /chassis/status, tf
Hardware:    can0 (SocketCAN, 500 Kbps)
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

try:
    import can
    HAS_CAN = True
except ImportError:
    HAS_CAN = False

from cleaning_robot_common.config import (
    # CAN
    DRIVE_IDS, LEFT_IDS, RIGHT_IDS,
    CAN_CMD_HEARTBEAT,
    # Kinematics
    MAX_LINEAR_VELOCITY, MAX_MOTOR_CURRENT, CURRENT_DEADZONE,
    CAN_QUERY_INTERVAL_MS, CAN_HEARTBEAT_INTERVAL_MS,
    CMD_VEL_TIMEOUT_MS, CAN_QUERY_TIMEOUT_MS,
    ODOM_PUBLISH_HZ, MOTOR_TEMP_WARN_C, MOTOR_TEMP_CUTOFF_C,
    # Topics & frames
    TOPIC_CMD_VEL, TOPIC_ODOM, TOPIC_CHASSIS_STATUS,
    FRAME_ODOM, FRAME_BASE_LINK,
)
from cleaning_robot_common.kinematics import (
    diff_decompose, erpm_to_ms, ms_to_current,
    compute_odom_velocity, integrate_odom, yaw_to_quaternion,
)
from cleaning_robot_common.can_protocol import (
    make_current_frame, make_query_frame, parse_query_erpm,
)


class ChassisDriverNode(Node):
    """ROS2 node: CAN → motors, speeds → odometry."""

    def __init__(self) -> None:
        super().__init__("chassis_driver")

        self._declare_params()
        self._read_params()

        # ---- Runtime state ----
        self._v_target = 0.0
        self._w_target = 0.0
        self._last_cmd_time: Optional[float] = None

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._odom_last_time: Optional[float] = None

        # Motor state
        self._motor_speeds = {cid: 0.0 for cid in DRIVE_IDS}

        # CAN state
        self._can_bus = None
        self._can_lock = threading.Lock()
        self._running = False
        self._can_thread: Optional[threading.Thread] = None

        # ---- ROS interfaces ----
        qos_s = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE)
        qos_c = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE)

        self._pub_odom   = self.create_publisher(Odometry, TOPIC_ODOM, qos_s)
        self._pub_status = self.create_publisher(Float32MultiArray, TOPIC_CHASSIS_STATUS, 10)
        self._sub_cmd    = self.create_subscription(
            Twist, TOPIC_CMD_VEL, self._cb_cmd_vel, qos_c)

        from tf2_ros import TransformBroadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # ---- Start ----
        if self._simulate:
            self.get_logger().info("[SIM] chassis_driver running")
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._tick_sim)
        else:
            self._start_can()
            self.create_timer(self._query_interval, self._query_speeds)
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._publish_odom)

    # ---- Parameters ----

    def _declare_params(self) -> None:
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("max_velocity", MAX_LINEAR_VELOCITY)
        self.declare_parameter("max_current", MAX_MOTOR_CURRENT)
        self.declare_parameter("current_deadzone", CURRENT_DEADZONE)
        self.declare_parameter("query_interval_ms", CAN_QUERY_INTERVAL_MS)
        self.declare_parameter("heartbeat_interval_ms", CAN_HEARTBEAT_INTERVAL_MS)
        self.declare_parameter("cmd_vel_timeout_ms", CMD_VEL_TIMEOUT_MS)
        self.declare_parameter("query_timeout_ms", CAN_QUERY_TIMEOUT_MS)
        self.declare_parameter("motor_temp_warn_c", MOTOR_TEMP_WARN_C)
        self.declare_parameter("motor_temp_cutoff_c", MOTOR_TEMP_CUTOFF_C)
        self.declare_parameter("simulate", False)

    def _read_params(self) -> None:
        g = lambda n: self.get_parameter(n).get_parameter_value()
        self._can_iface       = g("can_interface").string_value
        self._max_velocity    = g("max_velocity").double_value
        self._max_current     = g("max_current").integer_value
        self._current_dz      = g("current_deadzone").integer_value
        self._query_interval  = g("query_interval_ms").integer_value / 1000.0
        self._heartbeat_intv  = g("heartbeat_interval_ms").integer_value / 1000.0
        self._cmd_timeout     = g("cmd_vel_timeout_ms").integer_value / 1000.0
        self._query_timeout   = g("query_timeout_ms").integer_value / 1000.0
        self._temp_warn       = g("motor_temp_warn_c").double_value
        self._temp_cutoff     = g("motor_temp_cutoff_c").double_value
        self._simulate        = g("simulate").bool_value

    # ---- cmd_vel subscriber ----

    def _cb_cmd_vel(self, msg) -> None:
        self._v_target = msg.linear.x
        self._w_target = msg.angular.z
        self._last_cmd_time = time.time()

    @property
    def _cmd_timed_out(self) -> bool:
        return (self._last_cmd_time is None or
                (time.time() - self._last_cmd_time) > self._cmd_timeout)

    # ---- CAN bus ----

    def _start_can(self) -> None:
        if not HAS_CAN:
            self.get_logger().error("python-can not installed → sim mode")
            self._simulate = True
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._tick_sim)
            return
        try:
            self._can_bus = can.interface.Bus(channel=self._can_iface, bustype="socketcan")
            self.get_logger().info(f"CAN opened: {self._can_iface}")
        except Exception as e:
            self.get_logger().error(f"CAN open failed: {e} → sim mode")
            self._simulate = True
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._tick_sim)
            return
        self._running = True
        self._can_thread = threading.Thread(target=self._can_loop, daemon=True)
        self._can_thread.start()

    def _can_loop(self) -> None:
        self.get_logger().info("CAN control loop running")
        cycle = 0
        while self._running and rclpy.ok():
            timeout = self._cmd_timed_out
            for cid in DRIVE_IDS:
                if timeout:
                    self._send_frame(*make_current_frame(cid, 0))
                elif cycle % 2 == 0:
                    v_left, v_right = diff_decompose(self._v_target, self._w_target)
                    v_wheel = v_left if cid in LEFT_IDS else v_right
                    cur = ms_to_current(v_wheel, self._max_velocity,
                                        self._max_current, self._current_dz)
                    self._send_frame(*make_current_frame(cid, cur))
                else:
                    self._send_frame(cid, bytes([CAN_CMD_HEARTBEAT]))
                time.sleep(self._heartbeat_intv / len(DRIVE_IDS))
            cycle += 1

    def _send_frame(self, can_id: int, data: bytes) -> None:
        if self._can_bus is None:
            return
        try:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            with self._can_lock:
                self._can_bus.send(msg, timeout=0.001)
        except can.CanError as e:
            self.get_logger().warn(f"CAN send {can_id}: {e}")

    def _query_speeds(self) -> None:
        if self._can_bus is None:
            return
        for cid in DRIVE_IDS:
            self._send_frame(*make_query_frame(cid))
            try:
                resp = self._can_bus.recv(timeout=self._query_timeout)
                erpm = parse_query_erpm(resp.data) if resp else None
                if erpm is not None:
                    self._motor_speeds[cid] = erpm_to_ms(erpm)
            except Exception:
                pass

    # ---- Odometry ----

    def _publish_odom(self) -> None:
        now = time.time()
        dt = (now - self._odom_last_time) if self._odom_last_time else 0.02
        self._odom_last_time = now

        if self._simulate:
            v, w = self._v_target, self._w_target
        else:
            v, w = compute_odom_velocity(self._motor_speeds)

        self._x, self._y, self._theta = integrate_odom(
            self._x, self._y, self._theta, v, w, dt)

        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self._theta)

        # Odom message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = FRAME_ODOM
        odom.child_frame_id = FRAME_BASE_LINK
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        for i in (0, 7, 35):
            odom.pose.covariance[i] = 0.01 if i < 35 else 0.001
            odom.twist.covariance[i] = 0.01 if i < 35 else 0.001
        self._pub_odom.publish(odom)

        # TF
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = FRAME_ODOM
        tf.child_frame_id = FRAME_BASE_LINK
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation = q
        self._tf_broadcaster.sendTransform(tf)

    def _tick_sim(self) -> None:
        self._publish_odom()
        self._pub_status.publish(Float32MultiArray(data=[0.0] * 16))

    # ---- Lifecycle ----

    def destroy_node(self) -> None:
        self._running = False
        if self._can_thread:
            self._can_thread.join(timeout=1.0)
        if self._can_bus:
            self._can_bus.shutdown()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
