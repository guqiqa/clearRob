#!/usr/bin/env python3
"""
chassis_driver/chassis_node.py — CAN bus motor controller (ROS2 node).

Control logic lives in cleaning_robot_common.wheel_speed_loop (dependency-free
so it can be simulated/tested). This node handles only the ROS2 lifecycle,
CAN I/O, and scheduling:

  speed   — driver internal closed-loop (OID 0x02), software slew-rate ramp
  current — software PI + feed-forward on OID current control (0x01)

Speed feedback is read big-endian (matching the driver) — the old
little-endian parse produced garbage RPM / odometry.

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
    DRIVE_IDS, LEFT_IDS, RIGHT_IDS,
    MAX_LINEAR_VELOCITY, MAX_MOTOR_CURRENT, CURRENT_DEADZONE,
    MOTOR_POLE_PAIRS, MOTOR_GEAR_RATIO, WHEEL_RADIUS_M, TRACK_WIDTH_M,
    CAN_QUERY_INTERVAL_MS, CAN_HEARTBEAT_INTERVAL_MS,
    CMD_VEL_TIMEOUT_MS, CAN_QUERY_TIMEOUT_MS,
    ODOM_PUBLISH_HZ, MOTOR_TEMP_WARN_C, MOTOR_TEMP_CUTOFF_C,
    WHEEL_SPEED_PARAMS,
    TOPIC_CMD_VEL, TOPIC_ODOM, TOPIC_CHASSIS_STATUS,
    FRAME_ODOM, FRAME_BASE_LINK,
)
from cleaning_robot_common.kinematics import (
    diff_decompose, integrate_odom, compute_odom_velocity, yaw_to_quaternion,
)
from cleaning_robot_common.can_protocol import (
    make_current_frame, make_query_frame, make_speed_frame,
    make_set_accel_frame, make_set_decel_frame, make_set_max_current_frame,
    parse_query_erpm,
)
from cleaning_robot_common.wheel_speed_loop import WheelSpeedController


class ChassisDriverNode(Node):
    """ROS2 node: CAN → motors, speeds → odometry."""

    def __init__(self) -> None:
        super().__init__("chassis_driver")

        self._declare_params()
        self._read_params()

        # ---- cmd_vel state ----
        self._v_target = 0.0
        self._w_target = 0.0
        self._last_cmd_time: Optional[float] = None

        # ---- Odometry state ----
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._odom_last_time: Optional[float] = None

        # Motor / feedback state
        self._motor_speeds = {cid: 0.0 for cid in DRIVE_IDS}   # m/s (odom feed)
        self._measured_rpm = {cid: 0.0 for cid in DRIVE_IDS}   # physical wheel RPM
        self._feedback_time: dict = {}                          # monotonic ts per cid

        # CAN state
        self._can_bus = None
        self._can_lock = threading.Lock()
        self._running = False
        self._can_thread: Optional[threading.Thread] = None
        self._query_index = 0

        # Scheduler bookkeeping
        self._last_hb = time.monotonic()
        self._last_query = time.monotonic()
        self._last_ctrl = time.monotonic()

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

        # ---- Wheel speed-loop controller (pure logic) ----
        self._controller = WheelSpeedController(self._build_controller_params())

        # ---- Start ----
        if self._simulate:
            self.get_logger().info("[SIM] chassis_driver running")
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._tick_sim)
        else:
            self._start_can()
            self._configure_driver()
            # 10 ms scheduler tick — heartbeat / query / control time-sliced
            self.create_timer(0.01, self._main_tick)
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
        # 左侧电机反装 — 仅用于无PI的旧开环电流路径
        self.declare_parameter("left_motor_invert", True)

        # Kinematics
        self.declare_parameter("pole_pairs", MOTOR_POLE_PAIRS)
        self.declare_parameter("gear_ratio", MOTOR_GEAR_RATIO)
        self.declare_parameter("wheel_radius", WHEEL_RADIUS_M)
        self.declare_parameter("track_width", TRACK_WIDTH_M)

        # Wheel speed-loop (WheelPID V2 port)
        self.declare_parameter("wheel_control_mode", WHEEL_SPEED_PARAMS["wheel_control_mode"])
        self.declare_parameter("speed_accel_rpm_s", WHEEL_SPEED_PARAMS["speed_accel_rpm_s"])
        self.declare_parameter("speed_decel_rpm_s", WHEEL_SPEED_PARAMS["speed_decel_rpm_s"])
        self.declare_parameter("wheel_max_current", WHEEL_SPEED_PARAMS["wheel_max_current"])
        self.declare_parameter("wheel_speed_kp", WHEEL_SPEED_PARAMS["wheel_speed_kp"])
        self.declare_parameter("wheel_speed_ki", WHEEL_SPEED_PARAMS["wheel_speed_ki"])
        self.declare_parameter("wheel_feedforward_current", WHEEL_SPEED_PARAMS["wheel_feedforward_current"])
        self.declare_parameter("wheel_integral_max_current", WHEEL_SPEED_PARAMS["wheel_integral_max_current"])
        self.declare_parameter("wheel_start_initial_current", WHEEL_SPEED_PARAMS["wheel_start_initial_current"])
        self.declare_parameter("wheel_start_step_current", WHEEL_SPEED_PARAMS["wheel_start_step_current"])
        self.declare_parameter("wheel_start_step_ms", WHEEL_SPEED_PARAMS["wheel_start_step_ms"])
        self.declare_parameter("wheel_start_max_current", WHEEL_SPEED_PARAMS["wheel_start_max_current"])
        self.declare_parameter("wheel_start_threshold_rpm", WHEEL_SPEED_PARAMS["wheel_start_threshold_rpm"])
        self.declare_parameter("wheel_start_confirm_samples", WHEEL_SPEED_PARAMS["wheel_start_confirm_samples"])
        self.declare_parameter("wheel_stall_threshold_rpm", WHEEL_SPEED_PARAMS["wheel_stall_threshold_rpm"])
        self.declare_parameter("wheel_stall_confirm_samples", WHEEL_SPEED_PARAMS["wheel_stall_confirm_samples"])
        self.declare_parameter("wheel_start_timeout_ms", WHEEL_SPEED_PARAMS["wheel_start_timeout_ms"])
        self.declare_parameter("wheel_overspeed_rpm", WHEEL_SPEED_PARAMS["wheel_overspeed_rpm"])
        self.declare_parameter("speed_feedback_timeout_ms", WHEEL_SPEED_PARAMS["speed_feedback_timeout_ms"])
        self.declare_parameter("control_interval_ms", WHEEL_SPEED_PARAMS["control_interval_ms"])
        self.declare_parameter("motor_dirs", [-1, -1, 1, 1])
        self.declare_parameter("motor_gains", [1.0, 1.0, 1.0, 1.0])

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
        self._left_invert     = g("left_motor_invert").bool_value

        self._pole_pairs   = g("pole_pairs").integer_value
        self._gear_ratio   = g("gear_ratio").double_value
        self._wheel_radius = g("wheel_radius").double_value
        self._track_width  = g("track_width").double_value

        # cache the ROS-side kinematics so odometry uses the same values
        self._motor_dirs = {}
        dir_array = g("motor_dirs").integer_array_value
        order = [2, 1, 4, 3]
        if len(dir_array) != len(order):
            self.get_logger().error(f"motor_dirs must have {len(order)} entries, got {len(dir_array)}")
            dir_array = [-1, -1, 1, 1]
        self._motor_dirs = {order[i]: dir_array[i] for i in range(len(order))}
        self._motor_gains = {}
        gain_array = g("motor_gains").double_array_value if hasattr(g("motor_gains"), "double_array_value") else None
        if gain_array is None:
            gain_array = g("motor_gains").integer_array_value if hasattr(g("motor_gains"), "integer_array_value") else [1.0]*4
        if len(gain_array) != len(order):
            self.get_logger().error(f"motor_gains must have {len(order)} entries, got {len(gain_array)}")
            gain_array = [1.0]*4
        self._motor_gains = {order[i]: float(gain_array[i]) for i in range(len(order))}

    def _build_controller_params(self) -> dict:
        g = lambda n: self.get_parameter(n).value  # native Python type
        keys = [
            "wheel_control_mode", "speed_accel_rpm_s", "speed_decel_rpm_s",
            "wheel_max_current", "wheel_speed_kp", "wheel_speed_ki",
            "wheel_feedforward_current", "wheel_integral_max_current",
            "wheel_start_initial_current", "wheel_start_step_current",
            "wheel_start_step_ms", "wheel_start_max_current",
            "wheel_start_threshold_rpm", "wheel_start_confirm_samples",
            "wheel_stall_threshold_rpm", "wheel_stall_confirm_samples",
            "wheel_start_timeout_ms", "wheel_overspeed_rpm",
            "speed_feedback_timeout_ms", "control_interval_ms",
        ]
        params = {k: g(k) for k in keys}
        params["pole_pairs"] = self._pole_pairs
        params["gear_ratio"] = self._gear_ratio
        params["wheel_radius"] = self._wheel_radius
        params["track_width"] = self._track_width
        params["motor_dirs"] = self._motor_dirs
        params["motor_gains"] = self._motor_gains
        return params

    # ---- cmd_vel subscriber ----

    def _cb_cmd_vel(self, msg) -> None:
        self._v_target = msg.linear.x
        self._w_target = msg.angular.z
        self._last_cmd_time = time.monotonic()

    @property
    def _cmd_timed_out(self) -> bool:
        return (self._last_cmd_time is None or
                (time.monotonic() - self._last_cmd_time) > self._cmd_timeout)

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
        self._can_thread = threading.Thread(target=self._can_read_loop, daemon=True)
        self._can_thread.start()

    def _send_frame(self, can_id: int, data: bytes) -> None:
        if self._can_bus is None:
            return
        try:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            with self._can_lock:
                self._can_bus.send(msg, timeout=0.001)
        except can.CanError as e:
            self.get_logger().warn(f"CAN send {can_id}: {e}")

    def _can_read_loop(self) -> None:
        """Blocking receiver: parse speed-query responses → update feedback."""
        while self._running and rclpy.ok():
            try:
                resp = self._can_bus.recv(timeout=0.05)
            except can.CanError:
                continue
            if resp is None:
                continue
            erpm = parse_query_erpm(resp.data)
            if erpm is None:
                continue
            cid = resp.arbitration_id
            if cid not in DRIVE_IDS:
                continue
            rpm = self._controller.erpm_to_wheel_rpm(cid, erpm)
            self._measured_rpm[cid] = rpm
            self._feedback_time[cid] = time.monotonic()
            self._motor_speeds[cid] = self._controller.wheel_rpm_to_mps(rpm)

    def _configure_driver(self) -> None:
        """Send driver-side setup once at startup (speed mode)."""
        if self._can_bus is None or self._controller.mode != "speed":
            return
        accel = int(round(self._controller.accel_rpm_s *
                          self._pole_pairs * self._gear_ratio))
        decel = int(round(self._controller.decel_rpm_s *
                          self._pole_pairs * self._gear_ratio))
        for cid in DRIVE_IDS:
            self._send_frame(*make_set_max_current_frame(cid, self._controller.wheel_max_current))
            self._send_frame(*make_set_accel_frame(cid, accel))
            self._send_frame(*make_set_decel_frame(cid, decel))
            self._send_frame(*make_speed_frame(cid, 0))
        self.get_logger().info(
            f"speed mode driver config: max_cur={self._controller.wheel_max_current}x10mA "
            f"accel={accel} decel={decel} erpm/s")

    # ---- Scheduler ----

    def _main_tick(self) -> None:
        now = time.monotonic()
        if now - self._last_hb >= self._heartbeat_intv:
            for cid in DRIVE_IDS:
                self._send_frame(cid, bytes([0x00]))
            self._last_hb = now
        if now - self._last_query >= self._query_interval:
            cid = DRIVE_IDS[self._query_index]
            self._query_index = (self._query_index + 1) % len(DRIVE_IDS)
            self._send_frame(*make_query_frame(cid))
            self._last_query = now
        if now - self._last_ctrl >= self._controller.control_interval_s:
            self._control_tick()
            self._last_ctrl = now

    # ---- Control ----

    def _targets_from_cmd(self):
        v_left, v_right = diff_decompose(self._v_target, self._w_target,
                                         self._track_width)
        # Targets are PHYSICAL wheel RPM (positive = physical forward). The
        # controller converts to motor polarity at its output using motor_dirs
        # (right motors are reverse-mounted: forward = negative current).
        # Do NOT multiply by dir here — that would double-flip the differential
        # term and reverse steering direction.
        return {cid: (self._controller.mps_to_wheel_rpm(v_left) if cid in LEFT_IDS
                      else self._controller.mps_to_wheel_rpm(v_right))
                for cid in DRIVE_IDS}

    def _feedback_fresh(self, cid: int) -> bool:
        t = self._feedback_time.get(cid)
        return t is not None and (time.monotonic() - t) <= self._controller.feedback_timeout_s

    def _control_tick(self) -> None:
        if self._cmd_timed_out:
            for cid, kind, value in self._controller.stop_all():
                self._dispatch(cid, kind, value)
            if self._controller.safety_latched and not self._controller.safety_reported:
                self.get_logger().error(
                    f"WHEEL SAFETY STOP: {self._controller.safety_reason}")
                self._controller.safety_reported = True
            return
        targets = self._targets_from_cmd()
        fresh = {cid: self._feedback_fresh(cid) for cid in DRIVE_IDS}
        for cid, kind, value in self._controller.step(targets, self._measured_rpm, fresh):
            self._dispatch(cid, kind, value)
        if self._controller.safety_latched and not self._controller.safety_reported:
            self.get_logger().error(
                f"WHEEL SAFETY STOP: {self._controller.safety_reason}")
            self._controller.safety_reported = True

    def _dispatch(self, cid: int, kind: str, value: int) -> None:
        if kind == "speed":
            self._send_frame(*make_speed_frame(cid, value))
        elif kind == "current":
            self._send_frame(*make_current_frame(cid, value))

    # ---- Odometry ----

    def _publish_odom(self) -> None:
        now = time.monotonic()
        dt = (now - self._odom_last_time) if self._odom_last_time else 0.02
        self._odom_last_time = now

        if self._simulate:
            v, w = self._v_target, self._w_target
        else:
            v, w = compute_odom_velocity(self._motor_speeds, self._track_width)

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
        for cid, kind, value in self._controller.stop_all():
            self._dispatch(cid, kind, value)
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
