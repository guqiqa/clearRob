#!/usr/bin/env python3
"""
chassis_node.py — ROS2 Python shell for the C++ chassis core.

Architecture (C++ core + Python shell):
  - The field-tuned control logic (steering model, in-place rotation, heading
    hold, direction-change hold, gear selection, per-wheel speed loop) runs in
    the compiled C++ `chassis_core` pybind11 module — a faithful extraction of
    the colleague's WheelPID V2 program.
  - This node owns ALL ROS2 + hardware I/O:
      * subscribes /cmd_vel (Twist, backward compat) and /chassis/intent
        (std_msgs/String JSON, primary multi-source interface)
      * subscribes /imu/data and integrates yaw for heading hold + odometry
      * opens can0 via python-can, schedules heartbeat/query/control, reads
        wheel speed feedback
      * publishes /odom, /chassis/status, tf

Control-source arbitration (highest first):
  1. estop (intent.estop or remote SWB)
  2. /chassis/intent active (received within intent_timeout_ms)
  3. /cmd_vel active (received within cmd_vel_timeout_ms)
  4. failsafe (no active source → zero currents)

Intent JSON:  {"throttle": -1..1, "steering": -1..1, "gear": "LOW|MID|HIGH",
               "mower": 0|1, "estop": 0|1}
"""

import ast
import json
import math
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32MultiArray

try:
    import chassis_core
    HAS_CORE = True
except ImportError:
    HAS_CORE = False

try:
    import can
    HAS_CAN = True
except ImportError:
    HAS_CAN = False

from cleaning_robot_common.config import (
    DRIVE_IDS, LEFT_IDS, RIGHT_IDS,
    MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY,
    MOTOR_POLE_PAIRS, MOTOR_GEAR_RATIO, WHEEL_RADIUS_M, TRACK_WIDTH_M,
    CAN_QUERY_INTERVAL_MS, CAN_HEARTBEAT_INTERVAL_MS,
    CMD_VEL_TIMEOUT_MS,
    ODOM_PUBLISH_HZ,
    CHASSIS_CORE_DEFAULTS,
    TOPIC_CMD_VEL, TOPIC_ODOM, TOPIC_CHASSIS_STATUS,
    TOPIC_IMU_DATA, TOPIC_CHASSIS_INTENT,
    FRAME_ODOM, FRAME_BASE_LINK,
)
from cleaning_robot_common.kinematics import (
    compute_odom_velocity, integrate_odom, yaw_to_quaternion,
)
from cleaning_robot_common.can_protocol import (
    make_current_frame, make_query_frame, parse_query_erpm,
)

# C++ core motor order: FL, RL, FR, RR = CAN ids 2, 1, 4, 3.
CORE_ORDER = [2, 1, 4, 3]

GEARS = ("LOW", "MID", "HIGH")


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ChassisDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_driver")

        self._declare_params()
        self._read_params()

        # ---- intent / cmd_vel state ----
        self._intent = {"throttle": 0.0, "steering": 0.0, "gear": "MID",
                        "mower": 0, "estop": 0}
        self._intent_time: Optional[float] = None
        self._v_target = 0.0
        self._w_target = 0.0
        self._cmd_time: Optional[float] = None
        self._estop = False

        # ---- wheel feedback (physical wheel RPM, ordered CORE_ORDER) ----
        self._measured_rpm: Dict[int, float] = {cid: 0.0 for cid in DRIVE_IDS}
        self._feedback_time: Dict[int, float] = {}
        self._generation: Dict[int, int] = {cid: 0 for cid in DRIVE_IDS}
        self._motor_speeds: Dict[int, float] = {cid: 0.0 for cid in DRIVE_IDS}  # m/s

        # ---- IMU yaw integration ----
        self._yaw_deg = 0.0
        self._yaw_rate_dps = 0.0
        self._imu_valid = False
        self._imu_last = None
        self._calib_sum = 0.0
        self._calib_count = 0
        self._calib_start: Optional[float] = None
        self._calib_bias_dps = 0.0
        self._calibrated = False

        # ---- odometry ----
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._odom_last_time: Optional[float] = None

        # ---- ROS interfaces ----
        qos_s = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE)
        qos_c = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE)
        self._pub_odom = self.create_publisher(Odometry, TOPIC_ODOM, qos_s)
        self._pub_status = self.create_publisher(Float32MultiArray, TOPIC_CHASSIS_STATUS, 10)
        self._sub_cmd = self.create_subscription(
            Twist, TOPIC_CMD_VEL, self._cb_cmd_vel, qos_c)
        self._sub_intent = self.create_subscription(
            String, TOPIC_CHASSIS_INTENT, self._cb_intent, 10)
        self._sub_imu = self.create_subscription(
            Imu, TOPIC_IMU_DATA, self._cb_imu, 10)
        from tf2_ros import TransformBroadcaster
        self._tf = TransformBroadcaster(self)

        # ---- C++ chassis core ----
        if not HAS_CORE:
            self.get_logger().error("chassis_core module not importable — "
                                    "falling back to pure Python open-loop sim")
            self._core = None
            self._simulate = True
        else:
            try:
                self._core = chassis_core.ChassisCore(self._core_cfg)
                self.get_logger().info(
                    f"chassis_core loaded: mode={self._core_cfg.get('wheel_control_mode')} "
                    f"run_rpm={self._core_cfg.get('run_rpm')} "
                    f"dirs={self._core_cfg.get('motor_dirs')}")
            except Exception as e:
                self.get_logger().error(f"ChassisCore init failed: {e}")
                self._core = None
                self._simulate = True

        # ---- Hardware ----
        self._can_bus = None
        self._can_lock = threading.Lock()
        self._running = False
        self._can_thread: Optional[threading.Thread] = None
        self._query_index = 0
        self._last_hb = time.monotonic()
        self._last_query = time.monotonic()
        self._last_ctrl = time.monotonic()

        if self._simulate:
            self.get_logger().warn("[SIM] chassis_driver running WITHOUT C++ core / CAN")
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._tick_sim)
        else:
            self._start_can()
            self.create_timer(self._heartbeat_intv, self._tick_hb)
            self.create_timer(self._query_intv, self._tick_query)
            self.create_timer(self._ctrl_intv, self._tick_ctrl)
            self.create_timer(1.0 / ODOM_PUBLISH_HZ, self._publish_odom)

    # ---- parameters ----

    def _declare_params(self) -> None:
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("max_velocity", MAX_LINEAR_VELOCITY)
        self.declare_parameter("max_angular_velocity", MAX_ANGULAR_VELOCITY)
        self.declare_parameter("cmd_vel_timeout_ms", CMD_VEL_TIMEOUT_MS)
        self.declare_parameter("intent_timeout_ms", 300)
        self.declare_parameter("simulate", False)
        # One ROS2 param per chassis_core key (nested under chassis_core: in yaml)
        for key, default in CHASSIS_CORE_DEFAULTS.items():
            self.declare_parameter(f"chassis_core.{key}", default)

    def _read_params(self) -> None:
        g = lambda n: self.get_parameter(n).value
        self._can_iface = g("can_interface")
        self._max_v = float(g("max_velocity"))
        self._max_w = float(g("max_angular_velocity"))
        self._cmd_timeout = int(g("cmd_vel_timeout_ms")) / 1000.0
        self._intent_timeout = int(g("intent_timeout_ms")) / 1000.0
        self._simulate = bool(g("simulate"))

        self._core_cfg = {}
        for key in CHASSIS_CORE_DEFAULTS:
            self._core_cfg[key] = g(f"chassis_core.{key}")

        self._heartbeat_intv = int(self._core_cfg.get("heartbeat_ms", 200)) / 1000.0
        self._query_intv = int(self._core_cfg.get("speed_query_ms", 50)) / 1000.0
        self._ctrl_intv = int(self._core_cfg.get("control_ms", 50)) / 1000.0
        self._feedback_timeout = int(self._core_cfg.get("speed_feedback_timeout_ms", 1000)) / 1000.0
        self._pp = int(self._core_cfg.get("pole_pairs", MOTOR_POLE_PAIRS))
        self._gr = float(self._core_cfg.get("gear_ratio", MOTOR_GEAR_RATIO))
        self._wheel_r = float(self._core_cfg.get("wheel_radius", WHEEL_RADIUS_M))
        self._track_w = float(self._core_cfg.get("track_width", TRACK_WIDTH_M))
        # motor dirs keyed by CAN id — MUST match the C++ core config
        dirs = self._core_cfg.get("motor_dirs", [1, 1, -1, -1])
        self._dirs = {CORE_ORDER[i]: int(dirs[i]) for i in range(min(4, len(dirs)))}

    # ---- subscribers ----

    def _cb_cmd_vel(self, msg: Twist) -> None:
        self._v_target = msg.linear.x
        self._w_target = msg.angular.z
        self._cmd_time = time.monotonic()

    def _cb_intent(self, msg: String) -> None:
        try:
            data = msg.data
            if not data:
                return
            # `ros2 topic pub` mangles strict JSON into a Python-dict literal
            # ('{"throttle": 0.5}' → {'throttle': 0.5}).  Accept both forms so
            # manual terminal commands work without escaping gymnastics.
            try:
                d = json.loads(data)
            except json.JSONDecodeError:
                d = ast.literal_eval(data)
        except (json.JSONDecodeError, TypeError, ValueError, SyntaxError):
            self.get_logger().warn(f"bad intent JSON: {msg.data!r}")
            return
        self._intent["throttle"] = _clamp(float(d.get("throttle", 0.0)), -1.0, 1.0)
        self._intent["steering"] = _clamp(float(d.get("steering", 0.0)), -1.0, 1.0)
        gear = str(d.get("gear", "MID")).upper()
        self._intent["gear"] = gear if gear in GEARS else "MID"
        self._intent["mower"] = 1 if d.get("mower") else 0
        self._intent["estop"] = 1 if d.get("estop") else 0
        self._intent_time = time.monotonic()

    def _cb_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        # gyro z in rad/s → deg/s, with sign convention from config
        rate = msg.angular_velocity.z * (180.0 / math.pi) * float(self._core_cfg.get("imu_yaw_sign", -1.0))
        if self._calibrated:
            rate -= self._calib_bias_dps
            if abs(rate) < float(self._core_cfg.get("imu_deadband_dps", 0.30)):
                rate = 0.0
            if self._imu_last is not None:
                dt = now - self._imu_last
                if 0.0 < dt < 0.5:
                    self._yaw_deg += rate * dt
            self._yaw_rate_dps = rate
        else:
            # stationary calibration: average the rate for the calibration window
            if self._calib_start is None:
                self._calib_start = now
            self._calib_sum += rate
            self._calib_count += 1
            calib_dur = float(self._core_cfg.get("imu_calibrate_ms", 3000)) / 1000.0
            if now - self._calib_start >= calib_dur:
                self._calib_bias_dps = self._calib_sum / max(self._calib_count, 1)
                self._calibrated = True
                self._imu_valid = True
                self._yaw_deg = 0.0
                self.get_logger().info(f"IMU calibrated: bias={self._calib_bias_dps:.3f} deg/s "
                                       f"({self._calib_count} samples)")
        self._imu_last = now

    # ---- CAN ----

    def _start_can(self) -> None:
        if not HAS_CAN:
            self.get_logger().error("python-can not installed → sim mode")
            self._simulate = True
            return
        try:
            self._can_bus = can.interface.Bus(channel=self._can_iface, bustype="socketcan")
            self.get_logger().info(f"CAN opened: {self._can_iface}")
        except Exception as e:
            self.get_logger().error(f"CAN open failed: {e} → sim mode")
            self._simulate = True
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
            # erpm → PHYSICAL wheel rpm (matches colleague main loop)
            rpm = erpm * self._dirs.get(cid, 1) / (self._pp * self._gr)
            self._measured_rpm[cid] = rpm
            self._feedback_time[cid] = time.monotonic()
            self._generation[cid] += 1  # new_feedback signal for the C++ core
            self._motor_speeds[cid] = rpm * 2.0 * math.pi * self._wheel_r / 60.0

    # ---- timers ----

    def _tick_hb(self) -> None:
        if self._can_bus is None:
            return
        for cid in DRIVE_IDS:
            self._send_frame(cid, bytes([0x00]))

    def _tick_query(self) -> None:
        if self._can_bus is None:
            return
        cid = DRIVE_IDS[self._query_index]
        self._query_index = (self._query_index + 1) % len(DRIVE_IDS)
        self._send_frame(*make_query_frame(cid))

    def _tick_ctrl(self) -> None:
        if self._core is None:
            return
        # ---- arbitration ----
        estop, throttle, steering, gear, mower = self._resolve_command()
        # ---- feedback arrays (CORE_ORDER) ----
        now = time.monotonic()
        rpm = []
        valid = []
        gen = []
        for cid in CORE_ORDER:
            rpm.append(self._measured_rpm[cid])
            valid.append((now - self._feedback_time.get(cid, 0.0)) <= self._feedback_timeout)
            gen.append(self._generation[cid])
        # ---- tick the C++ core ----
        currents, safety, reason = self._core.tick(
            throttle, steering, gear, bool(mower), bool(estop),
            rpm, valid, gen, self._yaw_deg, self._yaw_rate_dps)
        # ---- send CAN currents ----
        for i, cid in enumerate(CORE_ORDER):
            self._send_frame(*make_current_frame(cid, int(currents[i])))
        # A safety latch is a transient event — log it, don't permanently glue
        # _estop (that would leave the chassis in failsafe forever).  The C++
        # core clears its own latch on the next stopped tick; the operator
        # clears estop via intent/RC.
        if safety:
            self.get_logger().error(f"CHASSIS SAFETY: {reason}")

    def _resolve_command(self):
        """Priority: estop > intent active > cmd_vel active > failsafe."""
        now = time.monotonic()
        # 1. estop — sticky until intent explicitly clears it
        if self._intent.get("estop"):
            self._estop = True
        elif self._intent_time is not None and (now - self._intent_time) <= self._intent_timeout:
            if not self._intent.get("estop") and self._estop:
                self._estop = False
        # 2. intent
        if self._intent_time is not None and (now - self._intent_time) <= self._intent_timeout:
            return (self._estop, self._intent["throttle"], self._intent["steering"],
                    self._intent["gear"], self._intent["mower"])
        # 3. cmd_vel (backward compat)
        if self._cmd_time is not None and (now - self._cmd_time) <= self._cmd_timeout:
            v = _clamp(self._v_target / max(self._max_v, 0.01), -1.0, 1.0)
            w = _clamp(self._w_target / max(self._max_w, 0.01), -1.0, 1.0)
            return (self._estop, v, w, "MID", 0)
        # 4. failsafe
        if self._estop:
            return (True, 0.0, 0.0, "MID", 0)
        return (False, 0.0, 0.0, "MID", 0)

    # ---- odometry ----

    def _publish_odom(self) -> None:
        now = time.monotonic()
        dt = (now - self._odom_last_time) if self._odom_last_time else 0.02
        self._odom_last_time = now

        if self._simulate or self._core is None:
            v, w = self._v_target, self._w_target
        else:
            v, w = compute_odom_velocity(self._motor_speeds, self._track_w)

        self._x, self._y, self._theta = integrate_odom(
            self._x, self._y, self._theta, v, w, dt)
        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self._theta)

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

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = FRAME_ODOM
        tf.child_frame_id = FRAME_BASE_LINK
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation = q
        self._tf.sendTransform(tf)

        self._pub_status.publish(Float32MultiArray(data=[
            self._measured_rpm.get(2, 0.0), self._measured_rpm.get(1, 0.0),
            self._measured_rpm.get(4, 0.0), self._measured_rpm.get(3, 0.0),
            float(self._yaw_deg), float(self._yaw_rate_dps),
            float(self._estop), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    def _tick_sim(self) -> None:
        self._publish_odom()

    # ---- lifecycle ----

    def destroy_node(self) -> None:
        self._running = False
        if self._core is not None:
            try:
                currents = self._core.stop_all()
                for i, cid in enumerate(CORE_ORDER):
                    self._send_frame(*make_current_frame(cid, int(currents[i])))
            except Exception:
                pass
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
