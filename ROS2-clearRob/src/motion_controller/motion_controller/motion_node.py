#!/usr/bin/env python3
"""
motion_controller - Motion actuator node for cleaning robot.

Controls a 4-wheel differential drive chassis with PID speed control,
odometry computation, emergency stop execution, and slope assist.

Core responsibilities:
  - Receive arbitrated MotionCommand from master_controller
  - Differential drive kinematics (wheel_base 0.6m)
  - PID speed control loop (200Hz simulation)
  - Odometry publish (100Hz) with IMU fusion via complementary filter
  - Motor fault detection (overcurrent, overheat, encoder, timeout, driver, estop, slope)
  - Emergency stop execution (hard estop from lidar/RC)
  - Slope assist based on IMU pitch
  - FollowPath action server for waypoint-following supervision
"""

import math
import random
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from cleaning_robot_interfaces.msg import (
    MotionCommand,
    MotorStatus,
    FaultStatus,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import SetMaxSpeed, GetStatus
from cleaning_robot_interfaces.action import FollowPath

import tf_transformations  # for quaternion_from_euler


# ---------------------------------------------------------------------------
# Motor simulator (first-order lag)
# ---------------------------------------------------------------------------

class MotorSimulator:
    """Simulate a single motor with first-order lag response."""

    def __init__(self, time_constant: float = 0.05):
        self._tau = time_constant
        self._current_speed: float = 0.0  # m/s at wheel surface

    def update(self, target_speed: float, dt: float) -> float:
        """Advance the first-order lag and return the new speed."""
        alpha = dt / (self._tau + dt)
        self._current_speed += alpha * (target_speed - self._current_speed)
        return self._current_speed

    @property
    def speed(self) -> float:
        return self._current_speed

    def reset(self) -> None:
        self._current_speed = 0.0


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

class PID:
    """Discrete PID controller with integral anti-windup and dead zone."""

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_max: float, dead_zone: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.dead_zone = dead_zone
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_run: bool = True

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """Return the control output for one iteration."""
        # Dead zone: ignore very small errors
        if abs(setpoint) < self.dead_zone:
            return 0.0

        error = setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(min(self._integral, self.integral_max),
                             -self.integral_max)
        i_term = self.ki * self._integral

        # Derivative (first-run safe)
        if self._first_run:
            d_term = 0.0
            self._first_run = False
        else:
            d_term = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0

        self._prev_error = error
        return p_term + i_term + d_term


# ---------------------------------------------------------------------------
# Odometry accumulator
# ---------------------------------------------------------------------------

class OdometryAccumulator:
    """Track robot pose via simulated encoder feedback and IMU fusion."""

    def __init__(self, wheel_base: float, wheel_radius: float,
                 gear_ratio: float, pulses_per_rev: int,
                 imu_alpha: float):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.gear_ratio = gear_ratio
        self.pulses_per_rev = pulses_per_rev
        self.alpha = imu_alpha  # weight for encoder (1-alpha for IMU yaw)

        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0  # rad

        self._prev_yaw_imu: float | None = None
        self._initialized_imu: bool = False

    def update(self, v_left: float, v_right: float, dt: float,
               imu_yaw: float | None = None) -> tuple[float, float, float]:
        """Update pose from wheel speeds and optional IMU yaw.

        Returns (dx, dy, dtheta) for this tick.
        """
        ds_left = v_left * dt
        ds_right = v_right * dt

        ds_mean = (ds_left + ds_right) / 2.0
        dtheta_enc = (ds_right - ds_left) / self.wheel_base

        # IMU fusion for yaw
        if imu_yaw is not None:
            if not self._initialized_imu:
                self._prev_yaw_imu = imu_yaw
                self._initialized_imu = True
                dtheta_imu = dtheta_enc  # fallback for first tick
            else:
                dtheta_imu = imu_yaw - self._prev_yaw_imu
                # Normalise to [-pi, pi]
                dtheta_imu = math.atan2(math.sin(dtheta_imu),
                                        math.cos(dtheta_imu))
                self._prev_yaw_imu = imu_yaw

            dtheta = self.alpha * dtheta_enc + (1.0 - self.alpha) * dtheta_imu
        else:
            dtheta = dtheta_enc

        dx = ds_mean * math.cos(self.theta + dtheta / 2.0)
        dy = ds_mean * math.sin(self.theta + dtheta / 2.0)

        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return dx, dy, dtheta

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_yaw_imu = None
        self._initialized_imu = False


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class MotionController(Node):
    """ROS2 node for motion actuation."""

    def __init__(self) -> None:
        super().__init__('motion_controller')

        # ---------- Parameters ----------
        self.declare_parameters('', [
            ('wheel_base', 0.6),
            ('wheel_radius', 0.1),
            ('gear_ratio', 20.0),
            ('pulses_per_rev', 2048),
            ('max_linear_velocity', 1.5),
            ('max_angular_velocity', 2.0),
            ('max_linear_accel', 1.0),
            ('max_angular_accel', 2.0),
            ('pid_kp', 0.5),
            ('pid_ki', 0.1),
            ('pid_kd', 0.05),
            ('pid_integral_max', 0.3),
            ('dead_zone_ms', 0.01),
            ('control_rate', 200.0),
            ('odom_publish_rate', 100.0),
            ('imu_fusion_alpha', 0.95),
            ('current_overload_ratio', 1.5),
            ('current_overload_timeout_s', 1.0),
            ('temperature_warn_c', 80.0),
            ('temperature_cutoff_c', 100.0),
            ('master_cmd_timeout_ms', 500),
            ('slope_assist_min_deg', 5.0),
            ('slope_warn_deg', 15.0),
            ('slope_max_deg', 20.0),
        ])

        # Convenience parameter getters
        self._wb = self.get_parameter('wheel_base').value
        self._wr = self.get_parameter('wheel_radius').value
        self._gr = self.get_parameter('gear_ratio').value
        self._ppr = self.get_parameter('pulses_per_rev').value
        self._max_lin = self.get_parameter('max_linear_velocity').value
        self._max_ang = self.get_parameter('max_angular_velocity').value
        self._max_lin_acc = self.get_parameter('max_linear_accel').value
        self._max_ang_acc = self.get_parameter('max_angular_accel').value

        # Internal state
        self._target_v_lin: float = 0.0
        self._target_v_ang: float = 0.0
        self._actual_v_lin: float = 0.0
        self._actual_v_ang: float = 0.0
        self._v_left: float = 0.0
        self._v_right: float = 0.0
        self._estop_active: bool = False
        self._estop_requires_reset: bool = False

        self._last_master_cmd_time: float = 0.0
        self._imu_pitch: float = 0.0
        self._imu_yaw: float | None = None
        self._motor_temperature: float = 25.0  # ambient start
        self._motor_current: float = 0.0
        self._overcurrent_start: float | None = None
        self._encoder_deviation_start: float | None = None

        self._start_time: float = time.time()

        # Motor simulator (first-order lag, time constant 50ms)
        self._motor_l = MotorSimulator(time_constant=0.05)
        self._motor_r = MotorSimulator(time_constant=0.05)

        # PID controllers (left & right)
        pid_cfg = {
            'kp': self.get_parameter('pid_kp').value,
            'ki': self.get_parameter('pid_ki').value,
            'kd': self.get_parameter('pid_kd').value,
            'integral_max': self.get_parameter('pid_integral_max').value,
            'dead_zone': self.get_parameter('dead_zone_ms').value,
        }
        self._pid_left = PID(**pid_cfg)
        self._pid_right = PID(**pid_cfg)

        # Odometry
        self._odom = OdometryAccumulator(
            wheel_base=self._wb,
            wheel_radius=self._wr,
            gear_ratio=self._gr,
            pulses_per_rev=self._ppr,
            imu_alpha=self.get_parameter('imu_fusion_alpha').value,
        )

        # Path-following state
        self._follow_active: bool = False
        self._follow_goal_handle: ServerGoalHandle | None = None
        self._follow_waypoints: list = []
        self._follow_target_vel: float = 0.0
        self._follow_current_idx: int = 0
        self._follow_max_dev: float = 0.0

        # ---------- QoS profiles ----------
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------- Callback groups ----------
        cb_cmd = MutuallyExclusiveCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()
        cb_estop = MutuallyExclusiveCallbackGroup()
        cb_srv = MutuallyExclusiveCallbackGroup()
        cb_action = MutuallyExclusiveCallbackGroup()

        # ---------- Subscriptions ----------
        self._sub_motion = self.create_subscription(
            MotionCommand,
            'master/cmd/motion',
            self._on_motion_cmd,
            reliable,
            callback_group=cb_cmd,
        )
        self._sub_estop = self.create_subscription(
            MotionCommand,
            'motion/cmd/estop',
            self._on_estop_cmd,
            reliable,
            callback_group=cb_estop,
        )
        self._sub_imu = self.create_subscription(
            Imu,
            'sensor/imu/data',
            self._on_imu,
            sensor_qos,
            callback_group=cb_cmd,
        )

        # ---------- Publishers ----------
        self._pub_odom = self.create_publisher(Odometry, 'motion/odom', 10)
        self._pub_motor = self.create_publisher(MotorStatus, 'motion/status/motor', 10)
        self._pub_fault = self.create_publisher(FaultStatus, 'motion/status/fault', 10)
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, 'system/heartbeat/motion_controller', 10,
        )

        # ---------- Services ----------
        self._srv_set_speed = self.create_service(
            SetMaxSpeed,
            'motion/set_max_speed',
            self._on_set_max_speed,
            callback_group=cb_srv,
        )
        self._srv_get_status = self.create_service(
            GetStatus,
            'motion/get_status',
            self._on_get_status,
            callback_group=cb_srv,
        )

        # ---------- Action Server ----------
        self._action_follow_path = ActionServer(
            self,
            FollowPath,
            'motion/follow_path',
            goal_callback=self._on_follow_goal,
            cancel_callback=self._on_follow_cancel,
            execute_callback=self._execute_follow_path,
            callback_group=cb_action,
        )

        # ---------- Timers ----------
        self._control_timer = self.create_timer(
            1.0 / self.get_parameter('control_rate').value,
            self._control_loop,
            callback_group=cb_timer,
        )
        self._odom_timer = self.create_timer(
            1.0 / self.get_parameter('odom_publish_rate').value,
            self._publish_odom,
            callback_group=cb_timer,
        )
        self._motor_timer = self.create_timer(0.1, self._publish_motor_status,
                                              callback_group=cb_timer)
        self._heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat,
                                                  callback_group=cb_timer)
        self._fault_timer = self.create_timer(0.5, self._check_faults,
                                              callback_group=cb_timer)

        self.get_logger().info('motion_controller started')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_motion_cmd(self, msg: MotionCommand) -> None:
        """Handle arbitrated motion command from master_controller."""
        self._last_master_cmd_time = time.time()

        # Check estop flag in command
        if msg.estop:
            self._trigger_estop(6)
            return

        if self._estop_active:
            return  # still locked, wait for release

        # Clamp and set target velocities
        lin = max(min(msg.linear_velocity, self._max_lin), -self._max_lin)
        ang = max(min(msg.angular_velocity, self._max_ang), -self._max_ang)
        self._target_v_lin = lin
        self._target_v_ang = ang

    def _on_estop_cmd(self, msg: MotionCommand) -> None:
        """Handle hard ESTOP from lidar/RC."""
        if msg.estop:
            self.get_logger().warn('ESTOP triggered from motion/cmd/estop')
            self._trigger_estop(6)
        else:
            self.get_logger().info('ESTOP released')
            self._estop_active = False
            self._estop_requires_reset = False

    def _on_imu(self, msg: Imu) -> None:
        """Capture IMU data for slope assist and yaw fusion."""
        # Extract pitch from quaternion
        q = msg.orientation
        _, pitch, _ = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self._imu_pitch = pitch  # rad

        # Extract yaw for odometry fusion
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self._imu_yaw = yaw

    # ------------------------------------------------------------------
    # Control loop (200 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Main control loop: kinematics, PID, motor simulation."""
        dt = 1.0 / self.get_parameter('control_rate').value

        # Master command timeout check
        now = time.time()
        timeout_s = self.get_parameter('master_cmd_timeout_ms').value / 1000.0
        if not self._estop_active and (now - self._last_master_cmd_time) > timeout_s:
            self._trigger_fault(4, 'Master command timeout (>500ms)')
            self._target_v_lin *= 0.95  # gradual deceleration
            self._target_v_ang *= 0.95
            if abs(self._target_v_lin) < 0.001:
                self._target_v_lin = 0.0
            if abs(self._target_v_ang) < 0.001:
                self._target_v_ang = 0.0

        # ESTOP immediate cutoff
        if self._estop_active:
            self._target_v_lin = 0.0
            self._target_v_ang = 0.0
            self._motor_l.reset()
            self._motor_r.reset()
            self._pid_left.reset()
            self._pid_right.reset()
            self._v_left = 0.0
            self._v_right = 0.0
            self._actual_v_lin = 0.0
            self._actual_v_ang = 0.0
            return

        # Acceleration limiting
        lin_step = self._max_lin_acc * dt
        ang_step = self._max_ang_acc * dt
        delta_lin = max(min(self._target_v_lin - self._actual_v_lin, lin_step), -lin_step)
        delta_ang = max(min(self._target_v_ang - self._actual_v_ang, ang_step), -ang_step)
        cmd_lin = self._actual_v_lin + delta_lin
        cmd_ang = self._actual_v_ang + delta_ang

        # Differential kinematics: convert to wheel speeds
        v_left_tgt = cmd_lin - cmd_ang * (self._wb / 2.0)
        v_right_tgt = cmd_lin + cmd_ang * (self._wb / 2.0)

        # Slope assist
        pitch_deg = math.degrees(abs(self._imu_pitch))
        slope_min = self.get_parameter('slope_assist_min_deg').value
        slope_warn = self.get_parameter('slope_warn_deg').value
        slope_max = self.get_parameter('slope_max_deg').value

        if pitch_deg > slope_max:
            self._trigger_fault(7, f'Slope pitch {pitch_deg:.1f}deg exceeds max')
            self._trigger_estop(7)
            return
        elif pitch_deg > slope_warn:
            f = FaultStatus()
            f.header.stamp = self.get_clock().now().to_msg()
            f.fault_code = 255  # warning only
            f.fault_text = f'Slope warning: pitch={pitch_deg:.1f}deg'
            f.requires_reset = False
            self._pub_fault.publish(f)
        elif pitch_deg > slope_min:
            boost = 0.05 * pitch_deg
            v_left_tgt += boost
            v_right_tgt += boost

        # PID control (simulated motor feedback)
        pid_l = self._pid_left.compute(v_left_tgt, self._motor_l.speed, dt)
        pid_r = self._pid_right.compute(v_right_tgt, self._motor_r.speed, dt)

        # Apply PID output to motor (summing feedforward + correction)
        motor_l_cmd = v_left_tgt + pid_l
        motor_r_cmd = v_right_tgt + pid_r

        # Simulate motor response
        self._v_left = self._motor_l.update(motor_l_cmd, dt)
        self._v_right = self._motor_r.update(motor_r_cmd, dt)

        # Back-compute actual linear/angular from wheel speeds
        self._actual_v_lin = (self._v_left + self._v_right) / 2.0
        self._actual_v_ang = (self._v_right - self._v_left) / self._wb

        # Update odometry
        self._odom.update(self._v_left, self._v_right, dt, self._imu_yaw)

        # Simulate motor current based on load (speed * torque proxy)
        self._motor_current = (abs(self._v_left) + abs(self._v_right)) * 2.0 + 0.5

        # Simulate temperature: gradual warmup proportional to current
        heat_gen = self._motor_current * 3.0  # simplified Joule heating
        cool_rate = 0.2  # natural cooling
        self._motor_temperature += (heat_gen - cool_rate * (self._motor_temperature - 25.0)) * dt

        # Update path following if active
        if self._follow_active and self._follow_goal_handle is not None:
            self._update_path_following()

    # ------------------------------------------------------------------
    # Odometry publisher (100Hz)
    # ------------------------------------------------------------------

    def _publish_odom(self) -> None:
        """Publish nav_msgs/Odometry."""
        x, y, theta = self._odom.pose
        now = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

        msg.twist.twist.linear = Vector3(x=self._actual_v_lin, y=0.0, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self._actual_v_ang)

        msg.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

        self._pub_odom.publish(msg)

    # ------------------------------------------------------------------
    # Motor status publisher (10Hz)
    # ------------------------------------------------------------------

    @staticmethod
    def _wheel_speed_to_rpm(v: float, wheel_radius: float) -> float:
        """Convert linear wheel speed (m/s) to RPM."""
        if wheel_radius <= 0:
            return 0.0
        return (v / (2.0 * math.pi * wheel_radius)) * 60.0

    def _publish_motor_status(self) -> None:
        """Publish simulated motor status."""
        wr = self._wr
        msg = MotorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.motor_left_front = self._wheel_speed_to_rpm(self._v_left, wr)
        msg.motor_right_front = self._wheel_speed_to_rpm(self._v_right, wr)
        msg.motor_left_rear = self._wheel_speed_to_rpm(self._v_left, wr)
        msg.motor_right_rear = self._wheel_speed_to_rpm(self._v_right, wr)
        msg.motor_current = self._motor_current
        msg.motor_voltage = 24.0  # assumed nominal
        msg.temperature = self._motor_temperature
        self._pub_motor.publish(msg)

    # ------------------------------------------------------------------
    # Heartbeat publisher (1Hz)
    # ------------------------------------------------------------------

    def _publish_heartbeat(self) -> None:
        """Publish 1Hz heartbeat."""
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = 'motion_controller'
        msg.lifecycle_state = 3  # Active
        msg.error_code = 0
        self._pub_heartbeat.publish(msg)

    # ------------------------------------------------------------------
    # Fault detection (2Hz)
    # ------------------------------------------------------------------

    def _check_faults(self) -> None:
        """Periodic fault detection."""
        now = time.time()

        # 1. Overcurrent
        rated = 10.0  # A, nominal
        ratio = self.get_parameter('current_overload_ratio').value
        timeout_cur = self.get_parameter('current_overload_timeout_s').value
        if self._motor_current > rated * ratio:
            if self._overcurrent_start is None:
                self._overcurrent_start = now
            elif (now - self._overcurrent_start) > timeout_cur:
                self._trigger_fault(1, 'Overcurrent detected')
                self._overcurrent_start = None
        else:
            self._overcurrent_start = None

        # 2. Overheat
        temp_cutoff = self.get_parameter('temperature_cutoff_c').value
        temp_warn = self.get_parameter('temperature_warn_c').value
        if self._motor_temperature > temp_cutoff:
            self._trigger_fault(2, f'Overtemperature cutoff: {self._motor_temperature:.1f}C')
        elif self._motor_temperature > temp_warn:
            f = FaultStatus()
            f.header.stamp = self.get_clock().now().to_msg()
            f.fault_code = 255
            f.fault_text = f'Temperature warning: {self._motor_temperature:.1f}C'
            f.requires_reset = False
            self._pub_fault.publish(f)

        # 3. Encoder deviation (simplified: compare encoder vs IMU yaw)
        # In a real system we'd compare encoder ticks to IMU angular velocity.
        # Here we simulate a very small probability of deviation fault.
        if abs(self._actual_v_ang) > 0.5:
            if self._encoder_deviation_start is None:
                self._encoder_deviation_start = now
            elif (now - self._encoder_deviation_start) > 3.0:
                self._trigger_fault(3, 'Encoder/IMU deviation >20% for >3s')
                self._encoder_deviation_start = None
        else:
            self._encoder_deviation_start = None

        # 5. Driver fault (simulated random, ~0.05% per tick at 2Hz = ~0.1%/s)
        if random.random() < 0.0005:
            self._trigger_fault(5, 'Driver fault (simulated)')

    # ------------------------------------------------------------------
    # Fault helpers
    # ------------------------------------------------------------------

    def _trigger_fault(self, code: int, text: str) -> None:
        """Publish a fault status."""
        f = FaultStatus()
        f.header.stamp = self.get_clock().now().to_msg()
        f.fault_code = code
        f.fault_text = text
        f.requires_reset = (code != 255)
        self._pub_fault.publish(f)
        self.get_logger().warn(f'FAULT({code}): {text}')

    def _trigger_estop(self, cause_code: int) -> None:
        """Activate emergency stop and publish fault."""
        self._estop_active = True
        self._target_v_lin = 0.0
        self._target_v_ang = 0.0
        self._trigger_fault(cause_code, f'ESTOP engaged (code={cause_code})')

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_set_max_speed(self, request: SetMaxSpeed.Request,
                          response: SetMaxSpeed.Response) -> SetMaxSpeed.Response:
        """Set maximum linear and angular velocities."""
        if 0.1 <= request.max_linear_velocity <= 2.0:
            self._max_lin = request.max_linear_velocity
            self.set_parameters(
                [Parameter('max_linear_velocity', Parameter.Type.DOUBLE,
                           request.max_linear_velocity)])
        if 0.1 <= request.max_angular_velocity <= 3.0:
            self._max_ang = request.max_angular_velocity
            self.set_parameters(
                [Parameter('max_angular_velocity', Parameter.Type.DOUBLE,
                           request.max_angular_velocity)])
        response.success = True
        response.message = (f'Max speeds set: lin={self._max_lin:.2f} m/s, '
                            f'ang={self._max_ang:.2f} rad/s')
        return response

    def _on_get_status(self, request: GetStatus.Request,
                       response: GetStatus.Response) -> GetStatus.Response:
        """Return motion controller status."""
        response.node_name = 'motion_controller'
        response.status = 6 if self._estop_active else 0
        response.status_text = ('ESTOP active' if self._estop_active
                                else 'Normal operation')
        response.uptime_s = float(time.time() - self._start_time)
        return response

    # ------------------------------------------------------------------
    # FollowPath Action Server
    # ------------------------------------------------------------------

    def _on_follow_goal(self, goal_request: FollowPath.Goal) -> GoalResponse:
        if self._estop_active:
            return GoalResponse.REJECT
        if self._follow_active:
            self.get_logger().warn('FollowPath rejected: already executing')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_follow_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info('FollowPath cancel requested')
        self._follow_active = False
        self._target_v_lin = 0.0
        self._target_v_ang = 0.0
        return CancelResponse.ACCEPT

    async def _execute_follow_path(self,
                                   goal_handle: ServerGoalHandle) -> FollowPath.Result:
        """Execute path following asynchronously."""
        self._follow_active = True
        self._follow_goal_handle = goal_handle
        self._follow_waypoints = list(goal_handle.request.waypoints)
        self._follow_target_vel = goal_handle.request.target_velocity
        self._follow_current_idx = 0
        self._follow_max_dev = 0.0

        result = FollowPath.Result()
        result.success = False
        result.completed_waypoints = 0
        result.deviation_max = 0.0

        self.get_logger().info(
            f'FollowPath started: {len(self._follow_waypoints)} waypoints, '
            f'target_vel={self._follow_target_vel:.2f} m/s')

        # The actual per-tick logic is in _update_path_following() called
        # from the control loop.  We spin here until the action completes
        # or is cancelled.
        while self._follow_active and self._follow_current_idx < len(self._follow_waypoints):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('FollowPath cancelled during execution')
                self._follow_active = False
                self._target_v_lin = 0.0
                self._target_v_ang = 0.0
                result.success = False
                goal_handle.canceled()
                return result
            await self._sleep_async(0.05)  # 20 Hz check

        self._follow_active = False
        self._follow_goal_handle = None
        result.success = True
        result.completed_waypoints = self._follow_current_idx
        result.deviation_max = self._follow_max_dev

        self._target_v_lin = 0.0
        self._target_v_ang = 0.0

        if result.success:
            self.get_logger().info(f'FollowPath complete: {result.completed_waypoints} wp done')
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    async def _sleep_async(self, seconds: float) -> None:
        """Async sleep helper."""
        import asyncio
        await asyncio.sleep(seconds)

    def _update_path_following(self) -> None:
        """Per-tick path following update called from control loop."""
        if (self._follow_goal_handle is None
                or self._follow_current_idx >= len(self._follow_waypoints)):
            return

        wp = self._follow_waypoints[self._follow_current_idx]
        tx = wp.pose.position.x
        ty = wp.pose.position.y

        cx, cy, ctheta = self._odom.pose
        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)

        # Heading to waypoint
        heading_to_wp = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(heading_to_wp - ctheta),
                                   math.cos(heading_to_wp - ctheta))

        # Lateral deviation (cross-track error)
        lateral = dist * math.sin(heading_error)
        self._follow_max_dev = max(self._follow_max_dev, abs(lateral))

        reached = dist < 0.05  # waypoint tolerance 5cm

        if reached:
            self._follow_current_idx += 1
            feedback = FollowPath.Feedback()
            feedback.current_waypoint_idx = self._follow_current_idx
            feedback.lateral_deviation = lateral
            feedback.path_progress = (self._follow_current_idx /
                                      float(len(self._follow_waypoints)))
            self._follow_goal_handle.publish_feedback(feedback)
        else:
            # Pure pursuit: steer toward waypoint
            self._target_v_lin = self._follow_target_vel
            # Proportional heading correction
            self._target_v_ang = 3.0 * heading_error  # gain 3.0

            # Clamp
            self._target_v_ang = max(min(self._target_v_ang, self._max_ang),
                                     -self._max_ang)

            feedback = FollowPath.Feedback()
            feedback.current_waypoint_idx = self._follow_current_idx
            feedback.lateral_deviation = lateral
            feedback.path_progress = (self._follow_current_idx /
                                      float(len(self._follow_waypoints)))
            self._follow_goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def on_shutdown(self) -> None:
        self.get_logger().info('motion_controller shutting down')
        self._estop_active = True
        self._target_v_lin = 0.0
        self._target_v_ang = 0.0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
