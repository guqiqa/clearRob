#!/usr/bin/env python3
"""
cleaning_robot_common/kinematics.py — Differential-drive math (pure functions).

No ROS dependencies. All functions are stateless — callers manage state.
"""

import math
from geometry_msgs.msg import Quaternion

from .config import (
    MOTOR_POLE_PAIRS, MOTOR_GEAR_RATIO,
    WHEEL_RADIUS_M, TRACK_WIDTH_M,
)


def diff_decompose(v_linear: float, w_angular: float) -> tuple:
    """cmd_vel (v, ω) → left/right wheel linear velocities (m/s).

    Returns (v_left, v_right).
    """
    half = TRACK_WIDTH_M / 2.0
    v_left  = v_linear - w_angular * half
    v_right = v_linear + w_angular * half
    return v_left, v_right


def erpm_to_ms(erpm: int) -> float:
    """Electrical RPM → wheel linear velocity (m/s)."""
    rpm_motor = erpm / MOTOR_POLE_PAIRS         # electrical → motor shaft rpm
    rpm_wheel = rpm_motor / MOTOR_GEAR_RATIO    # → wheel rpm (5.2:1 reduction)
    rad_s = rpm_wheel * 2.0 * math.pi / 60.0   # → rad/s
    return rad_s * WHEEL_RADIUS_M               # → m/s


def ms_to_current(target_vel: float, max_vel: float = 0.5,
                  max_current: int = 500, deadzone: int = 10) -> int:
    """Target wheel velocity (m/s) → CAN current value (int16, 10mA units).

    Linear scaling with dead zone. Returns 0 within dead zone.
    """
    ratio = target_vel / max(max_vel, 0.01)
    current = int(ratio * max_current)
    return 0 if abs(current) < deadzone else current


def compute_odom_velocity(motor_speeds: dict) -> tuple:
    """4-wheel motor speeds (m/s) → (v_linear, w_angular) for odometry.

    motor_speeds: {can_id: speed_m_s} for CAN IDs 1-4.
    """
    from .config import CAN_ID_LEFT_FRONT, CAN_ID_LEFT_REAR
    from .config import CAN_ID_RIGHT_FRONT, CAN_ID_RIGHT_REAR

    v_left  = (motor_speeds.get(CAN_ID_LEFT_FRONT, 0.0) +
               motor_speeds.get(CAN_ID_LEFT_REAR, 0.0)) / 2.0
    v_right = (motor_speeds.get(CAN_ID_RIGHT_FRONT, 0.0) +
               motor_speeds.get(CAN_ID_RIGHT_REAR, 0.0)) / 2.0

    v = (v_right + v_left) / 2.0
    w = (v_right - v_left) / TRACK_WIDTH_M
    return v, w


def integrate_odom(x: float, y: float, theta: float,
                   v: float, w: float, dt: float) -> tuple:
    """Euler integration of odometry. Returns (new_x, new_y, new_theta)."""
    new_theta = theta + w * dt
    dx = v * math.cos(theta) * dt
    dy = v * math.sin(theta) * dt
    return x + dx, y + dy, new_theta


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw angle (rad) → geometry_msgs/Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
