#!/usr/bin/env python3
"""
master_bridge/bridge_node.py — Phase 1 state machine + remote→chassis bridge.

Uses cleaning_robot_common for all constants.
State machine: STANDBY ↔ MANUAL (RC CH5 toggle).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from cleaning_robot_common.config import (
    TOPIC_CMD_VEL, TOPIC_JOY, TOPIC_MASTER_STATE,
    MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY,
    RC_CH_STEERING, RC_CH_THROTTLE,
    RC_CH_SWA, RC_CH_SWB,
    ESTOP_DEBOUNCE_MS,
)

STATE_STANDBY = "STANDBY"
STATE_MANUAL  = "MANUAL"


class MasterBridgeNode(Node):
    """Phase 1 master bridge: remote control → chassis control."""

    def __init__(self) -> None:
        super().__init__("master_bridge")

        self.declare_parameter("max_linear_velocity", MAX_LINEAR_VELOCITY)
        self.declare_parameter("max_angular_velocity", MAX_ANGULAR_VELOCITY)
        self.declare_parameter("axis_steering", 0)    # Joy.axes[0] = steering
        self.declare_parameter("axis_throttle", 1)    # Joy.axes[1] = throttle (NOT SBUS ch)
        self.declare_parameter("btn_start_stop", 0)  # joy.buttons[0] = SWA
        self.declare_parameter("btn_estop", 2)       # joy.buttons[2] = SWB
        self.declare_parameter("estop_debounce_ms", ESTOP_DEBOUNCE_MS)

        g = lambda n: self.get_parameter(n).get_parameter_value()
        self._max_v       = g("max_linear_velocity").double_value
        self._max_w       = g("max_angular_velocity").double_value
        self._ax_steer    = g("axis_steering").integer_value
        self._ax_throttle = g("axis_throttle").integer_value
        self._btn_start   = g("btn_start_stop").integer_value
        self._btn_estop   = g("btn_estop").integer_value
        self._estop_db    = g("estop_debounce_ms").integer_value / 1000.0

        # State
        self._state = STATE_STANDBY
        self._last_estop_time = 0.0
        self._estop_active = False
        self._swa_btn = 0          # debounced SWA state
        self._swa_last = 0
        self._swa_changed = 0.0    # timestamp when SWA raw value changed
        self._start_time = time.time()

        # ROS interfaces
        qos_cmd = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)
        self._pub_cmd   = self.create_publisher(Twist, TOPIC_CMD_VEL, qos_cmd)
        self._pub_state = self.create_publisher(String, TOPIC_MASTER_STATE, 10)
        self._sub_joy   = self.create_subscription(Joy, TOPIC_JOY, self._cb_joy, 10)
        self._hb_timer  = self.create_timer(1.0, self._publish_heartbeat)

        self.get_logger().info(f"master_bridge started — state: {self._state}")

    # ---- Joy → state machine + cmd_vel ----

    def _cb_joy(self, msg: Joy) -> None:
        now = time.time()
        steering = msg.axes[self._ax_steer] if self._ax_steer < len(msg.axes) else 0.0
        throttle = msg.axes[self._ax_throttle] if self._ax_throttle < len(msg.axes) else 0.0
        btn_start = msg.buttons[self._btn_start] if self._btn_start < len(msg.buttons) else 0
        btn_estop = msg.buttons[self._btn_estop] if self._btn_estop < len(msg.buttons) else 0

        # ESTOP (debounced)
        if btn_estop and not self._estop_active and (now - self._last_estop_time > self._estop_db):
            self._estop_active = True
            self._last_estop_time = now
            self.get_logger().warn("ESTOP activated")
            self._publish_zero()
        if not btn_estop and self._estop_active:
            self._estop_active = False
            self.get_logger().info("ESTOP released")
        if self._estop_active:
            return

        # SWA debounce: raw button must be stable for 0.5s before state change
        if btn_start != self._swa_last:
            self._swa_last = btn_start
            self._swa_changed = now
        elif now - self._swa_changed > 0.5 and btn_start != self._swa_btn:
            self._swa_btn = btn_start
            if btn_start:
                self._state = STATE_MANUAL
            else:
                self._state = STATE_STANDBY
            self.get_logger().info(f"→ {self._state}" if btn_start else f"→ STANDBY")

        # Motion
        v = throttle * self._max_v if self._state == STATE_MANUAL else 0.0
        w = steering * self._max_w if self._state == STATE_MANUAL else 0.0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self._pub_cmd.publish(twist)
        self._pub_state.publish(String(data=self._state))

    def _publish_zero(self) -> None:
        self._pub_cmd.publish(Twist())

    def _publish_heartbeat(self) -> None:
        self.get_logger().debug(f"heartbeat: {self._state} "
                                f"uptime={time.time()-self._start_time:.0f}s")

    def destroy_node(self) -> None:
        self._publish_zero()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
