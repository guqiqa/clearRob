#!/usr/bin/env python3
"""
master_bridge/bridge_node.py — Master state machine + remote→chassis bridge.

V1.2: Extended from STANDBY↔MANUAL to the full multi-mode state machine
(ported from horizon1 master_controller/master_node.py):

  STANDBY / MANUAL / CRUISING / CLEANING / RETURNING / OBSTACLE_PAUSE

Control sources:
  - Remote (C7mini) /joy → /chassis/intent (primary, gated by SWA)
  - UnifiedCommand (control/cmd/unified) → AUTO_SWEEP / ROAD_SWEEP / STOP / MANUAL
  - Semantic alert (fusion/semantic_alert) → PEDESTRIAN_AHEAD → OBSTACLE_PAUSE
  - Planner trigger (control/trigger_planning) → "GO" / "GO_ROAD" → path_planner

Intent JSON:  {"throttle": -1..1, "steering": -1..1, "gear": "MID",
               "mower": 0|1, "estop": 0|1}
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from cleaning_robot_common.config import (
    TOPIC_CMD_VEL, TOPIC_JOY, TOPIC_MASTER_STATE, TOPIC_CHASSIS_INTENT,
    TOPIC_UNIFIED_CMD, TOPIC_SEMANTIC_ALERT, TOPIC_TRIGGER_PLANNING,
    MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY,
    ESTOP_DEBOUNCE_MS,
)

STATE_STANDBY  = "STANDBY"
STATE_MANUAL   = "MANUAL"
STATE_CRUISING = "CRUISING"
STATE_CLEANING = "CLEANING"
STATE_RETURNING = "RETURNING"
STATE_OBSTACLE_PAUSE = "OBSTACLE_PAUSE"   # pause on semantic alert


class MasterBridgeNode(Node):
    """Full-mode master state machine: remote + autonomous control."""

    def __init__(self) -> None:
        super().__init__("master_bridge")

        self.declare_parameter("max_linear_velocity", MAX_LINEAR_VELOCITY)
        self.declare_parameter("max_angular_velocity", MAX_ANGULAR_VELOCITY)
        self.declare_parameter("axis_steering", 0)    # Joy.axes[0] = steering (CH1)
        self.declare_parameter("axis_throttle", 1)    # Joy.axes[1] = throttle (CH3)
        self.declare_parameter("btn_start_stop", 0)  # joy.buttons[0] = SWA (CH5)
        self.declare_parameter("btn_estop", 2)       # joy.buttons[2] = SWB (CH6)
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
        self._resume_state = None      # resume target after OBSTACLE_PAUSE
        self._last_estop_time = 0.0
        self._estop_active = False
        self._swa_btn = 0          # debounced SWA state
        self._swa_last = 0
        self._swa_changed = 0.0
        self._start_time = time.time()

        # ROS interfaces
        qos_cmd = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)
        self._pub_cmd    = self.create_publisher(Twist, TOPIC_CMD_VEL, qos_cmd)
        self._pub_intent = self.create_publisher(String, TOPIC_CHASSIS_INTENT, 10)
        self._pub_state  = self.create_publisher(String, TOPIC_MASTER_STATE, 10)
        self._sub_joy    = self.create_subscription(Joy, TOPIC_JOY, self._cb_joy, 10)
        self._sub_cmd    = self.create_subscription(String, TOPIC_UNIFIED_CMD, self._cb_unified, 10)
        self._sub_alert  = self.create_subscription(String, TOPIC_SEMANTIC_ALERT, self._cb_alert, 10)
        self._pub_trigger = self.create_publisher(String, TOPIC_TRIGGER_PLANNING, 10)
        self._hb_timer   = self.create_timer(1.0, self._publish_heartbeat)

        self.get_logger().info(f"master_bridge started — state: {self._state}")

    # ---- UnifiedCommand (AUTO_SWEEP / ROAD_SWEEP / STOP / MANUAL) ----

    def _cb_unified(self, msg: String) -> None:
        cmd = msg.data
        self.get_logger().info(f"收到最高指令: {cmd}")
        if cmd == "AUTO_SWEEP":
            self._state = STATE_CRUISING
            self._publish_trigger("GO")
        elif cmd == "ROAD_SWEEP":
            self._state = STATE_CRUISING
            self._publish_trigger("GO_ROAD")
        elif cmd == "STOP":
            self._state = STATE_STANDBY
            self._resume_state = None
            self._stop()
        elif cmd == "MANUAL":
            self._state = STATE_MANUAL
        self._pub_state.publish(String(data=self._state))

    # ---- Semantic alert (fusion_engine) ----

    def _cb_alert(self, msg: String) -> None:
        if msg.data == "PEDESTRIAN_AHEAD":
            if self._state == STATE_CRUISING:
                self.get_logger().warn("🚨 行人！暂停自动巡航，原地等待...")
                self._resume_state = self._state
                self._state = STATE_OBSTACLE_PAUSE
                self._stop()
        elif msg.data == "CLEAR":
            if self._state == STATE_OBSTACLE_PAUSE:
                self.get_logger().info("✅ 行人离开，恢复自动巡航")
                self._state = self._resume_state or STATE_CRUISING
                self._resume_state = None
                # re-trigger planner to continue
                self._publish_trigger("GO")
        self._pub_state.publish(String(data=self._state))

    # ---- Joy → state machine + intent + cmd_vel ----

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
        if not btn_estop and self._estop_active:
            self._estop_active = False
            self.get_logger().info("ESTOP released")

        # SWA debounce: SWA ON → MANUAL; SWA OFF → back to STANDBY
        if btn_start != self._swa_last:
            self._swa_last = btn_start
            self._swa_changed = now
        elif now - self._swa_changed > 0.5 and btn_start != self._swa_btn:
            self._swa_btn = btn_start
            if btn_start:
                self._state = STATE_MANUAL
            else:
                self._state = STATE_STANDBY
                self._resume_state = None
            self.get_logger().info(f"→ {self._state}")

        # Intent: estop overrides everything; MANUAL gates motion; others stop.
        estop = 1 if self._estop_active else 0
        if self._estop_active or self._state != STATE_MANUAL:
            t, s = 0.0, 0.0
        else:
            t, s = throttle, steering
        intent = {"throttle": round(t, 3), "steering": round(s, 3),
                  "gear": "MID", "mower": 0, "estop": estop}
        self._pub_intent.publish(String(data=json.dumps(intent)))

        # Backward-compat /cmd_vel (other consumers; chassis prefers intent)
        twist = Twist()
        twist.linear.x = t * self._max_v
        twist.angular.z = s * self._max_w
        self._pub_cmd.publish(twist)
        self._pub_state.publish(String(data=self._state))

    # ---- helpers ----

    def _publish_trigger(self, trigger: str) -> None:
        self._pub_trigger.publish(String(data=trigger))

    def _stop(self) -> None:
        self._pub_cmd.publish(Twist())
        self._pub_intent.publish(String(data=json.dumps(
            {"throttle": 0.0, "steering": 0.0, "gear": "MID", "mower": 0, "estop": 1})))

    def _publish_zero(self) -> None:
        self._stop()

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

