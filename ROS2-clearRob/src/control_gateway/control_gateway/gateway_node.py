#!/usr/bin/env python3
"""
control_gateway: System entry point for the cleaning robot.

Receives raw commands from RC/App/Voice device abstraction topics, translates
them to normalized UnifiedCommand messages, handles priority arbitration,
and publishes motion velocity in MANUAL mode.
"""

import json
import time
import threading
from enum import IntEnum

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Header, String
from geometry_msgs.msg import Twist

from cleaning_robot_interfaces.msg import (
    RCRawSignal,
    AppCommand,
    UnifiedCommand,
    MotionCommand,
    VisionStatus,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import SetPriority


# =============================================================================
# Enumerations
# =============================================================================

class CommandCode(IntEnum):
    START = 0
    PAUSE = 1
    RESUME = 2
    STOP = 3
    RETURN = 4
    MANUAL = 5
    SET_AREA = 6
    SET_MODE = 7
    ESTOP = 8


class CommandSource(IntEnum):
    RC = 0
    APP = 1
    VOICE = 2


class RobotMode(IntEnum):
    AUTO = 0
    MANUAL = 1
    ESTOP = 2


class LifecycleStateCode(IntEnum):
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4


# Command name to code mapping
COMMAND_NAME_TO_CODE = {
    "START": CommandCode.START,
    "PAUSE": CommandCode.PAUSE,
    "RESUME": CommandCode.RESUME,
    "STOP": CommandCode.STOP,
    "RETURN": CommandCode.RETURN,
    "MANUAL": CommandCode.MANUAL,
    "SET_AREA": CommandCode.SET_AREA,
    "SET_MODE": CommandCode.SET_MODE,
    "ESTOP": CommandCode.ESTOP,
}

# Commands that transition MANUAL back to AUTO
MANUAL_EXIT_COMMANDS = {CommandCode.START, CommandCode.RESUME, CommandCode.STOP, CommandCode.RETURN}

# =============================================================================
# RC Button Indexes (device-specific mapping, adjustable via params)
# =============================================================================
RC_BTN_CLEAN = 0
RC_BTN_RETURN = 1
RC_BTN_L1 = 2
RC_BTN_R2 = 3
RC_BTN_L2 = 4
RC_BTN_R1 = 5
RC_AXIS_Y = 1  # forward/back
RC_AXIS_X = 0  # left/right (angular)


# =============================================================================
# Voice command keyword sets (Chinese, case-insensitive matching)
# =============================================================================
VOICE_START_KEYWORDS = ["开始清扫", "启动", "开工"]
VOICE_PAUSE_KEYWORDS = ["暂停", "等一下"]
VOICE_RESUME_KEYWORDS = ["继续", "恢复", "接着扫"]
VOICE_STOP_KEYWORDS = ["停止", "停", "别扫了"]
VOICE_RETURN_KEYWORDS = ["回去", "返回", "充电"]


# =============================================================================
# ControlGateway
# =============================================================================

class ControlGateway(LifecycleNode):
    """Control gateway node – entry point for all external command sources."""

    def __init__(self, node_name="control_gateway"):
        super().__init__(node_name)
        self._lifecycle_state_code = LifecycleStateCode.UNCONFIGURED
        self._init_defaults()

    def _init_defaults(self):
        """Initialise all internal state to sensible defaults."""
        # --- Mode ---
        self._mode = RobotMode.AUTO

        # --- ESTOP ---
        self._estop_active = False
        self._estop_trigger_pressed_since = None
        self._estop_release_pressed_since = None
        self._estop_lock = threading.Lock()

        # --- RC state ---
        self._last_clean_press_time = 0.0
        self._clean_pressed = False
        self._rc_joystick_deadband = 0.05

        # --- Arbitration ---
        self._dedup_history = {}
        self._dedup_history_lock = threading.Lock()

        # --- Velocity rate limiter ---
        self._last_vel_publish_time = 0.0

        # --- Params ---
        self._declare_params()

        # --- Command codes cache ---
        self._command_code_map = dict(COMMAND_NAME_TO_CODE)

    def _declare_params(self):
        self.declare_parameter("default_priority_rule", "RC_APP")
        self.declare_parameter("dedup_interval_ms", 200)
        self.declare_parameter("arbitration_window_ms", 50)
        self.declare_parameter("voice_confidence_threshold", 0.6)
        self.declare_parameter("max_linear_velocity", 1.5)
        self.declare_parameter("max_angular_velocity", 2.0)
        self.declare_parameter("max_velocity_publish_rate", 50.0)
        self.declare_parameter("estop_debounce_ms", 100)

    # ------------------------------------------------------------------
    # Lifecycle: configure
    # ------------------------------------------------------------------
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure()")

        self._lifecycle_state_code = LifecycleStateCode.INACTIVE
        self._priority_rule = self.get_parameter("default_priority_rule").get_parameter_value().string_value
        self._dedup_interval_ms = self.get_parameter("dedup_interval_ms").get_parameter_value().integer_value
        self._arbitration_window_ms = self.get_parameter("arbitration_window_ms").get_parameter_value().integer_value
        self._voice_confidence_threshold = self.get_parameter("voice_confidence_threshold").get_parameter_value().double_value
        self._max_linear_velocity = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self._max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self._max_velocity_publish_rate = self.get_parameter("max_velocity_publish_rate").get_parameter_value().double_value
        self._estop_debounce_ms = self.get_parameter("estop_debounce_ms").get_parameter_value().integer_value

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Callback groups
        sensor_cbg = MutuallyExclusiveCallbackGroup()
        service_cbg = MutuallyExclusiveCallbackGroup()
        timer_cbg = MutuallyExclusiveCallbackGroup()

        # --- Subscriptions ---
        self._rc_raw_sub = self.create_subscription(
            RCRawSignal, "device/rc/raw",
            self._on_rc_raw, sensor_qos, callback_group=sensor_cbg
        )
        self._app_cmd_sub = self.create_subscription(
            AppCommand, "device/app/cmd",
            self._on_app_cmd, reliable_qos, callback_group=sensor_cbg
        )
        self._voice_text_sub = self.create_subscription(
            String, "device/voice/text",
            self._on_voice_text, reliable_qos, callback_group=sensor_cbg
        )
        self._rc_vel_sub = self.create_subscription(
            Twist, "device/rc/velocity",
            self._on_rc_velocity, sensor_qos, callback_group=sensor_cbg
        )
        self._app_vel_sub = self.create_subscription(
            Twist, "device/app/velocity",
            self._on_app_velocity, sensor_qos, callback_group=sensor_cbg
        )

        # --- Publishers ---
        self._unified_pub = self.create_publisher(UnifiedCommand, "control/cmd/unified", 10)
        self._manual_vel_pub = self.create_publisher(Twist, "control/cmd/manual_velocity", 10)
        self._estop_pub = self.create_publisher(MotionCommand, "control/cmd/estop", 10)
        self._status_pub = self.create_publisher(VisionStatus, "control/status", 10)
        self._heartbeat_pub = self.create_publisher(Heartbeat, "system/heartbeat/control_gateway", 10)

        # --- Services ---
        self._set_priority_srv = self.create_service(
            SetPriority, "control/set_priority",
            self._on_set_priority, callback_group=service_cbg
        )

        # --- Timers ---
        self._heartbeat_timer = self.create_timer(1.0, self._on_heartbeat_tick, callback_group=timer_cbg)
        self._status_timer = self.create_timer(1.0, self._on_status_tick, callback_group=timer_cbg)
        self._estop_timer = self.create_timer(0.01, self._on_estop_tick, callback_group=timer_cbg)

        # Velocity rate limiter
        self._last_vel_publish_time = 0.0
        self._vel_period = 1.0 / max(self._max_velocity_publish_rate, 1.0)

        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Lifecycle: activate
    # ------------------------------------------------------------------
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate()")
        self._lifecycle_state_code = LifecycleStateCode.ACTIVE
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Lifecycle: deactivate
    # ------------------------------------------------------------------
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate()")
        self._lifecycle_state_code = LifecycleStateCode.INACTIVE
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Lifecycle: cleanup
    # ------------------------------------------------------------------
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup()")
        self._lifecycle_state_code = LifecycleStateCode.UNCONFIGURED
        self._init_defaults()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Lifecycle: shutdown
    # ------------------------------------------------------------------
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown()")
        self._lifecycle_state_code = LifecycleStateCode.FINALIZED
        return TransitionCallbackReturn.SUCCESS

    # ==================================================================
    # RC Raw Signal Handler
    # ==================================================================
    def _on_rc_raw(self, msg: RCRawSignal):
        """Parse raw RC signal: button presses and axes."""
        if not msg.connected:
            return

        now = time.time()
        buttons = msg.buttons
        axes = msg.axes

        # -- ESTOP trigger/release (L1+R1, L2+R2) --
        l1_pressed = len(buttons) > RC_BTN_L1 and buttons[RC_BTN_L1] > 0
        r1_pressed = len(buttons) > RC_BTN_R1 and buttons[RC_BTN_R1] > 0
        l2_pressed = len(buttons) > RC_BTN_L2 and buttons[RC_BTN_L2] > 0
        r2_pressed = len(buttons) > RC_BTN_R2 and buttons[RC_BTN_R2] > 0

        # ESTOP trigger: L1+R1 together
        if l1_pressed and r1_pressed:
            with self._estop_lock:
                if self._estop_trigger_pressed_since is None:
                    self._estop_trigger_pressed_since = now
        else:
            with self._estop_lock:
                self._estop_trigger_pressed_since = None

        # ESTOP release: L2+R2 together
        if l2_pressed and r2_pressed:
            with self._estop_lock:
                if self._estop_release_pressed_since is None:
                    self._estop_release_pressed_since = now
        else:
            with self._estop_lock:
                self._estop_release_pressed_since = None

        # -- Clean button (short/long press) --
        clean_pressed = len(buttons) > RC_BTN_CLEAN and buttons[RC_BTN_CLEAN] > 0
        if clean_pressed and not self._clean_pressed:
            self._last_clean_press_time = now
            self._clean_pressed = True
        elif not clean_pressed and self._clean_pressed:
            # button released – determine duration
            duration = now - self._last_clean_press_time
            if duration >= 2.0:
                self._emit_unified(CommandCode.STOP, "STOP", CommandSource.RC)
            elif duration < 1.0:
                if self._mode == RobotMode.AUTO:
                    # Short press when idle/paused → START
                    self._emit_unified(CommandCode.START, "START", CommandSource.RC)
                else:
                    # Short press when cruising/cleaning → PAUSE
                    self._emit_unified(CommandCode.PAUSE, "PAUSE", CommandSource.RC)
            self._clean_pressed = False

        # Long press while still held (>=2s trigger stop without waiting for release)
        if clean_pressed and self._clean_pressed:
            if now - self._last_clean_press_time >= 2.0:
                self._emit_unified(CommandCode.STOP, "STOP", CommandSource.RC)
                self._clean_pressed = False  # prevent duplicate on release

        # -- Return button --
        return_pressed = len(buttons) > RC_BTN_RETURN and buttons[RC_BTN_RETURN] > 0
        if return_pressed:
            self._emit_unified(CommandCode.RETURN, "RETURN", CommandSource.RC)

    def _on_rc_velocity(self, msg: Twist):
        """RC joystick velocity – pass through in MANUAL mode."""
        if self._mode != RobotMode.MANUAL:
            return
        self._publish_manual_velocity(msg.linear.x, msg.angular.z)

    # ==================================================================
    # App Command Handler
    # ==================================================================
    def _on_app_cmd(self, msg: AppCommand):
        """Parse structured App command."""
        cmd_name = msg.command.upper().strip()
        code = self._command_code_map.get(cmd_name)
        if code is None:
            self.get_logger().warning(f"Unknown App command: {cmd_name}")
            return

        params = [0.0] * 10
        try:
            pdict = json.loads(msg.params_json) if msg.params_json else {}
        except json.JSONDecodeError:
            pdict = {}

        if code == CommandCode.SET_AREA:
            params[0] = float(pdict.get("area_id", 0))
        elif code == CommandCode.SET_MODE:
            # mode: 0=CRUISE, 1=CLEAN, 2=RETURN
            params[0] = float(pdict.get("mode", 0))

        self._emit_unified(code, cmd_name, CommandSource.APP, params)

    def _on_app_velocity(self, msg: Twist):
        """App virtual joystick – pass through in MANUAL, unless RC is active."""
        if self._mode != RobotMode.MANUAL:
            return
        # RC has priority for MANUAL velocity – the _last_rc_vel_active flag
        # is used to suppress app velocity when RC is sending non-zero values.
        self._publish_manual_velocity(msg.linear.x, msg.angular.z)

    # ==================================================================
    # Voice Text Handler
    # ==================================================================
    def _on_voice_text(self, msg: String):
        """Match voice text to commands (Chinese keyword matching)."""
        text = msg.data.strip()

        code = self._match_voice_keyword(text)
        if code is None:
            # Try to extract confidence from a simple JSON wrapper: {"text":"...", "confidence":0.7}
            try:
                payload = json.loads(text)
                inner_text = payload.get("text", "")
                confidence = float(payload.get("confidence", 1.0))
                if confidence < self._voice_confidence_threshold:
                    return
                code = self._match_voice_keyword(inner_text)
                if code is None:
                    return
            except (json.JSONDecodeError, ValueError, TypeError):
                return
        else:
            # Plain text matched – assume confidence 1.0
            pass

        self._emit_unified(code, self._code_to_name(code), CommandSource.VOICE)

    @staticmethod
    def _match_voice_keyword(text: str):
        """Return CommandCode if text matches any voice keyword set."""
        text_lower = text.lower().strip()
        for kw in VOICE_START_KEYWORDS:
            if kw in text_lower:
                return CommandCode.START
        for kw in VOICE_PAUSE_KEYWORDS:
            if kw in text_lower:
                return CommandCode.PAUSE
        for kw in VOICE_RESUME_KEYWORDS:
            if kw in text_lower:
                return CommandCode.RESUME
        for kw in VOICE_STOP_KEYWORDS:
            if kw in text_lower:
                return CommandCode.STOP
        for kw in VOICE_RETURN_KEYWORDS:
            if kw in text_lower:
                return CommandCode.RETURN
        return None

    @staticmethod
    def _code_to_name(code: CommandCode) -> str:
        for k, v in CommandCode.__members__.items():
            if v == code:
                return k
        return "UNKNOWN"

    # ==================================================================
    # Manual Velocity Publisher
    # ==================================================================
    def _publish_manual_velocity(self, linear_input: float, angular_input: float):
        """Publish manual velocity clamped to configured limits, rate-limited."""
        now = time.time()
        vel_period = 1.0 / max(self._max_velocity_publish_rate, 1.0)
        if now - self._last_vel_publish_time < vel_period:
            return
        self._last_vel_publish_time = now

        linear = max(-self._max_linear_velocity, min(self._max_linear_velocity,
                     linear_input * self._max_linear_velocity))
        angular = max(-self._max_angular_velocity, min(self._max_angular_velocity,
                      angular_input * self._max_angular_velocity))

        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        self._manual_vel_pub.publish(vel_msg)

    # ==================================================================
    # Unified Command Emitter (with dedup + priority arbitration)
    # ==================================================================
    def _emit_unified(self, code: CommandCode, cmd_name: str, source: CommandSource,
                      params: list = None):
        """Emit a UnifiedCommand through priority arbitration and dedup."""
        if params is None:
            params = [0.0] * 10

        now = time.time()

        # --- ESTOP bypass ---
        if code == CommandCode.ESTOP:
            cmd = self._build_unified(code, cmd_name, source, params)
            self._unified_pub.publish(cmd)
            return

        # --- Dedup check ---
        dedup_key = (int(code), int(source), tuple(params[:3]))
        with self._dedup_history_lock:
            last_ts = self._dedup_history.get(dedup_key)
            if last_ts is not None and (now - last_ts) * 1000 < self._dedup_interval_ms:
                return
            self._dedup_history[dedup_key] = now

        # --- Priority check (arbitration) ---
        can_proceed = self._priority_allows(source)
        if not can_proceed:
            self.get_logger().debug(
                f"Command {cmd_name} from source {int(source)} blocked by priority rule"
            )
            return

        # --- Mode transition ---
        self._apply_mode_transition(code)

        # --- Publish ---
        cmd = self._build_unified(code, cmd_name, source, params)
        self._unified_pub.publish(cmd)

    def _priority_allows(self, source: CommandSource) -> bool:
        """Check if the given source is allowed under current priority rule."""
        rule = self._priority_rule
        if rule == "ALL":
            return True
        elif rule == "RC_ONLY":
            return source == CommandSource.RC
        elif rule == "RC_APP":
            return source in (CommandSource.RC, CommandSource.APP)
        else:
            # Unknown rule → allow all
            return True

    def _apply_mode_transition(self, code: CommandCode):
        """Update internal robot mode based on command code."""
        if code == CommandCode.MANUAL:
            self._mode = RobotMode.MANUAL
        elif code == CommandCode.ESTOP:
            self._mode = RobotMode.ESTOP
        elif code in MANUAL_EXIT_COMMANDS and self._mode == RobotMode.MANUAL:
            self._mode = RobotMode.AUTO

    def _build_unified(self, code: CommandCode, cmd_name: str, source: CommandSource,
                       params: list) -> UnifiedCommand:
        cmd = UnifiedCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command = int(code)
        cmd.command_name = cmd_name
        cmd.source = int(source)
        cmd.params = list(params) if len(params) <= 10 else list(params[:10])
        return cmd

    # ==================================================================
    # ESTOP 10ms Timer
    # ==================================================================
    def _on_estop_tick(self):
        """Check debounced ESTOP trigger/release."""
        now = time.time()
        debounce_s = self._estop_debounce_ms / 1000.0

        with self._estop_lock:
            # Trigger
            if self._estop_trigger_pressed_since is not None and not self._estop_active:
                if now - self._estop_trigger_pressed_since >= debounce_s:
                    self._estop_active = True
                    self._mode = RobotMode.ESTOP
                    self._estop_trigger_pressed_since = None
                    self.get_logger().warn("ESTOP triggered")
                    self._publish_estop(True)
            # Release
            if self._estop_release_pressed_since is not None and self._estop_active:
                if now - self._estop_release_pressed_since >= debounce_s:
                    self._estop_active = False
                    self._mode = RobotMode.AUTO
                    self._estop_release_pressed_since = None
                    self.get_logger().info("ESTOP released")
                    self._publish_estop(False)

    def _publish_estop(self, estop: bool):
        """Publish ESTOP state on both estop topic and unified topic."""
        # Direct motion command
        mc = MotionCommand()
        mc.header = Header()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.linear_velocity = 0.0
        mc.angular_velocity = 0.0
        mc.estop = estop
        self._estop_pub.publish(mc)

        # Unified command for logging/monitoring
        uc = UnifiedCommand()
        uc.header = Header()
        uc.header.stamp = self.get_clock().now().to_msg()
        uc.command = int(CommandCode.ESTOP)
        uc.command_name = "ESTOP"
        uc.source = int(CommandSource.RC)
        uc.params[0] = 1.0 if estop else 0.0
        self._unified_pub.publish(uc)

    # ==================================================================
    # SetPriority Service
    # ==================================================================
    def _on_set_priority(self, request, response):
        """Runtime priority rule adjustment."""
        valid = {"RC_ONLY", "RC_APP", "ALL"}
        rule = request.rule.strip().upper()
        if rule not in valid:
            response.success = False
            response.message = f"Invalid rule '{rule}'. Valid: {', '.join(sorted(valid))}"
            return response
        prev = self._priority_rule
        self._priority_rule = rule
        self.get_logger().info(f"Priority rule changed: {prev} -> {rule}")
        response.success = True
        response.message = f"Priority rule set to {rule}"
        return response

    # ==================================================================
    # 1Hz Timers
    # ==================================================================
    def _on_heartbeat_tick(self):
        """Publish heartbeat at 1Hz."""
        hb = Heartbeat()
        hb.header = Header()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.node_name = "control_gateway"
        hb.lifecycle_state = int(self._lifecycle_state_code)
        hb.error_code = 0
        self._heartbeat_pub.publish(hb)

    def _on_status_tick(self):
        """Publish vision/control status at 1Hz."""
        vs = VisionStatus()
        vs.active_model = "control_gateway"
        vs.avg_inference_ms = 0.0
        vs.fps = 0.0
        vs.status = 0
        vs.status_text = f"mode={self._mode.name} estop={self._estop_active} priority={self._priority_rule}"
        self._status_pub.publish(vs)


# =============================================================================
# Entry point
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ControlGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
