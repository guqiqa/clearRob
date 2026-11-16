#!/usr/bin/env python3
"""
cleaning_actuator ROS2 Node.

Controls physical cleaning mechanisms for the cleaning robot:
  - Main brush, side brush, suction fan, water spray
  - Dust bin monitoring with simulated fill accumulation
  - Brush wear tracking and stall detection
  - Garbage-class adaptive parameter adjustment
  - Action server for execute_cleanup
  - Heartbeat, status publishing, and statistics reporting
"""

import random
from enum import IntEnum

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import State as LifecycleStateMsg
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# std_msgs
from std_msgs.msg import Float32, Header

# cleaning_robot_interfaces
from cleaning_robot_interfaces.msg import (
    CleanStrategy,
    DetectionArray,
    CleaningProgress,
    DustFull,
    BrushStatus,
    Heartbeat,
    Alert,
)
from cleaning_robot_interfaces.srv import GetStatus
from cleaning_robot_interfaces.action import ExecuteCleanup


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class Strategy(IntEnum):
    COVER = 0
    SPOT = 1
    TRACK = 2
    IDLE = 3


# Lifecycle state codes for Heartbeat msg (uint8 field values)
class LifecycleStateCode(IntEnum):
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4


class ErrorCode(IntEnum):
    NORMAL = 0
    INIT_FAILED = 1
    RUNTIME_ERROR = 2


# Pre-defined garbage class IDs (matching vision_detector labels)
GARBAGE_RECYCLABLE = 0      # large bottles / boxes
GARBAGE_KITCHEN = 1         # food waste
GARBAGE_HAZARDOUS = 2       # batteries / medicine
GARBAGE_OTHER = 3           # cigarettes / paper
GARBAGE_GREEN = 4           # leaves
GARBAGE_STAIN = 5           # stains on floor

# Class name mapping for safety checks
GARBAGE_CLASS_NAMES = {
    "recyclable": GARBAGE_RECYCLABLE,
    "kitchen_waste": GARBAGE_KITCHEN,
    "hazardous": GARBAGE_HAZARDOUS,
    "other_waste": GARBAGE_OTHER,
    "green_waste": GARBAGE_GREEN,
    "stain": GARBAGE_STAIN,
}


# ---------------------------------------------------------------------------
# Default strategy parameters: {Strategy: (main_brush, side_brush, suction, water)}
# ---------------------------------------------------------------------------

STRATEGY_DEFAULTS = {
    Strategy.COVER: (0.7, 0.7, 0.6, 0.3),
    Strategy.SPOT:  (1.0, 1.0, 1.0, 0.8),
    Strategy.TRACK: (0.8, 0.5, 0.9, 0.5),
    Strategy.IDLE:  (0.0, 0.0, 0.0, 0.0),
}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CleaningActuatorNode(LifecycleNode):
    """Lifecycle-managed node that simulates the cleaning actuator subsystem."""

    # ---- ROS parameters (defaults) ----------------------------------------
    _defaults = {
        "cleaning_width": 0.8,
        "max_brush_life_hours": 500.0,
        "max_dust_capacity_l": 50.0,
        "dust_fill_rate_per_minute": 0.033,
        "coverage_efficiency": 0.85,
    }

    def __init__(self, node_name: str = "cleaning_actuator"):
        super().__init__(node_name)

        # -- declare ROS parameters
        for key, val in self._defaults.items():
            self.declare_parameter(key, val)

        # -- internal state --------------------------------------------------
        self._start_time = self.get_clock().now()

        # strategy state
        self._active_strategy: Strategy = Strategy.IDLE
        self._cmd_main_brush = 0.0
        self._cmd_side_brush = 0.0
        self._cmd_suction = 0.0
        self._cmd_water = 0.0

        # adaptive overrides (set per detection class)
        self._adaptive_overrides = {}

        # brush state
        self._main_brush_accum_hours = 0.0
        self._side_brush_accum_hours = 0.0
        self._main_brush_stall = False
        self._side_brush_stall = False

        # simulated RPM (smoothed)
        self._main_brush_rpm = 0.0
        self._side_brush_rpm = 0.0

        # dust
        self._dust_level = 0.0        # 0.0 -> 1.0
        self._filter_clog_ratio = 0.0

        # water
        self._total_water_used_l = 0.0

        # statistics
        self._cleaned_area_m2 = 0.0
        self._distance_traveled = 0.0
        self._waste_volume_l = 0.0

        # suction overheat tracking
        self._suction_high_start = None   # clock time when suction > 0.8 started
        self._suction_reduced = False

        # heartbeat sequence
        self._hb_seq = 0

        # action state (set during active cleanup)
        self._action_active = False
        self._action_last_cleanliness = 0.5

        # timers
        self._timer_hz1 = None       # 1 Hz  (heartbeat, dust, brush)
        self._timer_hz02 = None      # 0.2 Hz (area, waste stats)

        # subscriptions
        self._sub_strategy = None
        self._sub_detections = None
        self._sub_cleanliness = None

        # publishers
        self._pub_progress = None
        self._pub_dust = None
        self._pub_brush = None
        self._pub_area = None
        self._pub_waste = None
        self._pub_heartbeat = None
        self._pub_alert = None

        # service
        self._srv_get_status = None

        # action server
        self._action_srv = None

        self.get_logger().info("CleaningActuatorNode constructed (not yet configured)")

    # =======================================================================
    # Lifecycle callbacks
    # =======================================================================

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure()")

        # -- QoS profiles ---------------------------------------------------
        qos_default = QoSProfile(depth=10)
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -- Callback groups ------------------------------------------------
        cb_default = MutuallyExclusiveCallbackGroup()
        cb_action = ReentrantCallbackGroup()

        # -- Subscriptions --------------------------------------------------
        self._sub_strategy = self.create_subscription(
            CleanStrategy,
            "master/task/clean_strategy",
            self._on_strategy,
            qos_reliable,
            callback_group=cb_default,
        )

        self._sub_detections = self.create_subscription(
            DetectionArray,
            "vision/detect/list",
            self._on_detections,
            qos_sensor,
            callback_group=cb_default,
        )

        self._sub_cleanliness = self.create_subscription(
            Float32,
            "vision/detect/cleanliness",
            self._on_cleanliness,
            qos_sensor,
            callback_group=cb_default,
        )

        # -- Publishers -----------------------------------------------------
        self._pub_progress = self.create_publisher(
            CleaningProgress, "cleaning/status/progress", qos_default
        )
        self._pub_dust = self.create_publisher(
            DustFull, "cleaning/status/dust_full", qos_default
        )
        self._pub_brush = self.create_publisher(
            BrushStatus, "cleaning/status/brush", qos_default
        )
        self._pub_area = self.create_publisher(
            Float32, "cleaning/stats/area", qos_default
        )
        self._pub_waste = self.create_publisher(
            Float32, "cleaning/stats/waste_volume", qos_default
        )
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, "system/heartbeat/cleaning_actuator", qos_default
        )
        self._pub_alert = self.create_publisher(
            Alert, "cleaning/alert", qos_default
        )

        # -- Service --------------------------------------------------------
        self._srv_get_status = self.create_service(
            GetStatus, "cleaning/get_status", self._on_get_status,
            callback_group=cb_default,
        )

        # -- Action server --------------------------------------------------
        self._action_srv = ActionServer(
            self,
            ExecuteCleanup,
            "cleaning/execute_cleanup",
            execute_callback=self._execute_cleanup_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_action,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate()")

        self._start_time = self.get_clock().now()
        self._dust_level = 0.0
        self._filter_clog_ratio = 0.0
        self._total_water_used_l = 0.0
        self._cleaned_area_m2 = 0.0
        self._waste_volume_l = 0.0
        self._suction_high_start = None
        self._suction_reduced = False

        # Start periodic timers
        self._timer_hz1 = self.create_timer(1.0, self._tick_1hz)
        self._timer_hz02 = self.create_timer(5.0, self._tick_02hz)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate()")

        # Stop all brushes / suction / water
        self._cmd_main_brush = 0.0
        self._cmd_side_brush = 0.0
        self._cmd_suction = 0.0
        self._cmd_water = 0.0
        self._active_strategy = Strategy.IDLE
        self._adaptive_overrides.clear()

        self._destroy_timer(self._timer_hz1)
        self._destroy_timer(self._timer_hz02)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup()")
        self.destroy_node()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown()")
        return TransitionCallbackReturn.SUCCESS

    # =======================================================================
    # Subscription callbacks
    # =======================================================================

    def _on_strategy(self, msg: CleanStrategy):
        """Receive a cleaning strategy command from the master."""
        strategy = Strategy(msg.strategy) if msg.strategy < 4 else Strategy.IDLE
        self._active_strategy = strategy

        # Apply base strategy defaults
        main_b, side_b, suction, water = STRATEGY_DEFAULTS[strategy]
        self._cmd_main_brush = main_b
        self._cmd_side_brush = side_b
        self._cmd_suction = suction
        self._cmd_water = water
        self._adaptive_overrides.clear()

        self.get_logger().debug(
            f"Strategy set to {strategy.name}: "
            f"main={main_b:.2f} side={side_b:.2f} "
            f"suction={suction:.2f} water={water:.2f}"
        )

    def _on_detections(self, msg: DetectionArray):
        """Vision detection list -- used for adaptive parameter adjustment."""
        if not msg.detections:
            self._adaptive_overrides.clear()
            return

        # Collect distinct classes seen in this frame
        seen_classes = set()
        for det in msg.detections:
            class_name = det.class_name.lower().replace(" ", "_")
            if class_name in GARBAGE_CLASS_NAMES:
                seen_classes.add(GARBAGE_CLASS_NAMES[class_name])

        if not seen_classes:
            self._adaptive_overrides.clear()
            return

        # Apply per-class overrides (cumulative)
        overrides = {}
        for cls_id in seen_classes:
            if cls_id == GARBAGE_RECYCLABLE:
                overrides["main_brush"] = overrides.get("main_brush", 0.0) + 0.10
                overrides["side_brush"] = overrides.get("side_brush", 0.0) + 0.10
                overrides["suction"] = overrides.get("suction", 0.0) + 0.20
            elif cls_id == GARBAGE_KITCHEN:
                # "suck_then_mop" mode: suction=1.0, wait, then water+15%, brush start
                overrides["suction"] = 1.0  # absolute override
                overrides["water"] = overrides.get("water", 0.0) + 0.15
                overrides["main_brush"] = overrides.get("main_brush", 0.0)
                overrides["side_brush"] = overrides.get("side_brush", 0.0)
            elif cls_id == GARBAGE_HAZARDOUS:
                # Separate collection -- lower suction, publish WARN
                overrides["suction"] = 0.3  # absolute (low)
                self._publish_alert(
                    SEVERITY_WARN, "hazardous_detected",
                    "Hazardous waste detected; lowering suction for separate collection"
                )
            elif cls_id == GARBAGE_OTHER:
                # Use strategy defaults (no override)
                pass
            elif cls_id == GARBAGE_GREEN:
                overrides["suction"] = overrides.get("suction", 0.0) + 0.15
                overrides["water"] = overrides.get("water", 0.0) - 0.20
            elif cls_id == GARBAGE_STAIN:
                overrides["water"] = 1.0
                overrides["main_brush"] = 0.90
                overrides["suction"] = 0.0  # mop only

        self._adaptive_overrides = overrides
        self.get_logger().debug(f"Adaptive overrides applied: {overrides}")

    def _on_cleanliness(self, msg: Float32):
        """Track the latest cleanliness score."""
        self._action_last_cleanliness = max(0.0, min(1.0, msg.data))

    # =======================================================================
    # Service callback
    # =======================================================================

    def _on_get_status(self, request, response):
        response.node_name = self.get_name()
        response.uptime_s = self._uptime_seconds()

        if self._main_brush_stall or self._side_brush_stall:
            response.status = 2  # runtime warning
            response.status_text = (
                f"Stall detected: main={self._main_brush_stall}, "
                f"side={self._side_brush_stall}"
            )
        elif self._dust_level >= 1.0:
            response.status = 1  # needs attention
            response.status_text = "Dust bin full"
        elif self._suction_reduced:
            response.status = 1
            response.status_text = "Suction reduced due to overheat protection"
        else:
            response.status = 0  # nominal
            response.status_text = "OK"

        return response

    # =======================================================================
    # Action server -- ExecuteCleanup
    # =======================================================================

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received cleanup goal: strategy={goal_request.strategy}, "
            f"duration={goal_request.duration_s}s"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Cleanup cancelled by client")
        return CancelResponse.ACCEPT

    async def _execute_cleanup_cb(self, goal_handle):
        """Main action server callback -- runs the cleaning cycle."""
        request = goal_handle.request
        feedback_msg = ExecuteCleanup.Feedback()

        strategy = Strategy(request.strategy) if request.strategy < 4 else Strategy.IDLE
        duration_s = request.duration_s

        # Apply strategy defaults
        main_b, side_b, suction, water = STRATEGY_DEFAULTS[strategy]
        self._cmd_main_brush = main_b
        self._cmd_side_brush = side_b
        self._cmd_suction = suction
        self._cmd_water = water
        self._active_strategy = strategy
        self._adaptive_overrides.clear()

        # Reset statistics for this run
        initial_cleanliness = self._action_last_cleanliness
        self._waste_volume_l = 0.0
        self._suction_high_start = None
        self._suction_reduced = False
        run_start = self.get_clock().now()

        self._action_active = True
        self.get_logger().info(
            f"Cleanup started: strategy={strategy.name}, "
            f"duration={'indefinite' if duration_s <= 0 else f'{duration_s}s'}"
        )

        try:
            while rclpy.ok():
                # --- check for cancellation ---
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._stop_mechanisms()
                    self._action_active = False
                    self.get_logger().info("Cleanup cancelled")
                    return ExecuteCleanup.Result()

                # --- elapsed time ---
                elapsed = (self.get_clock().now() - run_start).nanoseconds * 1e-9

                # --- duration check ---
                if duration_s > 0 and elapsed >= duration_s:
                    self.get_logger().info(f"Duration reached: {elapsed:.1f}s")
                    break

                # --- dust full abort ---
                if self._dust_level >= 1.0:
                    self.get_logger().warn("Dust bin full -- aborting cleanup")
                    self._stop_mechanisms()
                    self._action_active = False
                    goal_handle.abort()
                    return ExecuteCleanup.Result(success=False)

                # --- apply adaptive overrides from vision ---
                effective_main = self._cmd_main_brush
                effective_side = self._cmd_side_brush
                effective_suction = self._cmd_suction
                effective_water = self._cmd_water

                for key, val in self._adaptive_overrides.items():
                    if key == "main_brush":
                        effective_main = max(0.0, min(1.0, self._cmd_main_brush + val))
                    elif key == "side_brush":
                        effective_side = max(0.0, min(1.0, self._cmd_side_brush + val))
                    elif key == "suction":
                        effective_suction = max(0.0, min(1.0, val if val <= 1.0 else 1.0))
                    elif key == "water":
                        effective_water = max(0.0, min(1.0, self._cmd_water + val))

                # --- suction overheat protection ---
                effective_suction = self._check_suction_overheat(effective_suction)

                # --- brush stall detection ---
                self._check_brush_stall(effective_main, "main")
                self._check_brush_stall(effective_side, "side")

                # --- update brush wear (accumulate operating time) ---
                dt = 1.0 / 3600.0  # 1 second in hours
                if effective_main > 0.01:
                    self._main_brush_accum_hours += dt
                if effective_side > 0.01:
                    self._side_brush_accum_hours += dt

                # --- simulate RPM ---
                self._main_brush_rpm = self._simulate_rpm(effective_main, self._main_brush_rpm)
                self._side_brush_rpm = self._simulate_rpm(effective_side, self._side_brush_rpm)

                # --- accumulate dust ---
                self._accumulate_dust(effective_suction, 1.0)
                self._filter_clog_ratio = min(1.0, self._dust_level * 0.95)

                # --- accumulate water usage ---
                water_per_s = self.get_parameter("max_dust_capacity_l").value * \
                    effective_water * 0.0001  # ~0.005 L/s at full water
                self._total_water_used_l += water_per_s

                # --- water tank empty check ---
                max_water = 100.0
                if self._total_water_used_l >= max_water:
                    self._publish_alert(
                        SEVERITY_WARN, "water_tank_empty",
                        f"Water tank empty ({self._total_water_used_l:.1f}L used)"
                    )
                    effective_water = 0.0

                # --- cleaning area accumulation ---
                cleaning_width = self.get_parameter("cleaning_width").value
                coverage_eff = self.get_parameter("coverage_efficiency").value
                # assume 0.3 m/s travel during active cleaning
                travel_speed = 0.3 if self._active_strategy != Strategy.IDLE else 0.0
                self._distance_traveled += travel_speed * 1.0  # per tick
                self._cleaned_area_m2 += cleaning_width * travel_speed * 1.0 * coverage_eff

                # --- waste volume ---
                max_cap = self.get_parameter("max_dust_capacity_l").value
                self._waste_volume_l = self._dust_level * max_cap

                # --- publish feedback ---
                feedback_msg.elapsed_s = float(elapsed)
                feedback_msg.cleanliness_current = self._action_last_cleanliness
                feedback_msg.waste_instant_l = self._waste_volume_l
                goal_handle.publish_feedback(feedback_msg)

                # Wait 1 second between ticks (non-blocking via async sleep)
                await self._sleep_async(1.0)

        except Exception as e:
            self.get_logger().error(f"Cleanup action exception: {e}")
            self._stop_mechanisms()
            self._action_active = False
            goal_handle.abort()
            return ExecuteCleanup.Result(success=False)

        # --- cleanup complete ---
        self._stop_mechanisms()
        final_cleanliness = self._action_last_cleanliness
        actual_duration = (self.get_clock().now() - run_start).nanoseconds * 1e-9
        self._action_active = False

        result = ExecuteCleanup.Result()
        result.success = True
        result.actual_duration = float(actual_duration)
        result.waste_collected_l = self._waste_volume_l
        result.avg_cleanliness_before = initial_cleanliness
        result.avg_cleanliness_after = final_cleanliness

        self.get_logger().info(
            f"Cleanup finished: duration={actual_duration:.1f}s, "
            f"waste={self._waste_volume_l:.2f}L, "
            f"cleanliness {initial_cleanliness:.2f}->{final_cleanliness:.2f}"
        )

        goal_handle.succeed()
        return result

    # =======================================================================
    # Periodic timers
    # =======================================================================

    def _tick_1hz(self):
        """1 Hz timer: heartbeat, dust, brush, progress."""
        now = self.get_clock().now()

        # Heartbeat
        self._publish_heartbeat()

        # Dust full
        self._publish_dust_full(now)

        # Brush status
        self._publish_brush_status(now)

        # Cleaning progress (only meaningful during action)
        if self._action_active:
            self._publish_progress(now)

    def _tick_02hz(self):
        """0.2 Hz timer: area and waste volume stats."""
        now = self.get_clock().now()

        # Area stats
        area_msg = Float32()
        area_msg.data = float(self._cleaned_area_m2)
        self._pub_area.publish(area_msg)

        # Waste volume stats
        waste_msg = Float32()
        waste_msg.data = float(self._waste_volume_l)
        self._pub_waste.publish(waste_msg)

    # =======================================================================
    # Publisher helpers
    # =======================================================================

    def _publish_heartbeat(self):
        msg = Heartbeat()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="")
        msg.node_name = self.get_name()

        state = self._state_machine.current_state
        if state[0] == LifecycleStateMsg.PRIMARY_STATE_ACTIVE:
            msg.lifecycle_state = int(LifecycleStateCode.ACTIVE)
        elif state[0] == LifecycleStateMsg.PRIMARY_STATE_INACTIVE:
            msg.lifecycle_state = int(LifecycleStateCode.INACTIVE)
        elif state[0] == LifecycleStateMsg.PRIMARY_STATE_UNCONFIGURED:
            msg.lifecycle_state = int(LifecycleStateCode.UNCONFIGURED)
        else:
            msg.lifecycle_state = int(LifecycleStateCode.FINALIZED)

        if self._main_brush_stall or self._side_brush_stall:
            msg.error_code = int(ErrorCode.RUNTIME_ERROR)
        elif self._dust_level >= 1.0:
            msg.error_code = int(ErrorCode.RUNTIME_ERROR)
        else:
            msg.error_code = int(ErrorCode.NORMAL)

        self._pub_heartbeat.publish(msg)

    def _publish_dust_full(self, now):
        msg = DustFull()
        msg.header = Header(stamp=now.to_msg(), frame_id="")
        msg.is_full = self._dust_level >= 1.0
        msg.dust_level = float(self._dust_level)
        msg.filter_clog_ratio = float(self._filter_clog_ratio)
        self._pub_dust.publish(msg)

    def _publish_brush_status(self, now):
        msg = BrushStatus()
        msg.header = Header(stamp=now.to_msg(), frame_id="")
        msg.main_brush_rpm = float(self._main_brush_rpm)
        msg.side_brush_rpm = float(self._side_brush_rpm)

        max_life = self.get_parameter("max_brush_life_hours").value
        msg.main_brush_wear = float(min(1.0, self._main_brush_accum_hours / max_life))
        msg.side_brush_wear = float(min(1.0, self._side_brush_accum_hours / max_life))
        self._pub_brush.publish(msg)

    def _publish_progress(self, now):
        msg = CleaningProgress()
        msg.header = Header(stamp=now.to_msg(), frame_id="")
        msg.area_id = "current"
        msg.progress = 0.0  # updated externally or via action context
        msg.cleaned_area_m2 = float(self._cleaned_area_m2)
        msg.estimated_remaining_s = 0.0
        self._pub_progress.publish(msg)

    def _publish_alert(self, severity, code, text):
        msg = Alert()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="")
        msg.source_node = self.get_name()
        msg.severity = severity
        msg.alert_code = code
        msg.alert_text = text
        self._pub_alert.publish(msg)
        self.get_logger().warn(f"ALERT [{code}]: {text}")

    # =======================================================================
    # Simulation helpers
    # =======================================================================

    def _accumulate_dust(self, suction_power, dt_s):
        """Simulate dust accumulation. Fills 0->1 in ~30 min at full suction."""
        fill_rate = self.get_parameter("dust_fill_rate_per_minute").value
        increment = fill_rate * suction_power * (dt_s / 60.0)
        self._dust_level = min(1.0, self._dust_level + increment)

    def _simulate_rpm(self, commanded, current_rpm):
        """Simulate RPM with first-order smoothing toward commanded speed + noise."""
        target_rpm = commanded * 1500.0  # max 1500 RPM
        noise = random.gauss(0, 15)      # measurement noise
        alpha = 0.3                      # smoothing factor
        return current_rpm + alpha * (target_rpm + noise - current_rpm)

    def _check_brush_stall(self, command, which):
        """Detect stall: command > 0.5 but RPM < 0.1*target for persisted duration."""
        # Simplified heuristic -- in real hardware this would use current sensing
        # For simulation, stall is triggered randomly with low probability
        stall_prob = 0.001  # 0.1% per check
        if command > 0.5 and random.random() < stall_prob:
            if which == "main" and not self._main_brush_stall:
                self._main_brush_stall = True
                self._publish_alert(
                    SEVERITY_WARN, "main_brush_stall",
                    "Main brush stall detected -- attempting recovery"
                )
                self.get_logger().warn("Main brush stall detected")
            elif which == "side" and not self._side_brush_stall:
                self._side_brush_stall = True
                self._publish_alert(
                    SEVERITY_WARN, "side_brush_stall",
                    "Side brush stall detected -- stopping side brush"
                )
                self.get_logger().warn("Side brush stall detected")

        # Simulate recovery: main brush tries reverse for 2s then retry
        if which == "main" and self._main_brush_stall:
            # Reverse attempt
            self._main_brush_rpm *= -0.2  # weak reverse
            # After "2 seconds" (simulated by random chance), attempt retry
            if random.random() < 0.05:  # ~5% chance per tick to recover
                self._main_brush_stall = False
                self.get_logger().info("Main brush stall recovered")
                self._publish_alert(
                    SEVERITY_INFO, "main_brush_recovered",
                    "Main brush stall recovered after reverse attempt"
                )

        # Side brush: just stop it, continue others
        if which == "side" and self._side_brush_stall:
            self._side_brush_rpm *= 0.5  # coast down

    def _check_suction_overheat(self, suction_cmd):
        """Simulate suction motor overheat protection."""
        now = self.get_clock().now()

        if suction_cmd > 0.8:
            if self._suction_high_start is None:
                self._suction_high_start = now
            high_duration = (now - self._suction_high_start).nanoseconds * 1e-9
            if high_duration > 300.0:  # 5 minutes
                if not self._suction_reduced:
                    self._suction_reduced = True
                    self._publish_alert(
                        SEVERITY_WARN, "suction_overheat",
                        "Suction motor overheat -- reducing to 50%"
                    )
                    self.get_logger().warn("Suction overheat protection activated")
                return 0.5 * suction_cmd
        else:
            self._suction_high_start = None
            if self._suction_reduced and suction_cmd < 0.5:
                self._suction_reduced = False
                self.get_logger().info("Suction overheat condition cleared")

        if self._suction_reduced:
            return 0.5 * suction_cmd
        return suction_cmd

    def _stop_mechanisms(self):
        """Stop all actuators."""
        self._cmd_main_brush = 0.0
        self._cmd_side_brush = 0.0
        self._cmd_suction = 0.0
        self._cmd_water = 0.0
        self._active_strategy = Strategy.IDLE
        self._adaptive_overrides.clear()
        self._main_brush_rpm *= 0.5
        self._side_brush_rpm *= 0.5

    async def _sleep_async(self, duration_s):
        """Asynchronous sleep for use within action server coroutine."""
        # Using rclpy's built-in sleep mechanism via the executor
        loop_start = self.get_clock().now()
        while (self.get_clock().now() - loop_start).nanoseconds * 1e-9 < duration_s:
            await rclpy.task.sleep(0.1)

    # =======================================================================
    # Utility
    # =======================================================================

    def _uptime_seconds(self):
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    @staticmethod
    def _destroy_timer(timer):
        if timer is not None:
            timer.cancel()
            timer.destroy()


# ---------------------------------------------------------------------------
# Alert severity codes
# ---------------------------------------------------------------------------

SEVERITY_INFO = 0
SEVERITY_WARN = 1
SEVERITY_ERROR = 2
SEVERITY_FATAL = 3


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = CleaningActuatorNode()

    try:
        # Manually drive lifecycle transitions for standalone usage.
        # In a composed system a lifecycle manager would handle this.
        node.on_configure(LifecycleStateMsg(
            id=LifecycleStateMsg.PRIMARY_STATE_UNCONFIGURED, label='unconfigured'))
        node.on_activate(LifecycleStateMsg(
            id=LifecycleStateMsg.PRIMARY_STATE_INACTIVE, label='inactive'))

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
