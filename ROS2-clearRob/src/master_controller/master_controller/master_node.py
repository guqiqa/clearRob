"""
master_controller.master_node - Main node for the cleaning robot master controller.

This is the "brain" of the system: it receives normalized commands from
control_gateway, runs the autonomous cruise state machine, arbitrates motion
commands from multiple sources, manages cleaning lifecycle, and handles system
fault degradation.
"""

import enum
import time
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

# Standard ROS2 message types
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path

# Custom interfaces (from cleaning_robot_interfaces)
from cleaning_robot_interfaces.msg import (
    UnifiedCommand,
    PlannerState,
    FaultStatus,
    DustFull,
    CleaningProgress,
    DetectionArray,
    Fusion3DTarget,
    TaskArea,
    TaskMode,
    CleanStrategy,
    MotionCommand,
    Alert,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import GetStatus
from cleaning_robot_interfaces.action import (
    NavigateToGoal,
    FollowPath,
    ExecuteCleanup,
)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class SystemState(enum.IntEnum):
    """8-state machine for the master controller."""
    STANDBY    = 0   # S0: idle, waiting for command
    PLANNING   = 1   # S1: path planning in progress
    CRUISING   = 2   # S2: autonomous cruise
    CLEANING   = 3   # S3: spot cleaning
    PAUSED     = 4   # S4: paused / safety hold
    RETURNING  = 5   # S5: returning to charging station
    CHARGING   = 6   # S6: charging at station
    ESTOP      = 7   # S7: emergency stop


STATE_NAMES = {
    SystemState.STANDBY:   "STANDBY",
    SystemState.PLANNING:  "PLANNING",
    SystemState.CRUISING:  "CRUISING",
    SystemState.CLEANING:  "CLEANING",
    SystemState.PAUSED:    "PAUSED",
    SystemState.RETURNING: "RETURNING",
    SystemState.CHARGING:  "CHARGING",
    SystemState.ESTOP:     "ESTOP",
}


# ---------------------------------------------------------------------------
# Helper: velocity limiting
# ---------------------------------------------------------------------------

def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# Master Controller Node
# ---------------------------------------------------------------------------

class MasterControllerNode(Node):
    """ROS2 Node implementing the master controller for the cleaning robot."""

    def __init__(self):
        super().__init__('master_controller')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('max_linear_velocity', 1.5)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('arbitration_rate', 50.0)
        self.declare_parameter('maneuver_timeout_s', 0.2)
        self.declare_parameter('cleanliness_pass_threshold', 0.7)
        self.declare_parameter('max_resweep_count', 3)
        self.declare_parameter('battery_warn_threshold', 0.3)
        self.declare_parameter('battery_force_return_threshold', 0.15)
        self.declare_parameter('battery_emergency_threshold', 0.05)
        self.declare_parameter('battery_full_threshold', 0.95)
        self.declare_parameter('localization_confidence_min', 0.3)
        self.declare_parameter('localization_degrade_timeout_s', 5.0)
        self.declare_parameter('heartbeat_timeout_s', 5.0)
        self.declare_parameter('recovery_hold_s', 10.0)
        self.declare_parameter('critical_alert_auto_resume_s', 5.0)
        self.declare_parameter('critical_alert_max_duration_s', 30.0)
        self.declare_parameter('charger_x', 0.0)
        self.declare_parameter('charger_y', 0.0)

        self._max_linear_vel = self.get_parameter('max_linear_velocity').value
        self._max_angular_vel = self.get_parameter('max_angular_velocity').value
        self._max_linear_accel = self.get_parameter('max_linear_accel').value
        self._max_angular_accel = self.get_parameter('max_angular_accel').value
        self._arbitration_rate = self.get_parameter('arbitration_rate').value
        self._maneuver_timeout = self.get_parameter('maneuver_timeout_s').value
        self._cleanliness_pass = self.get_parameter('cleanliness_pass_threshold').value
        self._max_resweep = self.get_parameter('max_resweep_count').value
        self._battery_warn = self.get_parameter('battery_warn_threshold').value
        self._battery_force = self.get_parameter('battery_force_return_threshold').value
        self._battery_emergency = self.get_parameter('battery_emergency_threshold').value
        self._battery_full = self.get_parameter('battery_full_threshold').value
        self._loc_conf_min = self.get_parameter('localization_confidence_min').value
        self._loc_degrade_timeout = self.get_parameter('localization_degrade_timeout_s').value
        self._heartbeat_timeout = self.get_parameter('heartbeat_timeout_s').value
        self._recovery_hold = self.get_parameter('recovery_hold_s').value
        self._critical_resume = self.get_parameter('critical_alert_auto_resume_s').value
        self._critical_max = self.get_parameter('critical_alert_max_duration_s').value

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._state: SystemState = SystemState.STANDBY
        self._pre_pause_state: SystemState = SystemState.STANDBY
        self._estop_released = False
        self._battery_pct: float = 1.0
        self._cleanliness_score: float = 0.0
        self._consecutive_clean_fail: int = 0
        self._resweep_count: int = 0
        self._localization_confidence: float = 1.0
        self._loc_low_start: Optional[float] = None
        self._last_planner_twist: Optional[Twist] = None
        self._last_planner_twist_time: float = 0.0
        self._last_manual_twist: Optional[Twist] = None
        self._last_manual_twist_time: float = 0.0
        self._manual_source: str = ""
        self._cmd_mode: str = "AUTO"
        self._critical_alert_start: Optional[float] = None
        self._fusion_alert_level: str = "NONE"
        self._fusion_alert_type: str = ""
        self._garbage_detected: bool = False
        self._dust_full: bool = False
        self._charging: bool = False
        self._cleanup_complete: bool = False
        self._planner_state_val: str = ""
        self._navigate_goal_completed: bool = False

        # Previous velocity for acceleration limiting
        self._prev_linear: float = 0.0
        self._prev_angular: float = 0.0
        self._last_arb_time: float = 0.0

        # Module heartbeats: {module_name: last_time}
        self._heartbeats: dict = {}
        self._module_faults: dict = {}
        # Map topic suffix to module name
        self._hb_module_map = {
            'vision_detector': 'vision_detector',
            'lidar_perception': 'lidar_perception',
            'path_planner': 'path_planner',
            'motion_controller': 'motion_controller',
            'localization': 'localization',
            'cleaning_actuator': 'cleaning_actuator',
            'fusion_engine': 'fusion_engine',
            'control_gateway': 'control_gateway',
            'master_controller': 'master_controller',
        }

        # Motion fault state
        self._motion_fault_reset: bool = False

        # ------------------------------------------------------------------
        # Callback groups
        # ------------------------------------------------------------------
        self._main_cb_group = MutuallyExclusiveCallbackGroup()
        self._motion_cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        qos_default = QoSProfile(depth=10)
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sub_unified = self.create_subscription(
            UnifiedCommand, 'control/cmd/unified',
            self._cb_unified, qos_reliable, callback_group=self._main_cb_group)

        self._sub_manual_vel = self.create_subscription(
            Twist, 'control/cmd/manual_velocity',
            self._cb_manual_vel, qos_default, callback_group=self._main_cb_group)

        self._sub_maneuver = self.create_subscription(
            Twist, 'planner/cmd/maneuver',
            self._cb_maneuver, qos_default, callback_group=self._motion_cb_group)

        self._sub_path = self.create_subscription(
            Path, 'planner/output/path',
            self._cb_path, qos_default, callback_group=self._main_cb_group)

        self._sub_planner_state = self.create_subscription(
            PlannerState, 'planner/status/state',
            self._cb_planner_state, qos_default, callback_group=self._main_cb_group)

        self._sub_motion_fault = self.create_subscription(
            FaultStatus, 'motion/status/fault',
            self._cb_motion_fault, qos_reliable, callback_group=self._main_cb_group)

        self._sub_odom = self.create_subscription(
            Odometry, 'motion/odom',
            self._cb_odom, qos_default, callback_group=self._main_cb_group)

        self._sub_dust_full = self.create_subscription(
            DustFull, 'cleaning/status/dust_full',
            self._cb_dust_full, qos_default, callback_group=self._main_cb_group)

        self._sub_cleaning_progress = self.create_subscription(
            CleaningProgress, 'cleaning/status/progress',
            self._cb_cleaning_progress, qos_default, callback_group=self._main_cb_group)

        self._sub_detections = self.create_subscription(
            DetectionArray, 'vision/detect/list',
            self._cb_detections, qos_default, callback_group=self._main_cb_group)

        self._sub_cleanliness = self.create_subscription(
            Float32, 'vision/detect/cleanliness',
            self._cb_cleanliness, qos_default, callback_group=self._main_cb_group)

        self._sub_loc_confidence = self.create_subscription(
            Float32, 'localization/confidence',
            self._cb_loc_confidence, qos_default, callback_group=self._main_cb_group)

        self._sub_fusion_target = self.create_subscription(
            Fusion3DTarget, 'fusion/target_3d',
            self._cb_fusion_target, qos_reliable, callback_group=self._main_cb_group)

        # Heartbeat subscription for our own heartbeat (self-monitoring)
        self._sub_heartbeat = self.create_subscription(
            Heartbeat, 'system/heartbeat/master_controller',
            self._cb_heartbeat, qos_default, callback_group=self._main_cb_group)

        # We also need a wildcard for other module heartbeats. In ROS2 we
        # subscribe to known topics since wildcards aren't native. We subscribe
        # to each known module heartbeat individually.
        self._hb_subs = []
        for module in ['vision_detector', 'lidar_perception', 'path_planner',
                       'motion_controller', 'localization', 'cleaning_actuator',
                       'fusion_engine', 'control_gateway']:
            sub = self.create_subscription(
                Heartbeat, f'system/heartbeat/{module}',
                self._make_hb_callback(module), qos_default,
                callback_group=self._main_cb_group)
            self._hb_subs.append(sub)

        # Initialize heartbeat timestamps
        now = self.get_clock().now().nanoseconds / 1e9
        for m in self._hb_module_map.values():
            self._heartbeats[m] = now

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_task_area = self.create_publisher(
            TaskArea, 'master/task/area', qos_reliable)
        self._pub_task_mode = self.create_publisher(
            TaskMode, 'master/task/mode', qos_reliable)
        self._pub_clean_strategy = self.create_publisher(
            CleanStrategy, 'master/task/clean_strategy', qos_reliable)
        self._pub_motion = self.create_publisher(
            MotionCommand, 'master/cmd/motion', qos_reliable)
        self._pub_status = self.create_publisher(
            PlannerState, 'master/status/state', qos_default)
        self._pub_alert = self.create_publisher(
            Alert, 'system/alert', qos_reliable)
        self._pub_hb = self.create_publisher(
            Heartbeat, 'system/heartbeat/master_controller', qos_default)

        # ------------------------------------------------------------------
        # Service
        # ------------------------------------------------------------------
        self.create_service(GetStatus, 'master/get_lifecycle_state',
                           self._srv_get_status, callback_group=self._main_cb_group)

        # ------------------------------------------------------------------
        # Action clients
        # ------------------------------------------------------------------
        self._ac_navigate = ActionClient(
            self, NavigateToGoal, 'planner/navigate_to_goal')
        self._ac_follow_path = ActionClient(
            self, FollowPath, 'motion/follow_path')
        self._ac_execute_cleanup = ActionClient(
            self, ExecuteCleanup, 'cleaning/execute_cleanup')

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self._arb_timer = self.create_timer(
            1.0 / self._arbitration_rate, self._timer_arbitration,
            callback_group=self._motion_cb_group)
        self._hb_timer = self.create_timer(
            1.0, self._timer_heartbeat, callback_group=self._main_cb_group)
        self._status_timer = self.create_timer(
            1.0, self._timer_status, callback_group=self._main_cb_group)
        self._monitor_timer = self.create_timer(
            1.0, self._timer_monitor, callback_group=self._main_cb_group)

        # Last maneuver time for timeout detection
        self._last_arb_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('MasterControllerNode initialized in STANDBY state')

    # ------------------------------------------------------------------
    # Heartbeat helper
    # ------------------------------------------------------------------
    def _make_hb_callback(self, module_name: str):
        def cb(msg: Heartbeat):
            self._heartbeats[module_name] = (
                self.get_clock().now().nanoseconds / 1e9)
        return cb

    def _cb_heartbeat(self, msg: Heartbeat):
        """Handle our own heartbeat echo for self-monitoring."""
        pass  # Reserved for future heartbeat correlation

    # ------------------------------------------------------------------
    # Subscription callbacks (lightweight, just store data)
    # ------------------------------------------------------------------
    def _cb_unified(self, msg: UnifiedCommand):
        """Handle normalized command from control_gateway."""
        cmd_type = msg.command_type.upper()
        if cmd_type == 'START':
            self._handle_start(msg)
        elif cmd_type == 'STOP':
            self._handle_stop()
        elif cmd_type == 'PAUSE':
            self._handle_pause()
        elif cmd_type == 'RESUME':
            self._handle_resume()
        elif cmd_type == 'RETURN':
            self._handle_return()
        elif cmd_type == 'ESTOP':
            self._handle_estop()
        elif cmd_type == 'ESTOP_RELEASE':
            self._handle_estop_release()
        elif cmd_type == 'MANUAL':
            self._cmd_mode = 'MANUAL'
        elif cmd_type == 'AUTO':
            self._cmd_mode = 'AUTO'

    def _cb_manual_vel(self, msg: Twist):
        self._last_manual_twist = msg
        self._last_manual_twist_time = self.get_clock().now().nanoseconds / 1e9
        self._manual_source = 'RC'

    def _cb_maneuver(self, msg: Twist):
        self._last_planner_twist = msg
        self._last_planner_twist_time = self.get_clock().now().nanoseconds / 1e9

    def _cb_path(self, msg: Path):
        pass  # Stored for reference; planner state drives transitions

    def _cb_planner_state(self, msg: PlannerState):
        self._planner_state_val = msg.state_text.upper()
        if self._state == SystemState.PLANNING and self._planner_state_val == 'READY':
            self._transition_to(SystemState.CRUISING)
        elif self._state == SystemState.RETURNING and self._planner_state_val == 'COMPLETED':
            self._transition_to(SystemState.CHARGING)

    def _cb_motion_fault(self, msg: FaultStatus):
        if msg.requires_reset:
            self._motion_fault_reset = True
            self._publish_alert('MOTION_FAULT', f'{msg.fault_code}: {msg.fault_text}')
            self._transition_to(SystemState.ESTOP)
        else:
            self.get_logger().warn(f'Motion non-critical fault: {msg.fault_text}')

    def _cb_odom(self, msg: Odometry):
        pass  # Battery estimation handled in monitor timer

    def _cb_dust_full(self, msg: DustFull):
        self._dust_full = msg.is_full
        if self._dust_full and self._state in (SystemState.CRUISING, SystemState.CLEANING):
            self._publish_alert('DUST_FULL', 'Dust bin full, returning to station')
            self._transition_to(SystemState.RETURNING)

    def _cb_cleaning_progress(self, msg: CleaningProgress):
        self._cleanup_complete = (msg.status == 'COMPLETE')

    def _cb_detections(self, msg: DetectionArray):
        self._garbage_detected = len(msg.detections) > 0

    def _cb_cleanliness(self, msg: Float32):
        self._cleanliness_score = msg.data

    def _cb_loc_confidence(self, msg: Float32):
        self._localization_confidence = msg.data
        if msg.data < self._loc_conf_min:
            if self._loc_low_start is None:
                self._loc_low_start = self.get_clock().now().nanoseconds / 1e9
        else:
            self._loc_low_start = None

    def _cb_fusion_target(self, msg: Fusion3DTarget):
        self._fusion_alert_level = 'CRITICAL' if msg.alert_level >= 2 else 'WARN' if msg.alert_level >= 1 else 'NONE'
        self._fusion_alert_type = msg.target_class.upper()
        if self._fusion_alert_level == 'CRITICAL':
            if 'PEDESTRIAN' in self._fusion_alert_type:
                self._critical_alert_start = self.get_clock().now().nanoseconds / 1e9
                if self._state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                    self._transition_to(SystemState.PAUSED)
                    self._publish_alert('CRITICAL_PEDESTRIAN', 'Pedestrian approaching, pausing')
            elif 'IMPASSABLE' in self._fusion_alert_type:
                self._send_reroute_area()
                self._publish_alert('CRITICAL_OBSTACLE', 'Impassable obstacle, rerouting')

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------
    def _handle_start(self, msg: UnifiedCommand):
        if self._state == SystemState.STANDBY:
            self._publish_task_mode('CRUISE')
            self._transition_to(SystemState.PLANNING)
        elif self._state == SystemState.CHARGING:
            if self._battery_pct >= 0.3:
                self._publish_task_mode('CRUISE')
                self._transition_to(SystemState.PLANNING)
            else:
                self.get_logger().warn('Battery too low for task start')
        else:
            self.get_logger().warn(f'Cannot START from state {STATE_NAMES[self._state]}')

    def _handle_stop(self):
        if self._state in (SystemState.PLANNING, SystemState.CRUISING, SystemState.CLEANING,
                           SystemState.PAUSED):
            self._transition_to(SystemState.STANDBY)
        else:
            self.get_logger().warn(f'Cannot STOP from state {STATE_NAMES[self._state]}')

    def _handle_pause(self):
        if self._state in (SystemState.CRUISING, SystemState.CLEANING):
            self._pre_pause_state = self._state
            self._zero_velocity()
            self._transition_to(SystemState.PAUSED)
        elif self._state == SystemState.PLANNING:
            self._cancel_navigate()
            self._zero_velocity()
            self._pre_pause_state = SystemState.PLANNING
            self._transition_to(SystemState.PAUSED)

    def _handle_resume(self):
        if self._state == SystemState.PAUSED:
            if self._pre_pause_state in (SystemState.CRUISING, SystemState.CLEANING):
                self._transition_to(self._pre_pause_state)
                self._resume_follow_path()
            elif self._pre_pause_state == SystemState.PLANNING:
                self._transition_to(SystemState.PLANNING)
                self._restart_navigate()
            else:
                self._transition_to(SystemState.STANDBY)

    def _handle_return(self):
        if self._state in (SystemState.STANDBY, SystemState.CRUISING, SystemState.CLEANING,
                           SystemState.PLANNING, SystemState.PAUSED):
            self._transition_to(SystemState.RETURNING)

    def _handle_estop(self):
        self._publish_alert('ESTOP', 'Emergency stop activated')
        self._transition_to(SystemState.ESTOP)

    def _handle_estop_release(self):
        if self._state == SystemState.ESTOP:
            self._estop_released = True
            self._motion_fault_reset = False
            self._transition_to(SystemState.STANDBY)

    # ------------------------------------------------------------------
    # State machine engine
    # ------------------------------------------------------------------
    def _transition_to(self, new_state: SystemState):
        old_state = self._state
        if old_state == new_state:
            return

        # Exit actions
        self._on_exit_state(old_state)

        self.get_logger().info(
            f'State transition: {STATE_NAMES[old_state]} -> {STATE_NAMES[new_state]}')
        self._state = new_state

        # Entry actions
        self._on_enter_state(new_state)

    def _on_exit_state(self, state: SystemState):
        if state == SystemState.CRUISING:
            self._cancel_follow_path()

    def _on_enter_state(self, state: SystemState):
        if state == SystemState.PLANNING:
            self._start_navigate()
        elif state == SystemState.CRUISING:
            self._start_follow_path()
        elif state == SystemState.RETURNING:
            self._start_return_navigate()
        elif state == SystemState.CHARGING:
            self._publish_task_mode('STANDBY')
            self._charging = True
        elif state == SystemState.PAUSED:
            self._zero_velocity()
        elif state == SystemState.ESTOP:
            self._publish_motion_estop(True)
        elif state == SystemState.STANDBY:
            self._publish_task_mode('STANDBY')
            self._zero_velocity()

    # ------------------------------------------------------------------
    # Action client wrappers
    # ------------------------------------------------------------------
    def _start_navigate(self):
        if not self._ac_navigate.server_is_ready():
            self.get_logger().warn('NavigateToGoal action server not ready')
            return
        goal = NavigateToGoal.Goal()
        goal.target_area = 'cleaning_zone'
        self._ac_navigate.send_goal_async(goal)

    def _cancel_navigate(self):
        self._ac_navigate._cancel_goal_async(None)  # Best-effort cancel

    def _restart_navigate(self):
        self._start_navigate()

    def _start_return_navigate(self):
        if not self._ac_navigate.server_is_ready():
            self.get_logger().warn('NavigateToGoal action server not ready for return')
            return
        goal = NavigateToGoal.Goal()
        goal.target_area = 'charging_station'
        self._ac_navigate.send_goal_async(goal)

    def _start_follow_path(self):
        if not self._ac_follow_path.server_is_ready():
            self.get_logger().warn('FollowPath action server not ready')
            return
        goal = FollowPath.Goal()
        # Path is sent by the planner; this triggers motion
        self._ac_follow_path.send_goal_async(goal)

    def _cancel_follow_path(self):
        self._ac_follow_path._cancel_goal_async(None)

    def _resume_follow_path(self):
        self._start_follow_path()

    def _execute_cleanup(self):
        if not self._ac_execute_cleanup.server_is_ready():
            self.get_logger().warn('ExecuteCleanup action server not ready')
            return
        self._publish_clean_strategy('SPOT')
        goal = ExecuteCleanup.Goal()
        self._ac_execute_cleanup.send_goal_async(goal)

    # ------------------------------------------------------------------
    # Auto-detection triggers (called from monitor timer)
    # ------------------------------------------------------------------
    def _check_auto_triggers(self):
        """Check conditions that span states and trigger transitions."""
        now = self.get_clock().now().nanoseconds / 1e9

        # Global: motion fault
        if self._motion_fault_reset:
            self._transition_to(SystemState.ESTOP)
            return

        # Global: localization confidence low for extended period
        if self._localization_confidence < self._loc_conf_min and self._loc_low_start is not None:
            if (now - self._loc_low_start) > self._loc_degrade_timeout:
                if self._state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                    self._publish_alert('LOC_DEGRADE', 'Localization degraded, pausing')
                    self._pre_pause_state = self._state
                    self._transition_to(SystemState.PAUSED)
                    self._loc_low_start = None

        # Global: battery
        if self._battery_pct <= self._battery_emergency:
            self._publish_alert('BATTERY_EMERGENCY', 'Emergency battery level')
            if self._state != SystemState.ESTOP:
                self._transition_to(SystemState.RETURNING)

        # State-specific triggers
        state = self._state

        if state == SystemState.CRUISING:
            # Garbage detected -> clean
            if self._garbage_detected and not self._dust_full:
                self._consecutive_clean_fail = 0
                self._resweep_count = 0
                self._transition_to(SystemState.CLEANING)
                self._execute_cleanup()
            # Battery low -> return
            if self._battery_pct <= self._battery_force:
                self._publish_alert('BATTERY_LOW', 'Forcing return due to low battery')
                self._transition_to(SystemState.RETURNING)
            # Dust full -> return
            if self._dust_full:
                self._transition_to(SystemState.RETURNING)

        elif state == SystemState.CLEANING:
            if self._cleanup_complete:
                if self._cleanliness_score >= self._cleanliness_pass:
                    # Success: go back to cruising
                    self._consecutive_clean_fail = 0
                    self._resweep_count = 0
                    self._publish_clean_strategy('IDLE')
                    self._transition_to(SystemState.CRUISING)
                else:
                    self._consecutive_clean_fail += 1
                    if self._consecutive_clean_fail >= 3:
                        self._resweep_count += 1
                        if self._resweep_count < self._max_resweep:
                            self.get_logger().warn(
                                f'Cleanliness insufficient, resweep {self._resweep_count}/{self._max_resweep}')
                            self._execute_cleanup()
                        else:
                            self.get_logger().error('Max resweep reached, giving up on spot')
                            self._publish_clean_strategy('IDLE')
                            self._transition_to(SystemState.CRUISING)
                    else:
                        self.get_logger().info('Retry cleaning after cleanliness fail')
                        self._execute_cleanup()
            # Also check battery/dust during cleaning
            if self._battery_pct <= self._battery_force or self._dust_full:
                self._transition_to(SystemState.RETURNING)

        elif state == SystemState.CHARGING:
            if self._charging and self._battery_pct >= self._battery_full:
                self._charging = False
                self.get_logger().info('Battery fully charged')
            # Full charge + auto-start could happen here, but we wait for explicit START

        # Critical alert sustained
        if self._critical_alert_start is not None:
            elapsed = now - self._critical_alert_start
            if elapsed > self._critical_max:
                self._publish_alert('CRITICAL_SUSTAINED', 'Critical alert sustained, stop task')
                self._critical_alert_start = None
                if self._state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                    self._pre_pause_state = self._state
                    self._transition_to(SystemState.PAUSED)
            elif (elapsed > self._critical_resume and
                  'PEDESTRIAN' in self._fusion_alert_type and
                  self._state == SystemState.PAUSED):
                # Auto-resume from pedestrian pause
                self._critical_alert_start = None
                self._resume()

    def _resume(self):
        """Internal resume after auto-pause."""
        if self._state == SystemState.PAUSED and self._pre_pause_state in (
                SystemState.CRUISING, SystemState.CLEANING):
            self._transition_to(self._pre_pause_state)

    # ------------------------------------------------------------------
    # Module fault degradation check
    # ------------------------------------------------------------------
    def _check_module_faults(self):
        now = self.get_clock().now().nanoseconds / 1e9
        state = self._state

        # vision_detector timeout
        if now - self._heartbeats.get('vision_detector', now) > self._heartbeat_timeout:
            self._module_faults['vision_detector'] = True
            self._garbage_detected = False  # disable garbage-triggered cleaning
        else:
            self._module_faults['vision_detector'] = False

        # lidar_perception timeout -> safety critical, force pause
        if now - self._heartbeats.get('lidar_perception', now) > self._heartbeat_timeout:
            self._module_faults['lidar_perception'] = True
            if state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                self._publish_alert('LIDAR_TIMEOUT', 'Lidar perception timeout, pausing')
                self._pre_pause_state = state
                self._transition_to(SystemState.PAUSED)
        else:
            self._module_faults['lidar_perception'] = False

        # path_planner timeout or BLOCKED
        planner_hb_ok = (now - self._heartbeats.get('path_planner', now)) <= self._heartbeat_timeout
        planner_blocked = (self._planner_state_val.upper() == 'BLOCKED')
        if not planner_hb_ok or planner_blocked:
            self._module_faults['path_planner'] = True
            self._zero_velocity()
            if state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                self._pre_pause_state = state
                self._transition_to(SystemState.PAUSED)
        else:
            self._module_faults['path_planner'] = False

        # motion_controller fault already handled via FaultStatus msg

        # cleaning_actuator timeout
        if now - self._heartbeats.get('cleaning_actuator', now) > self._heartbeat_timeout:
            self._module_faults['cleaning_actuator'] = True
            # Mark cleaning unavailable, cruise only
        else:
            self._module_faults['cleaning_actuator'] = False

        # fusion_engine timeout
        if now - self._heartbeats.get('fusion_engine', now) > self._heartbeat_timeout:
            self._module_faults['fusion_engine'] = True
            # Use lidar obstacle list directly (handled by downstream)
        else:
            self._module_faults['fusion_engine'] = False

        # control_gateway timeout -> no command source
        if now - self._heartbeats.get('control_gateway', now) > self._heartbeat_timeout:
            self._module_faults['control_gateway'] = True
            if state not in (SystemState.PAUSED, SystemState.ESTOP, SystemState.STANDBY):
                self._pre_pause_state = state
                self._transition_to(SystemState.PAUSED)
        else:
            self._module_faults['control_gateway'] = False

    # ------------------------------------------------------------------
    # Motion arbitration (50Hz timer)
    # ------------------------------------------------------------------
    def _timer_arbitration(self):
        """Run motion arbitration at arbitration_rate Hz."""
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._last_arb_time if self._last_arb_time > 0 else 0.02
        self._last_arb_time = now

        state = self._state
        cmd = MotionCommand()

        if state == SystemState.ESTOP:
            cmd.estop = True
            self._prev_linear = 0.0
            self._prev_angular = 0.0
            self._publish_motion(cmd)
            return

        # Determine desired linear and angular
        desired_linear = 0.0
        desired_angular = 0.0

        # Priority 1: manual velocity (MANUAL mode, RC source)
        manual_valid = (
            self._last_manual_twist is not None and
            self._manual_source == 'RC' and
            (now - self._last_manual_twist_time) < self._maneuver_timeout
        )
        manual_allowed = (
            self._cmd_mode == 'MANUAL' and
            state not in (SystemState.ESTOP, SystemState.RETURNING, SystemState.CLEANING) and
            (self._localization_confidence >= self._loc_conf_min or self._cmd_mode == 'MANUAL')
        )

        planner_valid = (
            self._last_planner_twist is not None and
            (now - self._last_planner_twist_time) < self._maneuver_timeout
        )
        planner_required = state in (SystemState.RETURNING, SystemState.CLEANING)

        if manual_valid and manual_allowed and not planner_required:
            desired_linear = self._last_manual_twist.linear.x
            desired_angular = self._last_manual_twist.angular.z
        elif planner_valid:
            desired_linear = self._last_planner_twist.linear.x
            desired_angular = self._last_planner_twist.angular.z
        else:
            # Zero velocity
            desired_linear = 0.0
            desired_angular = 0.0

        # Limit velocity
        desired_linear = clamp(desired_linear, self._max_linear_vel)
        desired_angular = clamp(desired_angular, self._max_angular_vel)

        # Limit acceleration
        max_delta_linear = self._max_linear_accel * dt
        max_delta_angular = self._max_angular_accel * dt

        delta_linear = clamp(desired_linear - self._prev_linear, max_delta_linear)
        delta_angular = clamp(desired_angular - self._prev_angular, max_delta_angular)

        final_linear = self._prev_linear + delta_linear
        final_angular = self._prev_angular + delta_angular

        self._prev_linear = final_linear
        self._prev_angular = final_angular

        cmd.estop = False
        cmd.linear_velocity = final_linear
        cmd.angular_velocity = final_angular
        self._publish_motion(cmd)

    def _zero_velocity(self):
        self._prev_linear = 0.0
        self._prev_angular = 0.0
        cmd = MotionCommand()
        cmd.estop = False
        cmd.linear_velocity = 0.0
        cmd.angular_velocity = 0.0
        self._publish_motion(cmd)

    # ------------------------------------------------------------------
    # Timer: heartbeat
    # ------------------------------------------------------------------
    def _timer_heartbeat(self):
        hb = Heartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.header.frame_id = ''
        hb.node_name = 'master_controller'
        hb.lifecycle_state = 3  # ACTIVE
        hb.error_code = 0
        self._pub_hb.publish(hb)

        # Also update our own heartbeat
        self._heartbeats['master_controller'] = (
            self.get_clock().now().nanoseconds / 1e9)

    # ------------------------------------------------------------------
    # Timer: status (1Hz)
    # ------------------------------------------------------------------
    def _timer_status(self):
        ps = PlannerState()
        ps.state = int(self._state)
        ps.state_text = STATE_NAMES[self._state]
        self._pub_status.publish(ps)

    # ------------------------------------------------------------------
    # Timer: monitor (1Hz) - battery sim, fault checks, auto triggers
    # ------------------------------------------------------------------
    def _timer_monitor(self):
        # Battery simulation
        if self._charging:
            self._battery_pct = min(1.0, self._battery_pct + 0.01)  # charge at ~1%/s for sim speed
        elif self._state in (SystemState.CRUISING, SystemState.CLEANING, SystemState.RETURNING):
            self._battery_pct -= (5.0 / 100.0) / 3600.0  # 5%/hour real
        elif self._state in (SystemState.STANDBY, SystemState.PAUSED):
            self._battery_pct -= (1.0 / 100.0) / 3600.0
        self._battery_pct = max(0.0, self._battery_pct)

        # Emergency battery
        if self._battery_pct <= self._battery_emergency:
            pass  # Handled in _check_auto_triggers

        # Module fault degradation
        self._check_module_faults()

        # Auto-transition triggers
        self._check_auto_triggers()

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------
    def _srv_get_status(self, request, response):
        response.state = STATE_NAMES[self._state]
        response.battery = self._battery_pct
        response.success = True
        return response

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------
    def _publish_task_area(self, area_name: str):
        msg = TaskArea()
        msg.area_name = area_name
        self._pub_task_area.publish(msg)

    def _publish_task_mode(self, mode: str):
        msg = TaskMode()
        msg.mode = mode
        self._pub_task_mode.publish(msg)

    def _publish_clean_strategy(self, strategy: str):
        msg = CleanStrategy()
        msg.strategy = strategy
        self._pub_clean_strategy.publish(msg)

    def _publish_motion(self, cmd: MotionCommand):
        self._pub_motion.publish(cmd)

    def _publish_motion_estop(self, estop: bool):
        cmd = MotionCommand()
        cmd.estop = estop
        cmd.linear_velocity = 0.0
        cmd.angular_velocity = 0.0
        self._publish_motion(cmd)
        self._prev_linear = 0.0
        self._prev_angular = 0.0

    def _publish_alert(self, code: str, description: str):
        alert = Alert()
        alert.header.stamp = self.get_clock().now().to_msg()
        alert.header.frame_id = ''
        alert.source_node = 'master_controller'
        alert.severity = 2  # ERROR
        alert.alert_code = code
        alert.alert_text = description
        self._pub_alert.publish(alert)
        self.get_logger().warn(f'ALERT [{code}]: {description}')

    def _send_reroute_area(self):
        """Send a new SET_AREA to path_planner for rerouting."""
        self._publish_task_area('reroute_zone')
        self.get_logger().info('Sent reroute area to path_planner')

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('MasterControllerNode shutting down')
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MasterControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
