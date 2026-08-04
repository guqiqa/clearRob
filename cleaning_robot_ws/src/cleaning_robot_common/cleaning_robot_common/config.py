#!/usr/bin/env python3
"""
cleaning_robot_common/config.py — Unified configuration for all Phase 1 nodes.

Single source of truth for:
  - CAN bus IDs, protocol constants
  - Motor kinematics parameters
  - Topic names (pub/sub)
  - Safety limits

All nodes import from here instead of hardcoding magic numbers.
"""

# =========================================================================
# CAN Bus — Motor Assignment (OID FOC protocol)
# =========================================================================

CAN_ID_LEFT_REAR   = 1    # 001 = 左后轮
CAN_ID_LEFT_FRONT  = 2    # 002 = 左前轮
CAN_ID_RIGHT_REAR  = 3    # 003 = 右后轮
CAN_ID_RIGHT_FRONT = 4    # 004 = 右前轮
CAN_ID_CUTTER      = 5    # 刀盘 (Phase 2)

DRIVE_IDS = [CAN_ID_LEFT_REAR, CAN_ID_LEFT_FRONT,
             CAN_ID_RIGHT_REAR, CAN_ID_RIGHT_FRONT]
LEFT_IDS  = [CAN_ID_LEFT_FRONT, CAN_ID_LEFT_REAR]
RIGHT_IDS = [CAN_ID_RIGHT_FRONT, CAN_ID_RIGHT_REAR]

# =========================================================================
# CAN Protocol — Frame types
# =========================================================================

CAN_CMD_HEARTBEAT    = 0x00   # Keep-alive (DLC=1)
CAN_CMD_CURRENT      = 0x01   # Current control (DLC=3, int16 in 10mA units)
CAN_CMD_SPEED        = 0x02   # Speed control, driver closed-loop (DLC=5, int32 erpm BE)
CAN_CMD_SET_ACCEL    = 0x0A   # Driver acceleration ramp (DLC=5, int32 erpm/s BE)
CAN_CMD_SET_DECEL    = 0x10   # Driver deceleration ramp (DLC=5, int32 erpm/s BE)
CAN_CMD_SET_MAX_CURRENT = 0x14  # Driver closed-loop max current (DLC=3, uint16 BE)
CAN_CMD_QUERY        = 0x0F   # Query command (DLC=2)
CAN_QUERY_SPEED      = 0x01   # Query sub-code: motor speed (erpm, int32)
CAN_QUERY_FAULT      = 0x00   # Query sub-code: fault info
CAN_QUERY_TEMP       = 0x07   # Query sub-code: temperature (°C)
CAN_QUERY_CURRENT    = 0x05   # Query sub-code: motor current (10mA)

# Query response layout
QUERY_RESP_MIN_LEN   = 6      # DLC >= 6 bytes
QUERY_RESP_ERPM_OFF  = 2      # erpm at DATA[2:6] as int32 BIG-endian (see can_protocol)
QUERY_RESP_CMD_BYTE  = 0      # DATA[0] should == 0x0F

# Current scaling
CURRENT_UNIT_MA      = 10     # CAN current: 1 unit = 10mA

# =========================================================================
# Motor Kinematics (confirmed hardware specs)
# =========================================================================

# Motor specs (confirmed from hardware datasheet)
MOTOR_PHASE_RESISTANCE = 0.8     # Ω, phase resistance
MOTOR_PHASE_INDUCTANCE = 1.6     # mH, phase inductance
MOTOR_HALL_ANGLE       = 120     # degrees, Hall sensor electrical angle
MOTOR_POLE_PAIRS       = 10      # 20-pole motor → 10 pole pairs
MOTOR_GEAR_RATIO       = 5.2     # 5.2:1 reduction gearbox
WHEEL_RADIUS_M       = 0.10   # 200 mm wheel diameter (vendor WheelPID V2 config)
TRACK_WIDTH_M        = 0.45   # 450 mm track width (vendor WheelPID V2 config)

# Velocity limits
MAX_LINEAR_VELOCITY  = 0.5    # m/s (estimated, needs field calibration)
MAX_ANGULAR_VELOCITY = 2.0    # rad/s
MAX_MOTOR_CURRENT    = 500    # ×10mA = 5.0A max
CURRENT_DEADZONE     = 10     # ×10mA = 0.1A dead zone

# =========================================================================
# Timing
# =========================================================================

CAN_QUERY_INTERVAL_MS    = 20    # Speed query period
CAN_HEARTBEAT_INTERVAL_MS = 40   # CAN heartbeat period (was 10ms, too fast for 500Kbps)
CMD_VEL_TIMEOUT_MS        = 500  # /cmd_vel timeout → emergency stop
CAN_QUERY_TIMEOUT_MS      = 100  # Query response timeout
ODOM_PUBLISH_HZ           = 50   # Odom / TF publish rate
IMU_PUBLISH_HZ            = 200  # IMU data rate

# =========================================================================
# Wheel Speed-Loop Control (WheelPID V2 integration, verified on hardware)
# =========================================================================
# Ported from the C++ "new_progame_wheel_pid_v2" program. Two modes:
#   "current" — software PI on top of OID current control (0x01), with a
#               startup current ramp and stall detection.  THIS is the mode
#               the vendor WheelPID V2 actually runs (all its configs use it).
#   "speed"   — driver internal closed-loop (OID cmd 0x02); software only does
#               slew-rate ramping.  The driver firmware on this robot does NOT
#               respond to 0x02 speed commands (verified on the bus), so this
#               mode must NOT be used with the current drivers.
# Wheel RPM here is PHYSICAL wheel RPM (post gearbox).
# ERPM = wheel_rpm * pole_pairs * gear_ratio * motor_dir.
#
# Defaults match the vendor config.xml on the robot (current mode):
#   Run_RPM 100, Kp 0.60, Ki 0.05, FF 24, run-max 0.35A, start-max 0.70A,
#   accel 10 rpm/s, decel 30 rpm/s, overspeed 120, wheel 0.10 m, track 0.45 m.

WHEEL_CONTROL_MODE = "current"       # "current" | "speed"

# Driver closed-loop (speed mode — NOT supported by current driver firmware)
SPEED_ACCEL_RPM_S  = 30              # target ramp: wheel RPM/s while accelerating
SPEED_DECEL_RPM_S  = 30              # target ramp: wheel RPM/s while decelerating
WHEEL_MAX_CURRENT  = 35              # ×10mA = 0.35A, software PI run limit

# Software PI (current mode)
WHEEL_SPEED_KP           = 0.60      # current per RPM error
WHEEL_SPEED_KI           = 0.05      # integral gain
WHEEL_FEEDFORWARD_CURRENT = 24       # ×10mA feed-forward at full target speed
WHEEL_INTEGRAL_MAX_CURRENT = 10      # ×10mA integral clamp

# Startup current ramp (current mode) — breaks static friction.
# EXACT vendor config.xml values.  Tuning away from these (tried 60/4/80 + a
# 4rpm threshold, 2026-08-03) made startup stutter WORSE: the big start
# current + late handoff to PI made the PI slam on reverse current.  Startup
# stutter is fixed in wheel_speed_loop by catching ramped_target up to the
# measured wheel speed at the start→PI handoff (error≈0 → no reverse brake),
# NOT by tuning the start current magnitudes.
WHEEL_START_INITIAL_CURRENT = 55     # ×10mA
WHEEL_START_STEP_CURRENT   = 2       # ×10mA added per step
WHEEL_START_STEP_MS        = 200
WHEEL_START_MAX_CURRENT    = 70      # ×10mA
WHEEL_START_THRESHOLD_RPM  = 2       # wheel RPM that counts as "started"
WHEEL_START_CONFIRM_SAMPLES = 1
WHEEL_STALL_THRESHOLD_RPM  = 1
WHEEL_STALL_CONFIRM_SAMPLES = 3
WHEEL_START_TIMEOUT_MS     = 7000

# Safety
WHEEL_OVERSPEED_RPM        = 120     # latches stop if any wheel exceeds this
SPEED_FEEDBACK_TIMEOUT_MS  = 1000    # feedback older than this → stop that wheel
CONTROL_INTERVAL_MS        = 50      # control loop period

# Per-motor physical direction — calibrated 2026-08-03 by driving cmd_vel
# x=+0.15 (forward) and observing the car actually REVERSED, so all dirs are
# flipped.  Keyed by CAN id: {2: FL, 1: RL, 4: FR, 3: RR}.  Positive = the
# current polarity that makes the wheel drive the car FORWARD.
MOTOR_DIRS = {2: -1, 1: -1, 4: 1, 3: 1}

# Per-motor speed gain — calibrated from 0.2 m/s straight runs (2026-08-03).
# Iteration 1 (no gains):  RL 837, FL 890, RR -917, FR -954 → right +8.3%.
# Iteration 2 (gains 1.072/1.139/1.041/1.000):  RL 853, FL 892, RR -965, FR -886
#   → right still +6.1%, so gains were under-correcting.  New gains normalize
#   each wheel to the fastest measured (FR-RR ~930).  Applied to TARGET wheel
#   RPM so slow wheels command more current and all four converge.
MOTOR_GAINS = {2: 1.082, 1: 1.131, 4: 1.089, 3: 1.000}

# One dict of all wheel speed-loop params — single source for both the
# chassis_driver node (ROS params) and the pure-logic controller used by
# tests / simulation.
WHEEL_SPEED_PARAMS = {
    "pole_pairs": MOTOR_POLE_PAIRS,
    "gear_ratio": MOTOR_GEAR_RATIO,
    "wheel_radius": WHEEL_RADIUS_M,
    "track_width": TRACK_WIDTH_M,
    "wheel_control_mode": WHEEL_CONTROL_MODE,
    "speed_accel_rpm_s": SPEED_ACCEL_RPM_S,
    "speed_decel_rpm_s": SPEED_DECEL_RPM_S,
    "wheel_max_current": WHEEL_MAX_CURRENT,
    "wheel_speed_kp": WHEEL_SPEED_KP,
    "wheel_speed_ki": WHEEL_SPEED_KI,
    "wheel_feedforward_current": WHEEL_FEEDFORWARD_CURRENT,
    "wheel_integral_max_current": WHEEL_INTEGRAL_MAX_CURRENT,
    "wheel_start_initial_current": WHEEL_START_INITIAL_CURRENT,
    "wheel_start_step_current": WHEEL_START_STEP_CURRENT,
    "wheel_start_step_ms": WHEEL_START_STEP_MS,
    "wheel_start_max_current": WHEEL_START_MAX_CURRENT,
    "wheel_start_threshold_rpm": WHEEL_START_THRESHOLD_RPM,
    "wheel_start_confirm_samples": WHEEL_START_CONFIRM_SAMPLES,
    "wheel_stall_threshold_rpm": WHEEL_STALL_THRESHOLD_RPM,
    "wheel_stall_confirm_samples": WHEEL_STALL_CONFIRM_SAMPLES,
    "wheel_start_timeout_ms": WHEEL_START_TIMEOUT_MS,
    "wheel_overspeed_rpm": WHEEL_OVERSPEED_RPM,
    "speed_feedback_timeout_ms": SPEED_FEEDBACK_TIMEOUT_MS,
    "control_interval_ms": CONTROL_INTERVAL_MS,
    "motor_dirs": MOTOR_DIRS,
    "motor_gains": MOTOR_GAINS,
}

# =========================================================================
# Chassis Core (C++ pybind11 engine) — full parameter dictionary.
#
# Keys match chassis_core::configFromDict in src/chassis_core.  Values are the
# colleague's validated config.xml on this robot, transcribed 2026-08-04.
# The chassis_driver Python shell reads these as ROS2 params and passes the
# dict to ChassisCore(config_dict).  Change params here OR in
# phase1_params.yaml — the YAML is loaded last and wins.
#
# NOTE on motor_dirs: the colleague's program drives this robot with
# FL=1 RL=1 FR=-1 RR=-1 (right-side motors reverse-mounted).  The intent
# semantics follow the colleague's convention: throttle=+1 is the direction
# the colleague's SBUS-forward produced.  If a deployment test shows the
# robot reversed, flip this array (one param, no code change).
# =========================================================================

CHASSIS_CORE_DEFAULTS = {
    # Motor hardware
    "pole_pairs": 10,
    "gear_ratio": 5.2,
    "wheel_radius": 0.10,
    "track_width": 0.45,
    "motor_ids": [2, 1, 4, 3],      # FL, RL, FR, RR
    "motor_dirs": [-1, -1, 1, 1],   # flipped 2026-08-04 — colleague's [1,1,-1,-1] tested REVERSED; matches Python-port MOTOR_DIRS
    "mower_id": 5,

    # Direction sign (formal config.xml §2 Motion_Forward_RPM_Sign)
    "motion_forward_rpm_sign": -1,

    # Base speed
    "run_rpm": 100,
    "rotate_rpm": 50,
    "turn_max_rpm": 0,

    # Steering model
    "turn_inner_ratio": -0.20,
    "arc_turn_blend_ms": 700,

    # In-place rotation
    "rotate_in_place_enable": True,
    "rotate_direction_sign": 1,
    "rotate_factory_current_control_enable": True,
    "rotate_neutral_hold_ms": 150,
    "rotate_speed_guard_enable": True,
    "rotate_start_max_rpm": 25.0,
    "rotate_start_guard_timeout_ms": 1200,
    "rotate_max_current": 500,      # 10mA units → 5.00A in-place torque

    # Speed ramp
    "speed_accel_rpm_s": 10,
    "speed_decel_rpm_s": 30,

    # Wheel speed-loop PI (current mode — RC/manual)
    "wheel_control_mode": "current",
    "wheel_speed_kp": 0.60,
    "wheel_speed_ki": 0.05,
    "wheel_feedforward_current": 24,
    "wheel_max_current": 35,
    "wheel_integral_max_current": 10,

    # Startup current ramp
    "wheel_start_initial_current": 55,
    "wheel_start_step_current": 2,
    "wheel_start_step_ms": 200,
    "wheel_start_max_current": 70,
    "wheel_start_threshold_rpm": 2,
    "wheel_start_confirm_samples": 1,
    "wheel_stall_threshold_rpm": 1,
    "wheel_stall_confirm_samples": 3,
    "wheel_start_timeout_ms": 7000,
    "wheel_overspeed_rpm": 120,
    "speed_feedback_timeout_ms": 1000,

    # ---- Post-start anti-stutter (formal config.xml ROS2 section) ----
    "wheel_post_start_hold_ms": 500,
    "wheel_post_start_current": 48,
    "wheel_min_run_current": 42,

    # ---- ROS2 autonomous-mode speed loop (formal config.xml §13) ----
    "ros2_enable": True,
    "ros2_cmd_timeout_ms": 300,
    "ros2_publish_tf": True,
    "ros2_odom_frame": "odom",
    "ros2_base_frame": "base_link",
    "ros2_wheel_control_mode": "current",
    "ros2_wheel_speed_kp": 0.80,
    "ros2_wheel_speed_ki": 0.010,
    "ros2_wheel_feedforward_current": 30,
    "ros2_wheel_max_current": 130,
    "ros2_wheel_integral_max_current": 10,
    "ros2_wheel_start_initial_current": 130,
    "ros2_wheel_start_max_current": 170,

    # ---- Velocity / acceleration limits (formal config.xml §8) ----
    "max_velocity_mps": 0.0,
    "max_angular_radps": 0.0,
    "max_accel_mps2": 0.40,
    "max_decel_mps2": 0.60,
    "max_angular_accel_radps2": 1.00,

    # Mower
    "cut_current": 0,

    # RC channel thresholds (SBUS raw values)
    "low_threshold": 800,
    "high_threshold": 1199,
    "throttle_channel": 3,
    "steering_channel": 4,
    "mower_channel": 2,
    "gear_channel": 6,

    # Failsafe / timing
    "failsafe_ms": 300,
    "heartbeat_ms": 200,
    "control_ms": 50,
    "speed_query_ms": 50,

    # Gear profiles (SBUS gear switch)
    "gear_select_enable": True,
    "gear_low_run_rpm": 70,
    "gear_low_turn_max_rpm": 70,
    "gear_mid_run_rpm": 120,
    "gear_mid_turn_max_rpm": 100,
    "gear_high_run_rpm": 140,
    "gear_high_turn_max_rpm": 100,

    # App control (multi-source; the ROS2 shell also exposes /chassis/intent)
    "app_control_enable": True,
    "app_udp_port": 8091,
    "app_timeout_ms": 300,

    # Logging
    "show_log": False,

    # IMU heading hold
    "imu_heading_enable": True,
    "imu_yaw_sign": -1.0,
    "imu_calibrate_ms": 3000,
    "imu_deadband_dps": 0.30,
    "heading_kp": 1.2,
    "heading_ki": 0.0,
    "heading_kd": 0.2,
    "heading_integral_max_correction": 0,
    "heading_max_correction": 5,
    "heading_correction_sign": -1,
    "heading_forward_trim": 0,
    "heading_reverse_trim": 0,
    "heading_reset_steering_hold_ms": 500,

    # Direction-change hold (brake before reversing)
    "direction_change_hold_enable": True,
    "direction_change_stop_rpm": 5.0,
    "direction_change_hold_timeout_ms": 1500,
    "direction_change_post_hold_ms": 1000,
    "direction_change_post_max_diff_rpm": 4,

    # Lateral hold (off by default)
    "lateral_hold_enable": False,
    "lateral_kp_deg_per_m": 0.0,
    "lateral_max_heading_deg": 0.0,
}

# =========================================================================
# Safety
# =========================================================================

MOTOR_TEMP_WARN_C    = 80.0   # Overheat warning
MOTOR_TEMP_CUTOFF_C  = 100.0  # Emergency thermal cutoff
ESTOP_DEBOUNCE_MS    = 200    # ESTOP button debounce
JOY_AXIS_DEADZONE    = 0.05   # Joystick neutral dead zone

# =========================================================================
# Topic Names — single registry for all inter-node communication
# =========================================================================

TOPIC_CMD_VEL        = "/cmd_vel"
TOPIC_ODOM           = "/odom"
TOPIC_JOY            = "/joy"
TOPIC_IMU_DATA       = "/imu/data"
TOPIC_FIX            = "/fix"
TOPIC_CHASSIS_STATUS = "/chassis/status"
TOPIC_MASTER_STATE   = "/master/state"
TOPIC_CHASSIS_INTENT = "/chassis/intent"   # std_msgs/String JSON — primary chassis control interface

# TF frames
FRAME_ODOM           = "odom"
FRAME_BASE_LINK      = "base_link"
FRAME_IMU_LINK       = "imu_link"
FRAME_GPS_LINK       = "gps_link"

# =========================================================================
# SBUS / C7mini Remote Control
# =========================================================================

# C7mini outputs 1000-2000 (center=1500). port_bridge may rescale.
# Auto-calibration in remote_driver handles actual values.
SBUS_NUM_CHANNELS    = 16
SBUS_FRAME_SIZE      = 25
SBUS_HEADER          = 0x0F
SBUS_FOOTER          = 0x00
SBUS_CHANNEL_BITS    = 11

# port_bridge remaps C7mini (1000-2000) → standard SBUS (172-1811).
# Auto-calibration in remote_driver determines actual per-channel centers.
SBUS_MIN             = 172
SBUS_CENTER          = 992
SBUS_MAX             = 1811

# SBUS flag byte (byte 23)
SBUS_FLAG_CH17       = 0x01
SBUS_FLAG_CH18       = 0x02
SBUS_FLAG_FAILSAFE   = 0x04
SBUS_FLAG_FRAME_LOST = 0x08

# SBUS serial — direct connect (bypass MCU)
# C7mini receiver → transistor inverter → /dev/ttyS7 (S6 未使能)
SBUS_PORT_DIRECT     = "/dev/ttyS7"
SBUS_BAUD_DIRECT     = 100000   # Standard SBUS rate (8E2)
# SBUS serial — MCU fallback (via port_bridge → RemoteControl_Test)
SBUS_PORT_MCU        = "/dev/ttyS1"
SBUS_BAUD_MCU        = 115200   # MCU bridge rate (8N1)
SBUS_FLAG_FRAME_LOST = 0x08

# =========================================================================
# RC Channel Map (C7mini, 8-channel)
#
#   CH1 = right stick L/R       CH5 = (unused)
#   CH2 = right stick F/B       CH6 = SWA (3-pos, START/STOP)
#   CH3 = left stick F/B        CH7 = SWB (3-pos, ESTOP)
#   CH4 = left stick L/R        CH8 = VR1, CH9 = VR2
#
#   SWC (hardware): limits L/R stick travel. Not an SBUS channel.
#   SWD (hardware): momentary, toggles RF indicator. Not an SBUS channel.
# =========================================================================

RC_CH_STEERING       = 0     # CH1: right stick L/R
RC_CH_THROTTLE       = 2     # CH3: left stick F/B (left-hand throttle)
RC_CH_SWA            = 5     # CH6: SWA 3-position switch → START/STOP
RC_CH_SWB            = 6     # CH7: SWB 3-position switch → ESTOP

# =========================================================================
# IMU — QMI8658 calibration coefficients (vendor datasheet)
# =========================================================================

IMU_CALIBRATION = {
    "accel_bias_x":  0.320,
    "accel_bias_y":  0.027,
    "accel_bias_z":  0.802,
    "accel_scale_z": 0.991,
    "gyro_bias_x":   0.0265,
    "gyro_bias_y":  -0.0135,
    "gyro_bias_z":   0.0132,
}
IMU_RAW_SCALE        = 1e6     # LSB per m/s² (accel) or rad/s (gyro)
IMU_STRUCT_FORMAT     = "<iiiiii"  # 6 × int32 LE
IMU_STRUCT_SIZE       = 24

# RTK simulation
RTK_SIM_LAT          = 31.2304
RTK_SIM_LON          = 121.4737
