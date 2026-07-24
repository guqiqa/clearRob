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

CAN_ID_LEFT_FRONT  = 1    # 左前轮
CAN_ID_RIGHT_FRONT = 2    # 右前轮
CAN_ID_LEFT_REAR   = 3    # 左后轮
CAN_ID_RIGHT_REAR  = 4    # 右后轮
CAN_ID_CUTTER      = 5    # 刀盘 (Phase 2)

DRIVE_IDS = [CAN_ID_LEFT_FRONT, CAN_ID_RIGHT_FRONT,
             CAN_ID_LEFT_REAR, CAN_ID_RIGHT_REAR]
LEFT_IDS  = [CAN_ID_LEFT_FRONT, CAN_ID_LEFT_REAR]
RIGHT_IDS = [CAN_ID_RIGHT_FRONT, CAN_ID_RIGHT_REAR]

# =========================================================================
# CAN Protocol — Frame types
# =========================================================================

CAN_CMD_HEARTBEAT    = 0x00   # Keep-alive (DLC=1)
CAN_CMD_CURRENT      = 0x01   # Current control (DLC=3, int16 in 10mA units)
CAN_CMD_QUERY        = 0x0F   # Query command (DLC=2)
CAN_QUERY_SPEED      = 0x01   # Query sub-code: motor speed (erpm, int32)
CAN_QUERY_FAULT      = 0x00   # Query sub-code: fault info
CAN_QUERY_TEMP       = 0x07   # Query sub-code: temperature (°C)
CAN_QUERY_CURRENT    = 0x05   # Query sub-code: motor current (10mA)

# Query response layout
QUERY_RESP_MIN_LEN   = 6      # DLC >= 6 bytes
QUERY_RESP_ERPM_OFF  = 2      # erpm at DATA[2:6] as int32 LE
QUERY_RESP_CMD_BYTE  = 0      # DATA[0] should == 0x0F

# Current scaling
CURRENT_UNIT_MA      = 10     # CAN current: 1 unit = 10mA

# =========================================================================
# Motor Kinematics (confirmed hardware specs)
# =========================================================================

MOTOR_POLE_PAIRS     = 10     # 20-pole motor → 10 pole pairs
MOTOR_GEAR_RATIO     = 5.2    # 5.2:1 reduction gearbox
WHEEL_RADIUS_M       = 0.08   # 160 mm wheel diameter
TRACK_WIDTH_M        = 0.53   # 530 mm track width

# Velocity limits
MAX_LINEAR_VELOCITY  = 0.5    # m/s (estimated, needs field calibration)
MAX_ANGULAR_VELOCITY = 2.0    # rad/s
MAX_MOTOR_CURRENT    = 500    # ×10mA = 5.0A max
CURRENT_DEADZONE     = 10     # ×10mA = 0.1A dead zone

# =========================================================================
# Timing
# =========================================================================

CAN_QUERY_INTERVAL_MS    = 20    # Speed query period
CAN_HEARTBEAT_INTERVAL_MS = 10   # CAN heartbeat period
CMD_VEL_TIMEOUT_MS        = 500  # /cmd_vel timeout → emergency stop
CAN_QUERY_TIMEOUT_MS      = 100  # Query response timeout
ODOM_PUBLISH_HZ           = 50   # Odom / TF publish rate
IMU_PUBLISH_HZ            = 200  # IMU data rate

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

# TF frames
FRAME_ODOM           = "odom"
FRAME_BASE_LINK      = "base_link"
FRAME_IMU_LINK       = "imu_link"
FRAME_GPS_LINK       = "gps_link"

# =========================================================================
# SBUS — Remote Control
# =========================================================================

SBUS_NUM_CHANNELS    = 16
SBUS_FRAME_SIZE      = 25
SBUS_HEADER          = 0x0F
SBUS_FOOTER          = 0x00
SBUS_CHANNEL_BITS    = 11

SBUS_MIN             = 172
SBUS_CENTER          = 992
SBUS_MAX             = 1811

# SBUS flag byte (byte 23)
SBUS_FLAG_FAILSAFE   = 0x04
SBUS_FLAG_FRAME_LOST = 0x08

# =========================================================================
# RC Channel Map (vendor-specific, verify on real hardware)
# =========================================================================

RC_AXIS_STEERING     = 0    # CH1: right stick X → angular velocity
RC_AXIS_THROTTLE     = 1    # CH2: left stick Y → linear velocity
RC_BTN_START_STOP    = 0    # CH5: SWA → toggle MANUAL/STANDBY
RC_BTN_ESTOP         = 2    # CH7: SWC → emergency stop

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
