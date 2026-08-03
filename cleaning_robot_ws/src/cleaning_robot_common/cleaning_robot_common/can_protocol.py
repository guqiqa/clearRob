#!/usr/bin/env python3
"""
cleaning_robot_common/can_protocol.py — CAN frame builders and parsers.

Encapsulates OID FOC motor protocol: current control, speed control (driver
closed-loop), heartbeat, and speed queries.

Byte order note: the driver uses BIG-endian for all multi-byte payloads
(speed/accel/decel erpm int32, max-current uint16, query response erpm).
Verified against the WheelPID V2 control program running on the same motors.
"""

import struct
from typing import Optional

from .config import (
    CAN_CMD_CURRENT, CAN_CMD_HEARTBEAT, CAN_CMD_QUERY,
    CAN_CMD_SPEED, CAN_CMD_SET_ACCEL, CAN_CMD_SET_DECEL,
    CAN_CMD_SET_MAX_CURRENT,
    CAN_QUERY_SPEED,
    QUERY_RESP_MIN_LEN, QUERY_RESP_ERPM_OFF,
)

# ---- Frame builders ----

def make_heartbeat_frame(can_id: int) -> tuple:
    """Build a heartbeat frame (DLC=1, byte0=0x00)."""
    return (can_id, bytes([CAN_CMD_HEARTBEAT]))


def make_current_frame(can_id: int, current: int) -> tuple:
    """Build a current-control frame (DLC=3, int16 big-endian).

    current: signed int16 in 10mA units. Clamped to [-32768, 32767].
    """
    current = max(-32768, min(32767, current))
    data = bytes([CAN_CMD_CURRENT, (current >> 8) & 0xFF, current & 0xFF])
    return (can_id, data)


def make_speed_frame(can_id: int, erpm: int) -> tuple:
    """Build a speed-control frame (DLC=5, int32 big-endian).

    erpm: signed int32 electrical RPM target for the driver's internal
    closed-loop speed controller (OID cmd 0x02).
    """
    erpm = max(-2**31, min(2**31 - 1, int(erpm)))
    data = bytes([CAN_CMD_SPEED]) + struct.pack(">i", erpm)
    return (can_id, data)


def make_set_accel_frame(can_id: int, erpm_per_s: int) -> tuple:
    """Build a driver-side acceleration-ramp frame (DLC=5, int32 BE, cmd 0x0a)."""
    val = max(-2**31, min(2**31 - 1, int(erpm_per_s)))
    return (can_id, bytes([CAN_CMD_SET_ACCEL]) + struct.pack(">i", val))


def make_set_decel_frame(can_id: int, erpm_per_s: int) -> tuple:
    """Build a driver-side deceleration-ramp frame (DLC=5, int32 BE, cmd 0x10)."""
    val = max(-2**31, min(2**31 - 1, int(erpm_per_s)))
    return (can_id, bytes([CAN_CMD_SET_DECEL]) + struct.pack(">i", val))


def make_set_max_current_frame(can_id: int, current_10ma: int) -> tuple:
    """Build a driver closed-loop max-current frame (DLC=3, uint16 BE, cmd 0x14).

    current_10ma: unsigned uint16 in 10mA units, clamped to [0, 32767].
    """
    current_10ma = max(0, min(32767, int(current_10ma)))
    return (can_id, bytes([CAN_CMD_SET_MAX_CURRENT]) + struct.pack(">H", current_10ma))


def make_query_frame(can_id: int, query_code: int = CAN_QUERY_SPEED) -> tuple:
    """Build a query frame (DLC=2: [0x0F, query_code])."""
    return (can_id, bytes([CAN_CMD_QUERY, query_code]))


def is_query_response(data: bytes) -> bool:
    """Check if a received CAN frame is a query response."""
    return (len(data) >= QUERY_RESP_MIN_LEN and data[0] == CAN_CMD_QUERY)


def parse_query_erpm(data: bytes) -> Optional[int]:
    """Extract int32 erpm from a query-speed response frame.

    Big-endian — confirmed by the WheelPID V2 program running on the same
    OID FOC drivers. The old little-endian read produced garbage RPM values.
    """
    if not is_query_response(data) or len(data) < QUERY_RESP_MIN_LEN:
        return None
    return struct.unpack(">i", data[QUERY_RESP_ERPM_OFF:QUERY_RESP_ERPM_OFF + 4])[0]


def parse_query_int16(data: bytes) -> Optional[int]:
    """Extract int16 value from a generic query response (e.g. temp/current)."""
    if not is_query_response(data) or len(data) < 4:
        return None
    return struct.unpack("<h", data[2:4])[0]
