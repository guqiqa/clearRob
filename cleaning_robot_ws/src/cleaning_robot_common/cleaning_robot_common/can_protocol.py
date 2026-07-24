#!/usr/bin/env python3
"""
cleaning_robot_common/can_protocol.py — CAN frame builders and parsers.

Encapsulates OID FOC motor protocol: current control, heartbeat, speed queries.
"""

import struct
from typing import Optional

from .config import (
    CAN_CMD_CURRENT, CAN_CMD_HEARTBEAT, CAN_CMD_QUERY,
    CAN_QUERY_SPEED,
    QUERY_RESP_MIN_LEN, QUERY_RESP_ERPM_OFF,
)

# ---- Frame builders ----

def make_heartbeat_frame(can_id: int) -> tuple:
    """Build a heartbeat frame (DLC=1, byte0=0x00)."""
    return (can_id, bytes([CAN_CMD_HEARTBEAT]))


def make_current_frame(can_id: int, current: int) -> tuple:
    """Build a current-control frame (DLC=3, int16 LE).

    current: signed int16 in 10mA units. Clamped to [-32768, 32767].
    """
    current = max(-32768, min(32767, current))
    data = bytes([CAN_CMD_CURRENT, (current >> 8) & 0xFF, current & 0xFF])
    return (can_id, data)


def make_query_frame(can_id: int, query_code: int = CAN_QUERY_SPEED) -> tuple:
    """Build a query frame (DLC=2: [0x0F, query_code])."""
    return (can_id, bytes([CAN_CMD_QUERY, query_code]))


def is_query_response(data: bytes) -> bool:
    """Check if a received CAN frame is a query response."""
    return (len(data) >= QUERY_RESP_MIN_LEN and data[0] == CAN_CMD_QUERY)


def parse_query_erpm(data: bytes) -> Optional[int]:
    """Extract int32 erpm from a query-speed response frame."""
    if not is_query_response(data) or len(data) < QUERY_RESP_MIN_LEN:
        return None
    return struct.unpack("<i", data[QUERY_RESP_ERPM_OFF:QUERY_RESP_ERPM_OFF + 4])[0]


def parse_query_int16(data: bytes) -> Optional[int]:
    """Extract int16 value from a generic query response (e.g. temp/current)."""
    if not is_query_response(data) or len(data) < 4:
        return None
    return struct.unpack("<h", data[2:4])[0]
