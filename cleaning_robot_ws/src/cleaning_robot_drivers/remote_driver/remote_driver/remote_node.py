#!/usr/bin/env python3
"""
remote_driver — SBUS RC receiver driver.

Reads SBUS frames from /dev/ttyS1, decodes 16 channels (11-bit each),
publishes sensor_msgs/Joy to /joy.

SBUS frame (25 bytes):
  Byte 0:   0x0F header
  Byte 1-22: 16 channels × 11 bits = 176 bits
  Byte 23:  flags (bit2=failsafe, bit3=frame_lost)
  Byte 24:  0x00 footer

Channel range: 172 (min) to 1811 (max), center = 992.
Normalized to Joy.axes: [-1.0, +1.0]
Buttons: channel > 1200 → pressed

Sim mode: simulate:=true — neutral joy, no serial read.
"""

import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# SBUS constants
SBUS_FRAME_SIZE = 25
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00
SBUS_NUM_CHANNELS = 16
SBUS_CHANNEL_BITS = 11
SBUS_MIN = 172
SBUS_CENTER = 992
SBUS_MAX = 1811
SBUS_RANGE = SBUS_MAX - SBUS_MIN


def decode_sbus(data: bytes) -> Optional[Tuple[List[int], bool, bool]]:
    """Decode a 25-byte SBUS frame.

    Returns (channels[16], failsafe, frame_lost) or None if invalid.
    """
    if len(data) != SBUS_FRAME_SIZE:
        return None
    if data[0] != SBUS_HEADER or data[24] != SBUS_FOOTER:
        return None

    channels = [0] * SBUS_NUM_CHANNELS
    bit_pos = 0
    byte_idx = 1
    current_byte = data[byte_idx]

    for ch in range(SBUS_NUM_CHANNELS):
        value = 0
        bits_read = 0
        while bits_read < SBUS_CHANNEL_BITS:
            if bit_pos >= 8:
                byte_idx += 1
                if byte_idx >= 23:
                    break
                current_byte = data[byte_idx]
                bit_pos = 0
            n = min(SBUS_CHANNEL_BITS - bits_read, 8 - bit_pos)
            mask = (1 << n) - 1
            value |= ((current_byte >> bit_pos) & mask) << bits_read
            bit_pos += n
            bits_read += n
        channels[ch] = value

    flags = data[23]
    failsafe = bool(flags & 0x04)
    frame_lost = bool(flags & 0x08)

    return channels, failsafe, frame_lost


def sbus_to_axis(raw: int) -> float:
    """Normalize SBUS channel value to [-1.0, +1.0]."""
    if raw < SBUS_CENTER:
        return -float(SBUS_CENTER - raw) / (SBUS_CENTER - SBUS_MIN)
    else:
        return float(raw - SBUS_CENTER) / (SBUS_MAX - SBUS_CENTER)


def sbus_to_button(raw: int) -> int:
    """Convert SBUS channel to binary button (1=pressed above 75% travel)."""
    return 1 if raw > (SBUS_CENTER + SBUS_RANGE // 4) else 0


class RemoteDriverNode(Node):
    """ROS2 node: SBUS RC receiver → Joy publisher."""

    def __init__(self) -> None:
        super().__init__("remote_driver")

        self.declare_parameter("device", "/dev/ttyS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "rc_link")
        self.declare_parameter("sbus_min", SBUS_MIN)
        self.declare_parameter("sbus_center", SBUS_CENTER)
        self.declare_parameter("sbus_max", SBUS_MAX)
        self.declare_parameter("axis_deadzone", 0.05)
        self.declare_parameter("simulate", False)

        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._deadzone = self.get_parameter("axis_deadzone").get_parameter_value().double_value
        self._simulate = self.get_parameter("simulate").get_parameter_value().bool_value

        self._pub = self.create_publisher(Joy, "/joy", 10)
        self._running = False
        self._thread: Optional[threading.Thread] = None

        if self._simulate:
            self.get_logger().info("[SIM] remote_driver running in simulated mode")
            self._sim_timer = self.create_timer(0.05, self._publish_sim)
        else:
            if not HAS_SERIAL:
                self.get_logger().error("pyserial not installed. Falling back to sim mode.")
                self._sim_timer = self.create_timer(0.05, self._publish_sim)
                return
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

    def _read_loop(self) -> None:
        try:
            ser = serial.Serial(self._device, self._baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open {self._device}: {e}")
            return

        self.get_logger().info(f"remote_driver: reading SBUS from {self._device}")
        buffer = bytearray()

        while self._running and rclpy.ok():
            try:
                chunk = ser.read(ser.in_waiting or 1)
                buffer.extend(chunk)
            except serial.SerialException:
                break

            # Search for valid SBUS frame
            while len(buffer) >= SBUS_FRAME_SIZE:
                if buffer[0] != SBUS_HEADER:
                    buffer.pop(0)
                    continue
                if len(buffer) >= SBUS_FRAME_SIZE and buffer[SBUS_FRAME_SIZE - 1] == SBUS_FOOTER:
                    frame = bytes(buffer[:SBUS_FRAME_SIZE])
                    buffer = buffer[SBUS_FRAME_SIZE:]
                    result = decode_sbus(frame)
                    if result:
                        self._publish_joy(*result)
                    break
                else:
                    buffer.pop(0)

        ser.close()

    def _publish_joy(self, channels: List[int], failsafe: bool, frame_lost: bool) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Failsafe / lost frame → force zero axes + estop
        if failsafe or frame_lost:
            msg.axes = [0.0] * 4
            msg.buttons = [0, 0, 1, 0]  # ESTOP active
            self.get_logger().error("SBUS failsafe — emergency stop!")
        else:
            # Normalize axes (deadzone applied)
            axes = []
            for ch_raw in channels[:4]:
                ax = sbus_to_axis(ch_raw)
                if abs(ax) < self._deadzone:
                    ax = 0.0
                axes.append(ax)

            # Ensure at least 4 axes for Joy message compatibility
            while len(axes) < 4:
                axes.append(0.0)
            msg.axes = axes

            # Buttons
            buttons = [sbus_to_button(channels[ch]) for ch in range(4, min(8, len(channels)))]
            while len(buttons) < 4:
                buttons.append(0)
            msg.buttons = buttons

        self._pub.publish(msg)

    def _publish_sim(self) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.axes = [0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0, 0, 0, 0]
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
