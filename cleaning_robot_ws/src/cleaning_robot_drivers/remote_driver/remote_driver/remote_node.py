#!/usr/bin/env python3
"""
remote_driver — SBUS RC receiver → /joy publisher.

Port_bridge init (115200 → TCSETS2 trigger) + EXACT sbus_parse from vendor source.
"""

import fcntl
import os
import struct
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# ---- port_bridge init (RemoteControl_Test::configureSerial disassembly) ----
TCGETS2 = 0x802c542a
TCSETS2 = 0x402c542b
NCCS = 19

# ---- SBUS Constants ----
SBUS_FRAME_SIZE   = 25
SBUS_HEADER       = 0x0F
SBUS_NUM_CHANNELS = 16
CALIB_SAMPLES = 100


def init_port_bridge(device: str) -> Optional[int]:
    """Open /dev/ttyS1 and trigger port_bridge SBUS mode (Reverse-engineered).

    Exact replica of RemoteControl_Test::configureSerial ioctl sequence:
    1. Open at 115200
    2. Apply c_cflag changes via TCGETS2/TCSETS2
    3. Set baudrate to 100000 (this triggers port_bridge SBUS pass-through)
    """
    try:
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
    except OSError:
        return None

    buf = bytearray(72)
    try:
        fcntl.ioctl(fd, TCGETS2, buf)
    except OSError:
        os.close(fd)
        return None

    cflag = struct.unpack_from('I', buf, 8)[0]
    cflag &= 0xffffffcf      # ~CSIZE
    cflag |= 0x30            # CS8
    cflag |= 0x100           # CLOCAL
    cflag &= ~0x200          # ~CREAD
    cflag |= 0x40
    cflag |= 0x880
    struct.pack_into('I', buf, 8, cflag)
    buf[17 + 5] = 0  # VMIN
    buf[17 + 6] = 0  # VTIME

    cflag2 = struct.unpack_from('I', buf, 8)[0]
    cflag2 &= 0xffffeff0
    cflag2 |= 0x1000
    struct.pack_into('I', buf, 8, cflag2)

    for off in (36, 40, 40, 44):
        struct.pack_into('I', buf, off, 100000)

    try:
        fcntl.ioctl(fd, TCSETS2, buf)
    except OSError:
        os.close(fd)
        return None
    return fd


# =========================================================================
# sbus_parse — EXACT replica of vendor SBUSReader::sbus_parse()
# =========================================================================

def sbus_parse(buf: bytes) -> Optional[Tuple[List[int], int, int, int, int]]:
    if buf is None or len(buf) < 25:
        return None
    if buf[0] != 0x0F:
        return None

    ch = [0] * 16
    for i in range(16):
        byte_idx = 1 + (i * 11) // 8
        bit_off  = (i * 11) % 8
        raw = (buf[byte_idx] |
               (buf[byte_idx + 1] << 8) |
               (buf[byte_idx + 2] << 16))
        ch[i] = (raw >> bit_off) & 0x07FF

    flags = buf[23]
    ch17 = flags & 0x01
    ch18 = (flags >> 1) & 0x01
    frame_lost = (flags >> 2) & 0x01
    failsafe   = (flags >> 3) & 0x01
    return ch, ch17, ch18, frame_lost, failsafe


def sbus_to_axis(raw: int, center: float, deadzone: float = 0.05) -> float:
    mn, mx = 172.0, 1811.0
    if raw < center:
        ax = -float(center - raw) / max(center - mn, 1.0)
    else:
        ax = float(raw - center) / max(mx - center, 1.0)
    return 0.0 if abs(ax) < deadzone else ax


class RemoteDriverNode(Node):
    def __init__(self):
        super().__init__("remote_driver")
        self.declare_parameter("device", "/dev/ttyS1")
        self.declare_parameter("frame_id", "rc_link")
        self.declare_parameter("axis_deadzone", 0.05)
        self.declare_parameter("ch_steering",   0)  # CH1
        self.declare_parameter("ch_throttle",   2)  # CH3
        self.declare_parameter("ch_btn_start",  4)   # CH5 SWA — 3-pos: 200=MANUAL, 1000=STANDBY, 1800=ESTOP
        self.declare_parameter("ch_btn_estop",  5)   # CH6 SWB — 3-pos: gear select LOW/MID/HIGH
        self.declare_parameter("steering_invert", True)
        self.declare_parameter("throttle_invert", False)  # forward=high→positive
        # CH1 steering travel is small (~0.33 full throw) on this C7mini —
        # scale it up so a full stick deflection = full steering authority.
        self.declare_parameter("steering_gain", 3.0)

        g = lambda n: self.get_parameter(n).get_parameter_value()
        self._device   = g("device").string_value
        self._frame_id = g("frame_id").string_value
        self._deadzone = g("axis_deadzone").double_value
        self._ch_steer = g("ch_steering").integer_value
        self._ch_throt = g("ch_throttle").integer_value
        self._ch_start = g("ch_btn_start").integer_value
        self._ch_estop = g("ch_btn_estop").integer_value
        self._steer_inv = g("steering_invert").bool_value
        self._throt_inv = g("throttle_invert").bool_value
        self._steer_gain = g("steering_gain").double_value

        self._pub = self.create_publisher(Joy, "/joy", 10)
        self._running = True
        self._fd = None
        self._thread = None

        self._calib_count = 0
        self._calib_sum = [0.0] * 16
        self._centers = [992.0] * 16
        self._calibrated = False
        self._swa_threshold = 1400.0  # SWA high=1800 → MANUAL
        self._prev_ch = [0] * 16

        self._frame_count = 0
        self._last_stats = time.time()

        self._start()

    def _start(self):
        self.get_logger().info(f"Opening {self._device}...")
        self._fd = init_port_bridge(self._device)
        if self._fd is None:
            self.get_logger().error(f"Failed to init {self._device}")
            return
        self.get_logger().info("port_bridge initialized — reading SBUS")
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # ---- readFrame — EXACT replica of SBUSReader::readFrame() ----

    def _read_loop(self):
        """Read SBUS frames with reliable byte-by-byte sync.

        port_bridge output: 0x0F header at variable intervals (not fixed 25-byte
        stride). Strategy: sync on 0x0F (rejecting 0x0F in channel data by
        validating that the next frame is valid SBUS), collect 25 bytes, parse.

        Performance: NO per-change logging (was CPU bottleneck at 70fps).
        Only 10s summary logs.
        """
        synced = False
        rx_buffer = bytearray(25)
        rx_index = 0

        while self._running and rclpy.ok() and self._fd is not None:
            try:
                chunk = os.read(self._fd, 256)
                if not chunk:
                    time.sleep(0.001)
                    continue
            except (OSError, BlockingIOError):
                time.sleep(0.001)
                continue

            for b in chunk:
                if not synced:
                    if b == 0x0F:
                        synced = True
                        rx_index = 0
                        rx_buffer[rx_index] = b
                        rx_index += 1
                    continue

                rx_buffer[rx_index] = b
                rx_index += 1

                if rx_index >= 25:
                    synced = False
                    rx_index = 0
                    frame = bytes(rx_buffer)
                    result = sbus_parse(frame)
                    if result:
                        ch, ch17, ch18, frame_lost, failsafe = result
                        # Reject false syncs: valid SBUS channels are 100-2000
                        if 100 <= ch[0] <= 2000 and 100 <= ch[1] <= 2000:
                            self._publish_joy(ch, frame_lost, failsafe)

    def _publish_joy(self, channels, frame_lost, failsafe):
        now = time.time()
        if not self._calibrated:
            for i in range(16):
                self._calib_sum[i] += channels[i]
            self._calib_count += 1
            if self._calib_count >= CALIB_SAMPLES:
                for i in range(16):
                    self._centers[i] = self._calib_sum[i] / self._calib_count
                self._calibrated = True
                self.get_logger().info(
                    f"Calibrated: CH1={self._centers[0]:.0f} CH3={self._centers[2]:.0f} | "
                    f"CH6(SWA)={self._centers[5]:.0f} CH7(SWB)={self._centers[6]:.0f}"
                )
            return

        self._frame_count += 1
        if now - self._last_stats > 10.0:
            self.get_logger().info(
                f"SBUS: {self._frame_count}f/10s | "
                f"CH1={channels[0]:4d} CH2={channels[1]:4d} CH3={channels[2]:4d} CH4={channels[3]:4d} | "
                f"CH5={channels[4]:4d} CH6={channels[5]:4d} CH7={channels[6]:4d} CH8={channels[7]:4d}"
            )
            self._frame_count = 0
            self._last_stats = now

        # NOTE: per-change logging disabled — ROS2 logger is synchronous
        # and at 70fps×8 channels it kills frame rate. Use 10s summary only.

        steer = sbus_to_axis(channels[self._ch_steer], self._centers[self._ch_steer], self._deadzone)
        throt = sbus_to_axis(channels[self._ch_throt], self._centers[self._ch_throt], self._deadzone)
        if self._steer_inv: steer = -steer
        if self._throt_inv: throt = -throt
        # Scale up the small CH1 travel so full stick = full steering authority.
        steer = max(-1.0, min(1.0, steer * self._steer_gain))
        # SWA(CH5) 3-pos: 200=UP(MANUAL), 1000=MID(STANDBY), 1800=DOWN(ESTOP)
        swa = channels[self._ch_start]
        btn_manual = 1 if swa < 700 else 0       # up → MANUAL
        btn_estop  = 1 if swa > 1400 else 0      # down → ESTOP (merged onto SWA)
        # SWB(CH6) 3-pos: gear select → axes[2] as -1/0/+1
        swb = channels[self._ch_estop]
        if swb > 1400:    gear_axis = 1.0   # HIGH
        elif swb < 700:   gear_axis = -1.0  # LOW
        else:             gear_axis = 0.0   # MID (default)

        m = Joy()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._frame_id
        m.axes = [steer, throt, gear_axis, 0.0]
        m.buttons = [btn_manual, 0, btn_estop, 0]
        self._pub.publish(m)

    def destroy_node(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._fd is not None:
            os.close(self._fd); self._fd = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RemoteDriverNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
