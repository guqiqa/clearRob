#!/usr/bin/env python3
"""
rtk_driver — NMEA 0183 GNSS driver for NTRIP high-precision GNSS.

Reads from /dev/ttyS2 @ 460800 bps, parses $GPGGA and $GPRMC,
publishes sensor_msgs/NavSatFix to /fix.

Sim mode: simulate:=true — publishes fixed coordinates, no serial read.
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


def nmea_checksum(sentence: str) -> bool:
    """Validate NMEA checksum (*XX at end)."""
    if '*' not in sentence:
        return False
    payload, checksum = sentence[1:].split('*', 1)
    calc = 0
    for c in payload:
        calc ^= ord(c)
    try:
        return calc == int(checksum, 16)
    except ValueError:
        return False


def parse_ddmm_mmmm(raw: str, direction: str) -> float:
    """Convert NMEA ddmm.mmmm format to decimal degrees."""
    if not raw or not direction:
        return 0.0
    deg_len = 3 if direction in ('E', 'W') else 2
    deg = float(raw[:deg_len])
    minutes = float(raw[deg_len:])
    result = deg + minutes / 60.0
    if direction in ('S', 'W'):
        result = -result
    return result


class RtkDriverNode(Node):
    """ROS2 node: NMEA 0183 GNSS driver."""

    def __init__(self) -> None:
        super().__init__("rtk_driver")

        self.declare_parameter("device", "/dev/ttyS2")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("simulate", False)
        self.declare_parameter("sim_lat", 31.2304)
        self.declare_parameter("sim_lon", 121.4737)

        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._simulate = self.get_parameter("simulate").get_parameter_value().bool_value
        self._sim_lat = self.get_parameter("sim_lat").get_parameter_value().double_value
        self._sim_lon = self.get_parameter("sim_lon").get_parameter_value().double_value

        self._pub_fix = self.create_publisher(NavSatFix, "/fix", 10)
        self._running = False
        self._thread: Optional[threading.Thread] = None

        if self._simulate:
            self.get_logger().info(f"[SIM] rtk_driver: lat={self._sim_lat}, lon={self._sim_lon}")
            self._sim_timer = self.create_timer(1.0, self._publish_sim)
        else:
            if not HAS_SERIAL:
                self.get_logger().error("pyserial not installed. Falling back to sim mode.")
                self._simulate = True
                self._sim_timer = self.create_timer(1.0, self._publish_sim)
                return
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

    def _read_loop(self) -> None:
        try:
            ser = serial.Serial(self._device, self._baudrate, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open {self._device}: {e}")
            return

        self.get_logger().info(f"rtk_driver: reading from {self._device} @ {self._baudrate}")
        lat = lon = alt = 0.0
        quality = 0

        while self._running and rclpy.ok():
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                break

            if line.startswith('$GPGGA') and nmea_checksum(line):
                fields = line.split(',')
                if len(fields) >= 15:
                    lat2 = parse_ddmm_mmmm(fields[2], fields[3])
                    lon2 = parse_ddmm_mmmm(fields[4], fields[5])
                    if lat2 != 0.0:
                        lat, lon = lat2, lon2
                    try:
                        alt = float(fields[9]) if fields[9] else alt
                        quality = int(fields[6])
                    except (ValueError, IndexError):
                        pass
                    self._publish_fix(lat, lon, alt, quality)

            elif line.startswith('$GPRMC') and nmea_checksum(line):
                fields = line.split(',')
                if len(fields) >= 10 and fields[2] == 'A':  # Valid fix
                    lat2 = parse_ddmm_mmmm(fields[3], fields[4])
                    lon2 = parse_ddmm_mmmm(fields[5], fields[6])
                    if lat2 != 0.0:
                        lat, lon = lat2, lon2
                        self._publish_fix(lat, lon, alt, quality)

        ser.close()

    def _publish_fix(self, lat: float, lon: float, alt: float, quality: int) -> None:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt

        # Covariance based on fix quality
        if quality >= 4:
            cov = 0.01   # RTK fixed: ~1cm
        elif quality >= 2:
            cov = 1.0    # DGPS: ~1m
        else:
            cov = 25.0   # Autonomous: ~5m

        fix.position_covariance[0] = cov
        fix.position_covariance[4] = cov
        fix.position_covariance[8] = cov * 4.0  # Height typically 2x worse
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        fix.status.status = quality
        fix.status.service = NavSatStatus.SERVICE_GPS

        self._pub_fix.publish(fix)

    def _publish_sim(self) -> None:
        self._publish_fix(self._sim_lat, self._sim_lon, 10.0, 4)  # Sim RTK fixed

    def destroy_node(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
