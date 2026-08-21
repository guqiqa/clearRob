#!/usr/bin/env python3
"""
rtk_driver — NMEA 0183 GNSS driver for NTRIP high-precision GNSS.

Reads from /dev/ttyS2 @ 460800 bps, parses $GxGGA and $GxRMC
(multi-talker: $GPGGA/$GNGGA/$BDGGA etc.), publishes
sensor_msgs/NavSatFix to /fix.

Optional NTRIP client (ntrip_enable:=true): connects to an NTRIP caster
over the network (WiFi/4G), receives RTCM3 differential corrections and
injects them into the same serial line (module UART RX), enabling RTK
fixed solutions even when the module has no SIM card of its own.

Sim mode: simulate:=true — publishes fixed coordinates, no serial read.
"""

import base64
import math
import socket
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


def _checksum_hex(payload: str) -> str:
    """XOR checksum of a NMEA payload (chars between $ and *)."""
    calc = 0
    for c in payload:
        calc ^= ord(c)
    return f"{calc:02X}"


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


def to_ddmm_mmmm(dec: float, is_lat: bool) -> str:
    """Convert decimal degrees to NMEA ddmm.mmmm (lat 2-digit, lon 3-digit deg)."""
    neg = dec < 0
    dec = abs(dec)
    deg = int(dec)
    minutes = (dec - deg) * 60.0
    return f"{deg:0{2 if is_lat else 3}d}{minutes:07.4f}"


def build_gga(lat: float, lon: float, utc: str = "000000.00") -> bytes:
    """Build a minimal valid GGA sentence (for NTRIP/VRS positioning)."""
    lat_s = to_ddmm_mmmm(lat, True)
    lon_s = to_ddmm_mmmm(lon, False)
    lat_dir = "S" if lat < 0 else "N"
    lon_dir = "W" if lon < 0 else "E"
    payload = (f"GPGGA,{utc},{lat_s},{lat_dir},{lon_s},{lon_dir},"
               f"1,08,1.0,10.0,M,0.0,M,,")
    return f"${payload}*{_checksum_hex(payload)}\r\n".encode()


def build_ntrip_request(mount: str, host: str, port: int, user: str, pwd: str) -> bytes:
    """Build NTRIP 1.0 HTTP GET request with Basic auth."""
    if not mount.startswith("/"):
        mount = "/" + mount
    token = base64.b64encode(f"{user}:{pwd}".encode()).decode()
    return (f"GET {mount} HTTP/1.0\r\n"
            f"Host: {host}:{port}\r\n"
            f"User-Agent: rtk_driver/1.0\r\n"
            f"Authorization: Basic {token}\r\n"
            f"\r\n").encode()


class RtkDriverNode(Node):
    """ROS2 node: NMEA 0183 GNSS driver (optionally with NTRIP client)."""

    def __init__(self) -> None:
        super().__init__("rtk_driver")

        self.declare_parameter("device", "/dev/ttyS2")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("simulate", False)
        self.declare_parameter("sim_lat", 31.2304)
        self.declare_parameter("sim_lon", 121.4737)
        # NTRIP client params (WiFi/4G differential injection, no module SIM needed)
        self.declare_parameter("ntrip_enable", False)
        self.declare_parameter("ntrip_host", "203.107.45.154")
        self.declare_parameter("ntrip_port", 8002)
        self.declare_parameter("ntrip_mount", "AUTO")
        self.declare_parameter("ntrip_user", "")
        self.declare_parameter("ntrip_pwd", "")
        self.declare_parameter("ntrip_gga_interval", 10)
        self.declare_parameter("ntrip_fallback_lat", 31.2304)
        self.declare_parameter("ntrip_fallback_lon", 121.4737)

        self._device = self.get_parameter("device").get_parameter_value().string_value
        self._baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._simulate = self.get_parameter("simulate").get_parameter_value().bool_value
        self._sim_lat = self.get_parameter("sim_lat").get_parameter_value().double_value
        self._sim_lon = self.get_parameter("sim_lon").get_parameter_value().double_value
        self._ntrip_enable = self.get_parameter("ntrip_enable").get_parameter_value().bool_value
        self._ntrip_host = self.get_parameter("ntrip_host").get_parameter_value().string_value
        self._ntrip_port = self.get_parameter("ntrip_port").get_parameter_value().integer_value
        self._ntrip_mount = self.get_parameter("ntrip_mount").get_parameter_value().string_value
        self._ntrip_user = self.get_parameter("ntrip_user").get_parameter_value().string_value
        self._ntrip_pwd = self.get_parameter("ntrip_pwd").get_parameter_value().string_value
        self._ntrip_gga_interval = self.get_parameter("ntrip_gga_interval").get_parameter_value().integer_value
        self._ntrip_fallback_lat = self.get_parameter("ntrip_fallback_lat").get_parameter_value().double_value
        self._ntrip_fallback_lon = self.get_parameter("ntrip_fallback_lon").get_parameter_value().double_value

        self._pub_fix = self.create_publisher(NavSatFix, "/fix", 10)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._ntrip_thread: Optional[threading.Thread] = None
        self._ser: Optional[serial.Serial] = None
        self._last_lat: Optional[float] = None
        self._last_lon: Optional[float] = None
        self._last_gga_raw: Optional[str] = None

        if self._simulate:
            self.get_logger().info(f"[SIM] rtk_driver: lat={self._sim_lat}, lon={self._sim_lon}")
            self._sim_timer = self.create_timer(1.0, self._publish_sim)
        else:
            if not HAS_SERIAL:
                self.get_logger().error("pyserial not installed. Falling back to sim mode.")
                self._simulate = True
                self._sim_timer = self.create_timer(1.0, self._publish_sim)
                return
            try:
                self._ser = serial.Serial(self._device, self._baudrate, timeout=1.0)
            except serial.SerialException as e:
                self.get_logger().error(f"Cannot open {self._device}: {e}")
                return
            self.get_logger().info(f"rtk_driver: reading from {self._device} @ {self._baudrate}")
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            if self._ntrip_enable:
                self._ntrip_thread = threading.Thread(target=self._ntrip_loop, daemon=True)
                self._ntrip_thread.start()

    def _read_loop(self) -> None:
        lat = lon = alt = 0.0
        quality = 0
        while self._running and rclpy.ok():
            try:
                line = self._ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                break

            if line.startswith('$') and line[3:6] == 'GGA' and nmea_checksum(line):
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
                    if lat != 0.0 and lon != 0.0:
                        self._last_lat = lat
                        self._last_lon = lon
                        self._last_gga_raw = line
                    self._publish_fix(lat, lon, alt, quality)

            elif line.startswith('$') and line[3:6] == 'RMC' and nmea_checksum(line):
                fields = line.split(',')
                if len(fields) >= 10 and fields[2] == 'A':  # Valid fix
                    lat2 = parse_ddmm_mmmm(fields[3], fields[4])
                    lon2 = parse_ddmm_mmmm(fields[5], fields[6])
                    if lat2 != 0.0:
                        lat, lon = lat2, lon2
                        self._publish_fix(lat, lon, alt, quality)

    def _ntrip_loop(self) -> None:
        """NTRIP client: pull RTCM3 corrections over network, inject to module UART."""
        host, port = self._ntrip_host, self._ntrip_port
        request = build_ntrip_request(self._ntrip_mount, host, port,
                                      self._ntrip_user, self._ntrip_pwd)
        retry_delay = 5.0
        total = 0
        prev_8k = 0
        self.get_logger().info(
            f"NTRIP: client {host}:{port} mount={self._ntrip_mount} "
            f"(user={self._ntrip_user})")
        while self._running and rclpy.ok():
            sock = None
            try:
                sock = socket.create_connection((host, port), timeout=10.0)
                sock.settimeout(10.0)
                sock.sendall(request)
                header = b""
                while b"\r\n\r\n" not in header:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    header += chunk
                if b"200 OK" not in header:
                    self.get_logger().error(
                        f"NTRIP: caster rejected — {header[:120]!r}. "
                        "Check account/mountpoint.")
                    sock.close()
                    sock = None
                    time.sleep(retry_delay)
                    continue
                self.get_logger().info("NTRIP: connected — ICY 200 OK")
                rest = header.split(b"\r\n\r\n", 1)[1]
                if rest:
                    self._ser.write(rest)
                    total += len(rest)
                self._send_gga(sock)
                last_gga = time.monotonic()
                while self._running and rclpy.ok():
                    now = time.monotonic()
                    if (self._ntrip_gga_interval > 0
                            and now - last_gga >= self._ntrip_gga_interval):
                        self._send_gga(sock)
                        last_gga = now
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("caster closed connection")
                    self._ser.write(chunk)
                    total += len(chunk)
                    if total // 8192 != prev_8k:
                        prev_8k = total // 8192
                        self.get_logger().info(f"NTRIP: {total} RTCM bytes injected")
            except (socket.error, ConnectionError, serial.SerialException) as e:
                self.get_logger().warn(f"NTRIP: {e} — retrying in {retry_delay:.0f}s")
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            time.sleep(retry_delay)

    def _send_gga(self, sock: socket.socket) -> None:
        """Send the module's current position (or fallback) to the caster for VRS."""
        if self._last_gga_raw:
            gga = self._last_gga_raw.encode('ascii', errors='ignore') + b"\r\n"
        else:
            gga = build_gga(self._ntrip_fallback_lat, self._ntrip_fallback_lon)
        try:
            sock.sendall(gga)
        except socket.error as e:
            self.get_logger().warn(f"NTRIP: GGA send failed: {e}")

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
        if self._ntrip_thread and self._ntrip_thread.is_alive():
            self._ntrip_thread.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
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
