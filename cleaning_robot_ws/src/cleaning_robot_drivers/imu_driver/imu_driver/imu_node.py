#!/usr/bin/env python3
"""
QMI8658 6-axis IMU driver node.

Reads raw accelerometer and gyroscope data from /dev/qmi8658_imu,
applies vendor calibration coefficients, and publishes sensor_msgs/Imu
messages at 200 Hz.

Simulation mode (simulate:=true) publishes zero linear acceleration
and zero angular velocity with gravity vector only.
"""

import os
import struct
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ---------------------------------------------------------------------------
# Vendor calibration offsets (per QMI8658 manual)
# ---------------------------------------------------------------------------
CALIBRATION = {
    'ax_offset': 0.320,      # ax = ax_raw / 1e6 + 0.320
    'ay_offset': 0.027,      # ay = ay_raw / 1e6 + 0.027
    'az_offset': 0.802,      # az = (az_raw / 1e6 + 0.802) * 0.991
    'az_scale':  0.991,
    'gx_offset': 0.0265,     # gx = gx_raw / 1e6 + 0.0265
    'gy_offset': -0.0135,    # gy = gy_raw / 1e6 - 0.0135
    'gz_offset': 0.0132,     # gz = gz_raw / 1e6 + 0.0132
}

# Raw value scale factor: LSB per m/s^2 (accel) and rad/s (gyro)
RAW_SCALE = 1e6

# struct format: 6 x int32 little-endian = 24 bytes
STRUCT_FORMAT = '<iiiiii'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)  # 24 bytes

# Gravity constant (m/s^2)
GRAVITY = 9.80665


class IMUDriver(Node):
    """QMI8658 6-axis IMU driver node."""

    def __init__(self):
        super().__init__('imu_node')

        # --- Declare parameters ---
        self.declare_parameter('device', '/dev/qmi8658_imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 200)
        self.declare_parameter('simulate', False)

        # Calibration coefficients (overridable via ROS params)
        self.declare_parameter('ax_offset', CALIBRATION['ax_offset'])
        self.declare_parameter('ay_offset', CALIBRATION['ay_offset'])
        self.declare_parameter('az_offset', CALIBRATION['az_offset'])
        self.declare_parameter('az_scale', CALIBRATION['az_scale'])
        self.declare_parameter('gx_offset', CALIBRATION['gx_offset'])
        self.declare_parameter('gy_offset', CALIBRATION['gy_offset'])
        self.declare_parameter('gz_offset', CALIBRATION['gz_offset'])

        # --- Cache parameters ---
        self._device = self.get_parameter('device').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._simulate = self.get_parameter('simulate').value

        self._calib = {
            'ax_offset': self.get_parameter('ax_offset').value,
            'ay_offset': self.get_parameter('ay_offset').value,
            'az_offset': self.get_parameter('az_offset').value,
            'az_scale': self.get_parameter('az_scale').value,
            'gx_offset': self.get_parameter('gx_offset').value,
            'gy_offset': self.get_parameter('gy_offset').value,
            'gz_offset': self.get_parameter('gz_offset').value,
        }

        # --- Publisher ---
        self._publisher = self.create_publisher(Imu, '/imu/data', 10)

        # --- Timer (fixed-rate publish loop) ---
        period = 1.0 / self._publish_rate
        self._timer = self.create_timer(period, self._timer_callback)

        # --- Device handle (lazy-open in simulation mode) ---
        self._fd = None
        if not self._simulate:
            self._open_device()

        self.get_logger().info(
            'IMU driver started: device=%s rate=%d Hz simulate=%s',
            self._device, self._publish_rate, str(self._simulate))

    def _open_device(self):
        """Open the IMU device file for blocking reads."""
        try:
            self._fd = os.open(self._device, os.O_RDONLY)
            self.get_logger().info('Opened IMU device: %s', self._device)
        except OSError as e:
            self.get_logger().error(
                'Cannot open IMU device %s: %s', self._device, e)
            self._fd = None

    def _read_raw(self):
        """Read one 24-byte raw frame from the device.

        Returns:
            tuple(ax, ay, az, gx, gy, gz) of raw int32 values, or None on error.
        """
        try:
            data = os.read(self._fd, STRUCT_SIZE)
            if len(data) != STRUCT_SIZE:
                self.get_logger().warn(
                    'Short read: expected %d, got %d bytes', STRUCT_SIZE, len(data))
                return None
            return struct.unpack(STRUCT_FORMAT, data)
        except OSError as e:
            self.get_logger().error('Read error on %s: %s', self._device, e)
            return None

    def _apply_calibration(self, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw):
        """Apply vendor calibration to raw sensor values.

        Returns:
            tuple(ax, ay, az, gx, gy, gz) in m/s^2 and rad/s.
        """
        ax = ax_raw / RAW_SCALE + self._calib['ax_offset']
        ay = ay_raw / RAW_SCALE + self._calib['ay_offset']
        az = (az_raw / RAW_SCALE + self._calib['az_offset']) * self._calib['az_scale']
        gx = gx_raw / RAW_SCALE + self._calib['gx_offset']
        gy = gy_raw / RAW_SCALE + self._calib['gy_offset']
        gz = gz_raw / RAW_SCALE + self._calib['gz_offset']
        return (ax, ay, az, gx, gy, gz)

    def _build_imu_msg(self, ax, ay, az, gx, gy, gz):
        """Build a sensor_msgs/Imu message from calibrated values."""
        now = self.get_clock().now().to_msg()

        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id

        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Angular velocity (rad/s)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Orientation: not provided by this IMU; covariance set to -1
        msg.orientation_covariance[0] = -1.0

        # Linear acceleration covariance (moderate confidence)
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        # Angular velocity covariance (moderate confidence)
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        return msg

    def _simulate_reading(self):
        """Return simulated IMU values: zero acceleration in x/y, gravity in z,
        zero angular velocity."""
        return (0.0, 0.0, GRAVITY, 0.0, 0.0, 0.0)

    def _timer_callback(self):
        """Timer callback: read (or simulate), calibrate, publish."""
        if self._simulate:
            ax, ay, az, gx, gy, gz = self._simulate_reading()
        else:
            if self._fd is None:
                return  # device not open; nothing to publish
            raw = self._read_raw()
            if raw is None:
                return
            ax, ay, az, gx, gy, gz = self._apply_calibration(*raw)

        msg = self._build_imu_msg(ax, ay, az, gx, gy, gz)
        self._publisher.publish(msg)

    def destroy_node(self):
        """Clean up device handle on shutdown."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
