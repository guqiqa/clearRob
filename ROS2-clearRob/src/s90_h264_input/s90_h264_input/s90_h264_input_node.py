#!/usr/bin/env python3
"""Decode one S90 H264 HTTP stream and publish ROS2 images.

Run one process per eye. Keeping decoding in the process main loop avoids
blocking the ROS executor and makes reconnect behavior independent per stream.
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image


class S90H264InputNode(Node):
    def __init__(self) -> None:
        super().__init__("s90_h264_input")
        self.declare_parameter("url", "")
        self.declare_parameter("output_topic", "camera_native/left/image_raw")
        self.declare_parameter("frame_id", "stereo_left_camera_optical_frame")
        self.declare_parameter("reconnect_s", 1.0)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("output_width", 544)
        self.declare_parameter("output_height", 640)
        self.declare_parameter("output_encoding", "mono8")

        qos = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Image, self.get_parameter("output_topic").value, qos)
        self._url = str(self.get_parameter("url").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._reconnect_s = max(float(self.get_parameter("reconnect_s").value), 0.1)
        self._period_s = 1.0 / max(float(self.get_parameter("max_fps").value), 1.0)
        self._width = int(self.get_parameter("output_width").value)
        self._height = int(self.get_parameter("output_height").value)
        self._encoding = str(self.get_parameter("output_encoding").value)
        if self._width <= 0 or self._height <= 0:
            raise ValueError("output dimensions must be positive")
        if self._encoding not in ("mono8", "bgr8"):
            raise ValueError("output_encoding must be mono8 or bgr8")
        cv2.setNumThreads(1)

    def _make_image(self, frame) -> Image:
        frame = cv2.resize(frame, (self._width, self._height), interpolation=cv2.INTER_AREA)
        if self._encoding == "mono8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = self._encoding
        msg.is_bigendian = False
        msg.step = msg.width if self._encoding == "mono8" else msg.width * 3
        msg.data = frame.tobytes()
        return msg

    def run(self) -> None:
        if not self._url:
            raise RuntimeError("url parameter is required")
        while rclpy.ok():
            cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().warning(f"unable to open H264 source: {self._url}")
                cap.release()
                time.sleep(self._reconnect_s)
                continue
            self.get_logger().info(f"opened H264 source: {self._url}")
            frames = 0
            while rclpy.ok():
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.get_logger().warning(f"H264 source ended after {frames} frames")
                    break
                started = time.monotonic()
                try:
                    self._pub.publish(self._make_image(frame))
                except RCLError:
                    if not rclpy.ok():
                        break
                    raise
                frames += 1
                if frames == 1 or frames % 25 == 0:
                    self.get_logger().info(
                        f"published {frames} frames {self._width}x{self._height} {self._encoding}"
                    )
                delay = self._period_s - (time.monotonic() - started)
                if delay > 0:
                    time.sleep(delay)
            cap.release()
            time.sleep(self._reconnect_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = S90H264InputNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
