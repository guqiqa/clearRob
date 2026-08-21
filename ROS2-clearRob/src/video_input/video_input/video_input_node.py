#!/usr/bin/env python3
"""
Independent video input frontend.

This node only handles camera ingest, normalization, and caching. It republishes
stable topics that downstream modules consume:
- vision_detector -> sensor/camera/image_raw
- stereo_depth -> sensor/stereo/left/right/image_rect and camera_info
"""

import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from cleaning_robot_interfaces.msg import Heartbeat
from cleaning_robot_interfaces.srv import GetStatus


STATUS_NORMAL = 0
STATUS_DEGRADED = 1
STATUS_FAULT = 2
LIFECYCLE_ACTIVE = 3
ERROR_NONE = 0
ERROR_RUNTIME = 2


class VideoInputNode(Node):
    def __init__(self) -> None:
        super().__init__("video_input")
        self._declare_parameters()

        self._start_time = time.monotonic()
        self._fault_reason = ""
        self._last_rgb_time = 0.0
        self._last_left_time = 0.0
        self._last_right_time = 0.0
        self._latest_rgb: Optional[Image] = None
        self._latest_left: Optional[Image] = None
        self._latest_right: Optional[Image] = None
        self._latest_left_info: Optional[CameraInfo] = None
        self._latest_right_info: Optional[CameraInfo] = None
        self._latest_rgb_info: Optional[CameraInfo] = None
        self._last_sim_tick = 0.0

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pub_rgb = self.create_publisher(
            Image, self.get_parameter("output_rgb_image_topic").value, sensor_qos
        )
        self._pub_rgb_info = self.create_publisher(
            CameraInfo,
            self.get_parameter("output_rgb_camera_info_topic").value,
            info_qos,
        )
        self._pub_left = self.create_publisher(
            Image, self.get_parameter("output_left_image_topic").value, sensor_qos
        )
        self._pub_left_info = self.create_publisher(
            CameraInfo,
            self.get_parameter("output_left_camera_info_topic").value,
            info_qos,
        )
        self._pub_right = self.create_publisher(
            Image, self.get_parameter("output_right_image_topic").value, sensor_qos
        )
        self._pub_right_info = self.create_publisher(
            CameraInfo,
            self.get_parameter("output_right_camera_info_topic").value,
            info_qos,
        )
        self._pub_hb = self.create_publisher(
            Heartbeat, "system/heartbeat/video_input", 10
        )

        self.create_service(GetStatus, "video_input/get_status", self._on_get_status)

        if not bool(self.get_parameter("simulate_input").value):
            self._sub_rgb = self._maybe_subscribe(
                Image,
                self.get_parameter("source_rgb_image_topic").value,
                self._on_rgb_image,
                sensor_qos,
            )
            self._sub_rgb_info = self._maybe_subscribe(
                CameraInfo,
                self.get_parameter("source_rgb_camera_info_topic").value,
                self._on_rgb_info,
                info_qos,
            )
            self._sub_left = self._maybe_subscribe(
                Image,
                self.get_parameter("source_left_image_topic").value,
                self._on_left_image,
                sensor_qos,
            )
            self._sub_left_info = self._maybe_subscribe(
                CameraInfo,
                self.get_parameter("source_left_camera_info_topic").value,
                self._on_left_info,
                info_qos,
            )
            self._sub_right = self._maybe_subscribe(
                Image,
                self.get_parameter("source_right_image_topic").value,
                self._on_right_image,
                sensor_qos,
            )
            self._sub_right_info = self._maybe_subscribe(
                CameraInfo,
                self.get_parameter("source_right_camera_info_topic").value,
                self._on_right_info,
                info_qos,
            )

        target_fps = max(float(self.get_parameter("target_fps").value), 1.0)
        self.create_timer(1.0 / target_fps, self._timer_tick)
        self.create_timer(1.0, self._heartbeat_tick)

        self.get_logger().info(
            "video_input started "
            f"(simulate={bool(self.get_parameter('simulate_input').value)}, "
            f"target_fps={target_fps})"
        )

    def _declare_parameters(self) -> None:
        p = self.declare_parameter
        p("source_rgb_image_topic", "")
        p("source_rgb_camera_info_topic", "")
        p("source_left_image_topic", "")
        p("source_left_camera_info_topic", "")
        p("source_right_image_topic", "")
        p("source_right_camera_info_topic", "")
        p("output_rgb_image_topic", "sensor/camera/image_raw")
        p("output_rgb_camera_info_topic", "sensor/camera/camera_info")
        p("output_left_image_topic", "sensor/stereo/left/image_rect")
        p("output_left_camera_info_topic", "sensor/stereo/left/camera_info")
        p("output_right_image_topic", "sensor/stereo/right/image_rect")
        p("output_right_camera_info_topic", "sensor/stereo/right/camera_info")
        p("publish_rgb_from_left", True)
        p("publish_rgb_from_source", True)
        p("target_fps", 30.0)
        p("simulate_input", False)
        p("simulate_width", 1280)
        p("simulate_height", 720)
        p("frame_id", "camera_link")
        p("status_timeout_s", 2.0)

    def _maybe_subscribe(self, msg_type, topic, cb, qos):
        if not topic:
            return None
        output_topics = {
            self.get_parameter("output_rgb_image_topic").value,
            self.get_parameter("output_rgb_camera_info_topic").value,
            self.get_parameter("output_left_image_topic").value,
            self.get_parameter("output_left_camera_info_topic").value,
            self.get_parameter("output_right_image_topic").value,
            self.get_parameter("output_right_camera_info_topic").value,
        }
        if topic in output_topics:
            self._set_fault(f"source topic must differ from output topic: {topic}")
            return None
        return self.create_subscription(msg_type, topic, cb, qos)

    def _has_source_topic(self) -> bool:
        return any(
            bool(self.get_parameter(name).value)
            for name in (
                "source_rgb_image_topic",
                "source_rgb_camera_info_topic",
                "source_left_image_topic",
                "source_left_camera_info_topic",
                "source_right_image_topic",
                "source_right_camera_info_topic",
            )
        )

    def _on_rgb_image(self, msg: Image) -> None:
        self._latest_rgb = msg
        self._last_rgb_time = time.monotonic()
        if bool(self.get_parameter("publish_rgb_from_source").value):
            self._pub_rgb.publish(msg)

    def _on_rgb_info(self, msg: CameraInfo) -> None:
        self._latest_rgb_info = msg
        self._pub_rgb_info.publish(msg)

    def _on_left_image(self, msg: Image) -> None:
        self._latest_left = msg
        self._last_left_time = time.monotonic()
        self._pub_left.publish(msg)
        if bool(self.get_parameter("publish_rgb_from_left").value) and self._latest_rgb is None:
            self._pub_rgb.publish(msg)

    def _on_left_info(self, msg: CameraInfo) -> None:
        self._latest_left_info = msg
        self._pub_left_info.publish(msg)
        if self._latest_rgb_info is None:
            self._pub_rgb_info.publish(msg)

    def _on_right_image(self, msg: Image) -> None:
        self._latest_right = msg
        self._last_right_time = time.monotonic()
        self._pub_right.publish(msg)

    def _on_right_info(self, msg: CameraInfo) -> None:
        self._latest_right_info = msg
        self._pub_right_info.publish(msg)

    def _timer_tick(self) -> None:
        if bool(self.get_parameter("simulate_input").value):
            self._publish_simulated_frame()
            return

        if not self._has_source_topic():
            self._set_fault("no camera source topics configured")
            return

        timeout_s = float(self.get_parameter("status_timeout_s").value)
        now = time.monotonic()
        if timeout_s > 0.0:
            last_seen = max(self._last_rgb_time, self._last_left_time, self._last_right_time)
            if last_seen > 0.0 and now - last_seen > timeout_s:
                self._set_fault("camera input timeout")
                return
        self._fault_reason = ""

    def _publish_simulated_frame(self) -> None:
        width = int(self.get_parameter("simulate_width").value)
        height = int(self.get_parameter("simulate_height").value)
        now = time.monotonic()
        if now - self._last_sim_tick < 1.0 / max(float(self.get_parameter("target_fps").value), 1.0):
            return
        self._last_sim_tick = now

        left = np.zeros((height, width), dtype=np.uint8)
        right = np.zeros((height, width), dtype=np.uint8)
        x = int((now * 40.0) % max(width - 100, 1))
        left[:, :] = 24
        right[:, :] = 24
        left[height // 3: height // 3 + 120, x:x + 120] = 180
        right[height // 3: height // 3 + 120, max(0, x - 10):max(0, x - 10) + 120] = 180

        self._pub_left.publish(self._make_image(left, "mono8"))
        self._pub_right.publish(self._make_image(right, "mono8"))
        self._pub_rgb.publish(self._make_image(left, "mono8"))

        info = self._make_info(width, height)
        self._pub_left_info.publish(info)
        self._pub_right_info.publish(info)
        self._pub_rgb_info.publish(info)
        self._fault_reason = ""

    def _make_image(self, arr: np.ndarray, encoding: str) -> Image:
        msg = Image()
        msg.header = self._make_header()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width * (1 if encoding == "mono8" else 3)
        msg.data = arr.tobytes()
        return msg

    def _make_info(self, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.header = self._make_header()
        msg.width = width
        msg.height = height
        msg.k = [600.0, 0.0, width / 2.0, 0.0, 600.0, height / 2.0, 0.0, 0.0, 1.0]
        msg.p = [600.0, 0.0, width / 2.0, 0.0, 0.0, 600.0, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("frame_id").value
        return header

    def _set_fault(self, reason: str) -> None:
        if self._fault_reason != reason:
            self.get_logger().warn(reason)
        self._fault_reason = reason

    def _on_get_status(self, request, response):
        response.node_name = "video_input"
        response.uptime_s = float(time.monotonic() - self._start_time)
        if self._fault_reason:
            response.status = STATUS_DEGRADED
            response.status_text = self._fault_reason
        else:
            response.status = STATUS_NORMAL
            response.status_text = "normal"
        return response

    def _heartbeat_tick(self) -> None:
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = "video_input"
        msg.lifecycle_state = LIFECYCLE_ACTIVE
        msg.error_code = ERROR_RUNTIME if self._fault_reason else ERROR_NONE
        self._pub_hb.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
