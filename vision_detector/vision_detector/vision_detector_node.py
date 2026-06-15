"""
ROS2 Lifecycle Node: vision_detector (M1 — basic detection).

Subscriptions
  - sensor/camera/image_raw  (sensor_msgs/Image, Sensor Data QoS)

Publishers
  - vision/detect/list       (vision_detector/DetectionArray)

Lifecycle states: Unconfigured → Inactive → Active → Finalized
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import (
    LifecycleState,
    TransitionCallbackReturn,
)
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from vision_detector.msg import Detection as DetectionMsg
from vision_detector.msg import DetectionArray

from .detection_utils import Detection
from .yolo_detector import YOLODetector

logger = logging.getLogger("vision_detector.node")

# ---------------------------------------------------------------------------
# QoS presets
# ---------------------------------------------------------------------------

CAMERA_QOS = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

DETECTION_QOS = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# ---------------------------------------------------------------------------
# Parameter defaults (mirrors vision_detector.yaml)
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    "model_path": "",
    "model_version": "yolov8",
    "conf_threshold": 0.45,
    "nms_iou_threshold": 0.45,
    "input_width": 640,
    "input_height": 640,
    "device": "cuda:0",
    "half_precision": True,
    "target_fps": 30.0,
    "warn_latency_ms": 50.0,
}

# ---------------------------------------------------------------------------


class VisionDetectorNode(LifecycleNode):
    """ROS2 Lifecycle Node wrapping a YOLOv8 detector."""

    # ------------------------------------------------------------------
    def __init__(self) -> None:
        super().__init__("vision_detector")

        # internal state -------------------------------------------------
        self._detector = YOLODetector()
        self._bridge = CvBridge()
        self._frame_seq: int = 0
        self._last_frame_time: float = 0.0
        self._latency_buffer: list[float] = []  # rolling window (last 30 frames)
        self._warn_count_since_ok: int = 0

        # ROS2 handles (created in on_configure) -------------------------
        self._sub_image: Optional[rclpy.subscription.Subscription] = None
        self._pub_detection: Optional[rclpy.publisher.Publisher] = None

        # parameter overrides --------------------------------------------
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.45)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("half_precision", True)
        self.declare_parameter("target_fps", 30.0)
        self.declare_parameter("warn_latency_ms", 50.0)

        logger.info("vision_detector node constructed (state: Unconfigured)")

    # ==================================================================
    # Lifecycle transitions
    # ==================================================================

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure: load model, validate parameters."""
        logger.info("on_configure() — loading model…")

        model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        if not model_path:
            logger.error(
                "Parameter 'model_path' is empty — cannot configure."
            )
            return TransitionCallbackReturn.ERROR

        ok = self._detector.load(
            model_path=model_path,
            device=self.get_parameter("device").get_parameter_value().string_value,
            conf_threshold=self.get_parameter("conf_threshold").get_parameter_value().double_value,
            iou_threshold=self.get_parameter("nms_iou_threshold").get_parameter_value().double_value,
            input_width=self.get_parameter("input_width").get_parameter_value().integer_value,
            input_height=self.get_parameter("input_height").get_parameter_value().integer_value,
            half_precision=self.get_parameter("half_precision").get_parameter_value().bool_value,
        )

        if not ok:
            logger.error("Model failed to load.")
            return TransitionCallbackReturn.ERROR

        # ROS2 interfaces — created once in configure --------------------
        self._pub_detection = self.create_lifecycle_publisher(
            DetectionArray, "vision/detect/list", DETECTION_QOS
        )

        logger.info("on_configure() succeeded.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate: start image subscription."""
        logger.info("on_activate() — starting subscriptions…")

        self._sub_image = self.create_subscription(
            Image,
            "sensor/camera/image_raw",
            self._image_callback,
            CAMERA_QOS,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._frame_seq = 0
        self._last_frame_time = time.perf_counter()
        self._latency_buffer.clear()
        self._warn_count_since_ok = 0

        logger.info("on_activate() succeeded — node is Active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate: stop subscription (publishers stay alive)."""
        logger.info("on_deactivate() — stopping subscriptions…")
        if self._sub_image is not None:
            self.destroy_subscription(self._sub_image)
            self._sub_image = None
        logger.info("on_deactivate() succeeded — node is Inactive.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup: destroy all ROS2 interfaces & unload model."""
        logger.info("on_cleanup()")
        if self._sub_image is not None:
            self.destroy_subscription(self._sub_image)
            self._sub_image = None
        if self._pub_detection is not None:
            self.destroy_publisher(self._pub_detection)
            self._pub_detection = None
        self._detector.unload()
        logger.info("on_cleanup() succeeded.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        logger.info("on_shutdown() — finalizing node.")
        self._detector.unload()
        return TransitionCallbackReturn.SUCCESS

    # ==================================================================
    # Image callback — core inference loop
    # ==================================================================

    def _image_callback(self, msg: Image) -> None:
        """Receive a camera frame, run YOLO inference, publish detections."""
        if not self._detector.is_loaded:
            return

        # Decode image -------------------------------------------------
        try:
            image: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            logger.warning("Failed to decode image (frame_id=%s)", msg.header.frame_id, exc_info=True)
            return

        # Inference ----------------------------------------------------
        detections, infer_ms = self._detector.detect(image)

        # Track latency ------------------------------------------------
        self._latency_buffer.append(infer_ms)
        if len(self._latency_buffer) > 30:
            self._latency_buffer.pop(0)

        # Build & publish DetectionArray --------------------------------
        self._frame_seq += 1

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id or "camera_optical"

        da = DetectionArray()
        da.header = header
        da.frame_seq = self._frame_seq
        da.inference_ms = float(infer_ms)
        da.detections = [self._detection_to_msg(d) for d in detections]

        self._pub_detection.publish(da)

        # FPS / latency health checks ----------------------------------
        self._check_performance_health(infer_ms)

        # Debug log (throttled) ----------------------------------------
        now = time.perf_counter()
        if now - self._last_frame_time >= 5.0:
            fps = self._frame_seq / max(now - self._last_frame_time, 1e-3)
            avg_lat = (
                sum(self._latency_buffer) / len(self._latency_buffer)
                if self._latency_buffer
                else 0.0
            )
            logger.info(
                "Frame #%d | %.1f FPS | %.1f ms infer | %d detections",
                self._frame_seq, fps, avg_lat, len(detections),
            )
            self._last_frame_time = now
            self._frame_seq = 0

    # ==================================================================
    # Helpers
    # ==================================================================

    @staticmethod
    def _detection_to_msg(d: Detection) -> DetectionMsg:
        msg = DetectionMsg()
        msg.class_name = d.class_name
        msg.confidence = d.confidence
        msg.class_id = d.class_id
        msg.x1 = d.x1
        msg.y1 = d.y1
        msg.x2 = d.x2
        msg.y2 = d.y2
        msg.pixel_area = d.pixel_area
        return msg

    def _check_performance_health(self, infer_ms: float) -> None:
        warn_threshold = (
            self.get_parameter("warn_latency_ms").get_parameter_value().double_value
        )
        if infer_ms > warn_threshold:
            self._warn_count_since_ok += 1
        else:
            self._warn_count_since_ok = 0

        target_fps = (
            self.get_parameter("target_fps").get_parameter_value().double_value
        )
        # If latency exceeds warn threshold for 30 consecutive frames, log prominently
        if self._warn_count_since_ok >= 30:
            logger.warning(
                "Inference latency (%.1f ms) has exceeded warn threshold "
                "(%.1f ms) for 30 consecutive frames — consider lowering "
                "resolution or switching device.",
                infer_ms, warn_threshold,
            )
            self._warn_count_since_ok = 0

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetectorNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
