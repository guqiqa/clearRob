#!/usr/bin/env python3
"""
vision_detector/vision_node.py

ROS2 Node: vision_detector
Lifecycle-managed vision module for the cleaning robot system.

Handles:
  - YOLO-based garbage detection (simulated)
  - Drivable area inference
  - Cleanliness scoring
  - Mode-aware detection strategy
  - Model hot-switch
  - Fault handling and degradation
"""

import math
import os
import random
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.clock import ClockType

from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Image

from cleaning_robot_interfaces.msg import (
    Detection,
    DetectionArray,
    TaskMode,
    VisionStatus,
    Heartbeat,
)
from cleaning_robot_interfaces.srv import SetModel, GetStatus


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CLASS_NAMES = [
    "other_waste",
    "green_waste",
    "recyclable",
    "kitchen_waste",
    "road_obstacle",
    "pedestrian_pet",
    "hazardous",
    "stain",
]

# Class IDs that count as "garbage" for cleanliness scoring
GARBAGE_CLASS_IDS = {0, 1, 2, 3, 4}

# Class IDs for obstacles that block the drivable area
OBSTACLE_CLASS_IDS = {5, 6}

# Mode constants
MODE_CRUISE = 0
MODE_CLEAN = 1
MODE_RETURN = 2

MODE_NAME = {0: "CRUISE", 1: "CLEAN", 2: "RETURN"}

# Vision status codes
STATUS_NORMAL = 0
STATUS_DEGRADED = 1
STATUS_FAULT = 2

# Lifecycle states for heartbeat
LC_UNCONFIGURED = 1
LC_INACTIVE = 2
LC_ACTIVE = 3
LC_FINALIZED = 4

# Heartbeat error codes
ERR_NORMAL = 0
ERR_INIT_FAILED = 1
ERR_RUNTIME_ERROR = 2


# ---------------------------------------------------------------------------
# VisionDetectorNode
# ---------------------------------------------------------------------------


class VisionDetectorNode(LifecycleNode):
    """Lifecycle-managed vision detector node."""

    def __init__(self) -> None:
        super().__init__("vision_detector")

        # --- Parameters (declared in on_configure) ---
        self._params: dict = {}
        self._declare_params()

        # --- Runtime state ---
        self._active_model: str = ""
        self._frame_seq: int = 0
        self._mode: int = MODE_CRUISE
        self._mode_name: str = "CRUISE"

        # Cleanliness
        self._cleanliness: float = 1.0
        self._last_cleanliness_time: float = 0.0

        # Performance tracking
        self._inference_times: List[float] = []
        self._inference_time_window_s = 30.0
        self._degrade_triggered: bool = False
        self._degraded_fps: float = 0.0
        self._no_detection_start: Optional[float] = None
        self._camera_last_time: Optional[float] = None

        # Model switch state
        self._model_switching: bool = False
        self._model_switch_start: float = 0.0
        self._model_switch_duration: float = 0.0
        self._pending_model_name: str = ""

        # Fault state
        self._fault: bool = False
        self._fault_reason: str = ""
        self._in_degraded_mode: bool = False

        # Timers
        self._infer_timer: Optional[Timer] = None
        self._heartbeat_timer: Optional[Timer] = None

        # Start time
        self._start_time: float = time.time()

        self.get_logger().info("VisionDetectorNode __init__ complete")

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure()")

        # Re-declare params so they are available after lifecycle transitions
        self._declare_params()
        self._read_params()

        self._active_model = self._params["model_path"]
        self._model_switching = False

        self._compute_degraded_fps()

        self._frame_seq = 0
        self._cleanliness = 1.0
        self._last_cleanliness_time = time.time()
        self._inference_times.clear()
        self._degrade_triggered = False
        self._fault = False
        self._fault_reason = ""
        self._in_degraded_mode = False
        self._no_detection_start = None
        self._camera_last_time = None
        self._start_time = time.time()

        # Subscriptions
        self.create_subscription(
            Image,
            "sensor/camera/image_raw",
            self._on_image,
            10,
        )
        self.create_subscription(
            TaskMode,
            "master/task/mode",
            self._on_mode,
            10,
        )

        # Publishers
        self._pub_detect_list = self.create_publisher(
            DetectionArray, "vision/detect/list", 10
        )
        self._pub_cleanliness = self.create_publisher(
            Float32, "vision/detect/cleanliness", 10
        )
        self._pub_drivable = self.create_publisher(
            Image, "vision/detect/drivable", 10
        )
        self._pub_status = self.create_publisher(
            VisionStatus, "vision/status", 10
        )
        self._pub_annotated = self.create_publisher(
            Image, "vision/debug/annotated_image", 10
        )
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, "system/heartbeat/vision_detector", 10
        )

        # Services
        self._srv_set_model = self.create_service(
            SetModel, "vision/set_model", self._on_set_model
        )
        self._srv_get_status = self.create_service(
            GetStatus, "vision/get_status", self._on_get_status
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate()")

        # Inference timer
        period = 1.0 / self._params["target_fps"]
        self._infer_timer = self.create_timer(period, self._infer_tick)

        # Heartbeat timer
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_tick)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate()")
        self._destroy_timers()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup()")
        self._destroy_publishers()
        self._destroy_subscriptions()
        self._destroy_services()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown()")
        self._destroy_timers()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter("model_path", "/path/to/models/cleaning_v1.0.0_s_best.engine")
        self.declare_parameter("model_config", "/path/to/models/cleaning_v1.0.0_s_config.yaml")
        self.declare_parameter("model_version", "yolov8")
        self.declare_parameter("conf_threshold", 0.45)
        self.declare_parameter("nms_iou_threshold", 0.45)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("half_precision", False)
        self.declare_parameter("max_batch_size", 1)
        self.declare_parameter("enable_tracking", False)
        self.declare_parameter("publish_annotated_image", False)
        self.declare_parameter("target_fps", 30.0)
        self.declare_parameter("warn_latency_ms", 50.0)
        self.declare_parameter("degrade_fps_ratio", 0.5)
        self.declare_parameter("cleanliness_interval_s", 1.0)
        self.declare_parameter("cleanliness_saturation_ratio", 0.3)
        self.declare_parameter("simulate_camera", True)
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)

    def _read_params(self) -> None:
        """Read all declared parameters into self._params dict."""
        for name in [
            "model_path", "model_config", "model_version", "device",
        ]:
            self._params[name] = self.get_parameter(name).get_parameter_value().string_value
        for name in [
            "conf_threshold", "nms_iou_threshold", "input_width", "input_height",
            "target_fps", "warn_latency_ms", "degrade_fps_ratio",
            "cleanliness_interval_s", "cleanliness_saturation_ratio",
            "camera_width", "camera_height",
        ]:
            self._params[name] = self.get_parameter(name).get_parameter_value().double_value
        for name in [
            "half_precision", "max_batch_size", "enable_tracking",
            "publish_annotated_image", "simulate_camera",
        ]:
            self._params[name] = self.get_parameter(name).get_parameter_value().bool_value

    def _compute_degraded_fps(self) -> None:
        self._degraded_fps = (
            self._params["target_fps"] * self._params["degrade_fps_ratio"]
        )

    # ------------------------------------------------------------------
    # Timer helpers
    # ------------------------------------------------------------------

    def _destroy_timers(self) -> None:
        for t in (self._infer_timer, self._heartbeat_timer):
            if t is not None:
                self.destroy_timer(t)
        self._infer_timer = None
        self._heartbeat_timer = None

    def _destroy_publishers(self) -> None:
        for attr in (
            "_pub_detect_list", "_pub_cleanliness", "_pub_drivable",
            "_pub_status", "_pub_annotated", "_pub_heartbeat",
        ):
            pub = getattr(self, attr, None)
            if pub is not None:
                self.destroy_publisher(pub)
                setattr(self, attr, None)

    def _destroy_subscriptions(self) -> None:
        for sub in self.subscriptions:
            self.destroy_subscription(sub)

    def _destroy_services(self) -> None:
        for srv in (self._srv_set_model, self._srv_get_status):
            if srv is not None:
                self.destroy_service(srv)
        self._srv_set_model = None
        self._srv_get_status = None

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        if self._fault:
            return
        self._camera_last_time = time.time()

    def _on_mode(self, msg: TaskMode) -> None:
        prev_mode = self._mode
        self._mode = msg.mode
        self._mode_name = MODE_NAME.get(msg.mode, "UNKNOWN")
        if prev_mode != self._mode:
            self.get_logger().info(
                f"Mode switched: {MODE_NAME.get(prev_mode, '?')} -> {self._mode_name}"
            )

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_set_model(self, request: SetModel.Request, response: SetModel.Response) -> SetModel.Response:
        self.get_logger().info(f"set_model requested: {request.model_path}")

        # Validate path
        if not request.model_path or not os.path.isabs(request.model_path):
            response.success = False
            response.message = f"Invalid model_path: {request.model_path}"
            self.get_logger().warn(response.message)
            return response

        # Check extension
        ext = os.path.splitext(request.model_path)[1].lower()
        if ext not in (".pt", ".onnx", ".engine"):
            response.success = False
            response.message = f"Unsupported model format: {ext}"
            self.get_logger().warn(response.message)
            return response

        # Simulate async switch
        self._model_switching = True
        self._model_switch_start = time.time()
        self._model_switch_duration = random.uniform(2.0, 3.0)
        self._pending_model_name = request.model_path

        response.success = True
        response.message = (
            f"Model switch initiated: {request.model_path} "
            f"(ETA {self._model_switch_duration:.1f}s)"
        )
        return response

    def _on_get_status(self, request: GetStatus.Request, response: GetStatus.Response) -> GetStatus.Response:
        response.node_name = "vision_detector"
        if self._fault:
            response.status = STATUS_FAULT
            response.status_text = f"FAULT: {self._fault_reason}"
        elif self._in_degraded_mode:
            response.status = STATUS_DEGRADED
            response.status_text = "DEGRADED: high latency, reduced rate"
        else:
            response.status = STATUS_NORMAL
            response.status_text = "NORMAL"
        response.uptime_s = float(time.time() - self._start_time)
        return response

    # ------------------------------------------------------------------
    # Inference tick
    # ------------------------------------------------------------------

    def _infer_tick(self) -> None:
        """Main inference loop, called at target_fps (or degraded rate)."""
        if self._fault:
            return

        now = time.time()

        # --- Model switch completion check ---
        self._check_model_switch(now)

        # --- Fault: camera timeout ---
        self._check_camera_timeout(now)

        # --- Generate or use input image ---
        image_width = self._params["camera_width"]
        image_height = self._params["camera_height"]

        # --- Simulate inference latency ---
        inference_ms = max(0.0, random.gauss(28.0, 5.0))
        inference_ms = max(20.0, min(40.0, inference_ms))

        # Degraded-mode rate limiting: skip frames
        if self._in_degraded_mode:
            effective_fps = self._degraded_fps
            skip = random.random() > (effective_fps / self._params["target_fps"])
            if skip:
                return
        else:
            effective_fps = self._params["target_fps"]

        # --- Simulated detections ---
        self._frame_seq += 1
        detections = self._simulate_detections(image_width, image_height)

        # --- Filter per mode ---
        detections = self._filter_by_mode(detections)

        # --- Cleanliness scoring ---
        self._update_cleanliness(detections, image_width, image_height, now)

        # --- Publishes ---
        self._publish_detect_list(detections, inference_ms, now)
        self._publish_cleanliness(now)
        self._publish_drivable(detections, image_width, image_height, now)
        self._publish_annotated(detections)

        # --- Status tracking ---
        self._track_latency(inference_ms, now)
        self._check_no_detections(detections, now)

        # --- Status at 1 Hz ---
        self._publish_status_if_due(inference_ms, effective_fps, now)

    # ------------------------------------------------------------------
    # Simulated detection
    # ------------------------------------------------------------------

    def _simulate_detections(self, img_width: int, img_height: int) -> List[Detection]:
        """Generate synthetic detections mimicking a real YOLO model."""

        # 70% chance of 0-2 detections (clean scenes)
        if random.random() < 0.70:
            num_detections = random.randint(0, 2)
        else:
            num_detections = random.randint(0, 8)

        # Class distribution weights
        class_weights = [0.30, 0.20, 0.15, 0.10, 0.10, 0.05, 0.05, 0.05]
        class_ids = random.choices(range(8), weights=class_weights, k=num_detections)

        detections: List[Detection] = []
        for cid in class_ids:
            # Random bbox: width 40-200px, height 40-200px
            w = random.uniform(40.0, 200.0)
            h = random.uniform(40.0, 200.0)
            x1 = random.uniform(0.0, float(img_width) - w)
            y1 = random.uniform(0.0, float(img_height) - h)
            x2 = x1 + w
            y2 = y1 + h

            conf = random.uniform(0.45, 0.95)

            det = Detection()
            det.class_name = CLASS_NAMES[cid]
            det.confidence = float(conf)
            det.class_id = cid
            det.x1 = float(x1)
            det.y1 = float(y1)
            det.x2 = float(x2)
            det.y2 = float(y2)
            det.pixel_area = float(w * h)
            detections.append(det)

        return detections

    # ------------------------------------------------------------------
    # Mode-aware filtering
    # ------------------------------------------------------------------

    def _filter_by_mode(self, detections: List[Detection]) -> List[Detection]:
        """Filter detections based on current operational mode.

        CRUISE: all 8 classes
        CLEAN:  class_id 0-4 (garbage) + 7 (stain)
        RETURN: class_id 5-6 (road_obstacle + pedestrian_pet)
        """
        if self._mode == MODE_CRUISE:
            return detections

        if self._mode == MODE_CLEAN:
            return [d for d in detections if d.class_id in (0, 1, 2, 3, 4, 7)]

        if self._mode == MODE_RETURN:
            return [d for d in detections if d.class_id in (5, 6)]

        return detections

    # ------------------------------------------------------------------
    # Cleanliness scoring
    # ------------------------------------------------------------------

    def _update_cleanliness(
        self,
        detections: List[Detection],
        img_width: int,
        img_height: int,
        now: float,
    ) -> None:
        total_pixels = float(img_width * img_height)
        saturation = self._params["cleanliness_saturation_ratio"]  # 0.3

        # Only garbage classes (0-4) count for cleanliness
        garbage_area = sum(
            d.pixel_area for d in detections if d.class_id in GARBAGE_CLASS_IDS
        )
        garbage_ratio = garbage_area / max(total_pixels, 1.0)
        cleanliness = max(0.0, 1.0 - garbage_ratio / saturation)
        self._cleanliness = float(cleanliness)

    # ------------------------------------------------------------------
    # Drivable area
    # ------------------------------------------------------------------

    def _publish_drivable(
        self,
        detections: List[Detection],
        img_width: int,
        img_height: int,
        now: float,
    ) -> None:
        """Publish drivable-area mask (MONO8, 640x320).

        Only published in CRUISE mode.
        """
        if self._mode != MODE_CRUISE:
            return

        # Work with lower half: rows [img_height//2 .. img_height)
        half_start = img_height // 2
        half_height = img_height - half_start

        # Create mask: 255 = drivable
        mask = np.full((half_height, img_width), 255, dtype=np.uint8)

        # Mark obstacle footprints as non-drivable (0)
        for d in detections:
            if d.class_id not in OBSTACLE_CLASS_IDS:
                continue
            # Only obstacles that intersect the lower half
            if d.y2 <= half_start:
                continue
            y1_local = max(0, int(d.y1) - half_start)
            y2_local = min(half_height, max(0, int(d.y2) - half_start))
            x1_local = max(0, int(d.x1))
            x2_local = min(img_width, int(d.x2))
            if y2_local > y1_local and x2_local > x1_local:
                mask[y1_local:y2_local, x1_local:x2_local] = 0

        # Resize to 640x320 using nearest-neighbor indexing
        h_ratio = half_height / 320.0
        w_ratio = img_width / 640.0
        row_idx = (np.arange(320) * h_ratio).astype(np.int32)
        col_idx = (np.arange(640) * w_ratio).astype(np.int32)
        scaled = mask[row_idx[:, None], col_idx]

        # Build Image message
        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_link"
        ros_img.height = 320
        ros_img.width = 640
        ros_img.encoding = "mono8"
        ros_img.is_bigendian = 0
        ros_img.step = 640
        ros_img.data = scaled.tobytes()

        self._pub_drivable.publish(ros_img)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_detect_list(
        self,
        detections: List[Detection],
        inference_ms: float,
        now: float,
    ) -> None:
        if self._fault:
            return

        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.detections = detections
        msg.frame_seq = self._frame_seq
        msg.inference_ms = float(inference_ms)
        self._pub_detect_list.publish(msg)

    def _publish_cleanliness(self, now: float) -> None:
        """Publish cleanliness.

        CRUISE/RETURN: at configured interval (default 1 Hz)
        CLEAN: at inference rate
        """
        if self._mode == MODE_CLEAN:
            # publish every inference tick
            pass  # fall through to publish
        else:
            interval = self._params["cleanliness_interval_s"]
            if now - self._last_cleanliness_time < interval:
                return

        val = -1.0 if self._fault else self._cleanliness
        msg = Float32()
        msg.data = float(val)
        self._pub_cleanliness.publish(msg)
        self._last_cleanliness_time = now

    def _publish_status_if_due(
        self, inference_ms: float, fps: float, now: float
    ) -> None:
        """Publish VisionStatus at 1 Hz."""
        if not hasattr(self, "_last_status_time"):
            self._last_status_time = 0.0
        if now - self._last_status_time < 1.0:
            return

        status_msg = VisionStatus()
        status_msg.active_model = self._active_model
        status_msg.avg_inference_ms = float(inference_ms)
        status_msg.fps = float(fps)
        if self._fault:
            status_msg.status = STATUS_FAULT
            status_msg.status_text = f"FAULT: {self._fault_reason}"
        elif self._in_degraded_mode:
            status_msg.status = STATUS_DEGRADED
            status_msg.status_text = "DEGRADED"
        else:
            status_msg.status = STATUS_NORMAL
            status_msg.status_text = "NORMAL"
        self._pub_status.publish(status_msg)
        self._last_status_time = now

    def _publish_annotated(self, detections: List[Detection]) -> None:
        """Publish annotated debug image if enabled."""
        if not self._params["publish_annotated_image"]:
            return

        # Generate a synthetic debug image with overlaid bboxes
        w, h = self._params["camera_width"], self._params["camera_height"]
        # Create a dark-gray background
        buf = np.full((h, w, 3), 60, dtype=np.uint8)
        # Draw detections as white rectangles
        for d in detections:
            x1, y1 = int(d.x1), int(d.y1)
            x2, y2 = int(d.x2), int(d.y2)
            buf[y1:y2, x1:x2] = [200, 200, 200]

        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_link"
        ros_img.height = h
        ros_img.width = w
        ros_img.encoding = "rgb8"
        ros_img.is_bigendian = 0
        ros_img.step = w * 3
        ros_img.data = buf.tobytes()

        self._pub_annotated.publish(ros_img)

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------

    def _heartbeat_tick(self) -> None:
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = "vision_detector"

        state_id = self.get_current_state().state_id
        # Map lifecycle states to heartbeat enum
        if state_id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
            msg.lifecycle_state = LC_UNCONFIGURED
        elif state_id == LifecycleState.PRIMARY_STATE_INACTIVE:
            msg.lifecycle_state = LC_INACTIVE
        elif state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
            msg.lifecycle_state = LC_ACTIVE
        elif state_id == LifecycleState.PRIMARY_STATE_FINALIZED:
            msg.lifecycle_state = LC_FINALIZED
        else:
            msg.lifecycle_state = LC_ACTIVE

        msg.error_code = ERR_RUNTIME_ERROR if self._fault else ERR_NORMAL
        self._pub_heartbeat.publish(msg)

    # ------------------------------------------------------------------
    # Fault & degradation monitoring
    # ------------------------------------------------------------------

    def _check_model_switch(self, now: float) -> None:
        if not self._model_switching:
            return
        if now - self._model_switch_start >= self._model_switch_duration:
            self._model_switching = False
            self._active_model = self._pending_model_name
            self.get_logger().info(f"Model switch complete: {self._active_model}")

    def _check_camera_timeout(self, now: float) -> None:
        if self._camera_last_time is None:
            # No camera image yet; if simulate_camera=true, treat as OK
            if self._params["simulate_camera"]:
                return
            return

        if now - self._camera_last_time > 2.0:
            if not self._fault:
                self._fault = True
                self._fault_reason = "Camera timeout (>2s)"
                self.get_logger().error(self._fault_reason)
        else:
            if self._fault and self._fault_reason == "Camera timeout (>2s)":
                self._fault = False
                self._fault_reason = ""
                self.get_logger().info("Camera recovered")

    def _track_latency(self, inference_ms: float, now: float) -> None:
        self._inference_times.append((now, inference_ms))

        # Prune old entries
        cutoff = now - self._inference_time_window_s
        self._inference_times = [
            (t, v) for t, v in self._inference_times if t >= cutoff
        ]

        if not self._inference_times:
            return

        avg_ms = sum(v for _, v in self._inference_times) / len(self._inference_times)
        warn_ms = self._params["warn_latency_ms"]

        if avg_ms > warn_ms and not self._in_degraded_mode:
            self._in_degraded_mode = True
            self.get_logger().warn(
                f"Degraded mode: avg inference {avg_ms:.1f}ms > {warn_ms:.0f}ms"
            )
        elif avg_ms <= warn_ms and self._in_degraded_mode:
            self._in_degraded_mode = False
            self.get_logger().info("Recovered from degraded mode")

    def _check_no_detections(self, detections: List[Detection], now: float) -> None:
        """Track no-detection duration. Not a fault — normal for clean areas."""
        if detections:
            self._no_detection_start = None
            return

        if self._no_detection_start is None:
            self._no_detection_start = now
        elif now - self._no_detection_start > 10.0:
            # Clean area — set cleanliness to 1.0
            self._cleanliness = 1.0


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = VisionDetectorNode()
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
