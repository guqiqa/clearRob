#!/usr/bin/env python3
"""
Lightweight stereo depth frontend.

Primary purpose: provide depth image and sparse point cloud for SLAM mapping.
Obstacle avoidance remains owned by the radar/fusion pipeline.
"""

import base64
import hashlib
import json
import math
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    cv2 = None
    _HAS_CV2 = False

try:
    import sensor_msgs_py.point_cloud2 as pc2
    _HAS_PC2 = True
except Exception:
    pc2 = None
    _HAS_PC2 = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from stereo_msgs.msg import DisparityImage

from cleaning_robot_interfaces.msg import Heartbeat, StereoDistance, VisionStatus
from cleaning_robot_interfaces.srv import GetStatus, GetStereoDistance


STATUS_NORMAL = 0
STATUS_DEGRADED = 1
STATUS_FAULT = 2
LIFECYCLE_ACTIVE = 3
ERROR_NONE = 0
ERROR_RUNTIME = 2


class StereoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("stereo_depth")
        self._declare_parameters()

        self._start_time = time.monotonic()
        self._left_msg: Optional[Image] = None
        self._right_msg: Optional[Image] = None
        self._left_info: Optional[CameraInfo] = None
        self._right_info: Optional[CameraInfo] = None
        self._left_recv_time = 0.0
        self._right_recv_time = 0.0
        self._last_frame_time = 0.0
        self._last_compute_ms = 0.0
        self._frame_seq = 0
        self._fault_reason = ""
        self._latest_lock = threading.Lock()
        self._latest = {
            "ok": False,
            "source": "startup",
            "timestamp": 0.0,
            "frame_seq": 0,
            "distance_m": 0.0,
            "confidence": 0.0,
            "valid_pixels": 0,
            "total_pixels": 0,
            "compute_ms": 0.0,
        }
        self._latest_depth: Optional[np.ndarray] = None
        self._calibration = None
        self._rectify_maps = {}
        if _HAS_CV2:
            self._calibration = self._load_calibration(
                str(self.get_parameter("calibration_file").value)
            )

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

        self.create_subscription(
            Image,
            self.get_parameter("left_image_topic").value,
            self._on_left_image,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.get_parameter("right_image_topic").value,
            self._on_right_image,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("left_camera_info_topic").value,
            self._on_left_info,
            info_qos,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("right_camera_info_topic").value,
            self._on_right_info,
            info_qos,
        )

        self._depth_pub = self.create_publisher(
            Image, "sensor/stereo/depth/image_raw", sensor_qos,
        )
        self._aligned_depth_pub = self.create_publisher(
            Image, "sensor/camera/aligned_depth_to_color/image_raw", sensor_qos,
        )
        self._disparity_pub = self.create_publisher(
            DisparityImage, "sensor/stereo/disparity", sensor_qos,
        )
        self._points_pub = self.create_publisher(
            PointCloud2, "sensor/stereo/points", sensor_qos,
        )
        self._distance_pub = self.create_publisher(
            StereoDistance, "sensor/stereo/distance", 10,
        )
        self._status_pub = self.create_publisher(
            VisionStatus, "stereo_depth/status", 10,
        )
        self._heartbeat_pub = self.create_publisher(
            Heartbeat, "system/heartbeat/stereo_depth", 10,
        )

        self.create_service(GetStatus, "stereo_depth/get_status", self._on_get_status)
        self.create_service(
            GetStereoDistance,
            "stereo_depth/get_distance",
            self._on_get_distance,
        )

        self._http_server = None
        if self.get_parameter("enable_http").value:
            self._start_http_server()

        period = 1.0 / max(float(self.get_parameter("target_fps").value), 0.1)
        self.create_timer(period, self._compute_tick)
        self.create_timer(1.0, self._status_tick)
        self.create_timer(1.0, self._heartbeat_tick)

        self.get_logger().info(
            "stereo_depth started for SLAM mapping "
            f"(target_fps={self.get_parameter('target_fps').value}, "
            f"compute={self.get_parameter('compute_width').value}x"
            f"{self.get_parameter('compute_height').value}, cv2={_HAS_CV2})"
        )

    def _declare_parameters(self) -> None:
        p = self.declare_parameter
        p("left_image_topic", "sensor/stereo/left/image_rect")
        p("right_image_topic", "sensor/stereo/right/image_rect")
        p("left_camera_info_topic", "sensor/stereo/left/camera_info")
        p("right_camera_info_topic", "sensor/stereo/right/camera_info")
        p("assume_rectified", True)
        p("calibration_file", "")
        p("checkerboard_cols", 7)
        p("checkerboard_rows", 10)
        p("square_size_m", 0.02)
        p("target_fps", 5.0)
        p("compute_width", 640)
        p("compute_height", 360)
        p("publish_depth", True)
        p("publish_aligned_depth_compat", True)
        p("publish_disparity", True)
        p("publish_pointcloud", True)
        p("pointcloud_stride", 8)
        p("pointcloud_max_points", 20000)
        p("focal_px", 600.0)
        p("baseline_m", 0.07)
        p("depth_min_m", 0.25)
        p("depth_max_m", 8.0)
        p("num_disparities", 64)
        p("block_size", 7)
        p("uniqueness_ratio", 10)
        p("speckle_window_size", 80)
        p("speckle_range", 2)
        p("disp12_max_diff", 1)
        p("roi_x", 288)
        p("roi_y", 150)
        p("roi_width", 64)
        p("roi_height", 60)
        p("min_valid_ratio", 0.05)
        p("max_pair_time_diff_ms", 80.0)
        p("enable_http", True)
        p("http_host", "0.0.0.0")
        p("http_port", 8091)
        p("simulate_input", False)
        p("frame_id", "stereo_camera_link")
        p("status_timeout_s", 2.0)

    def _on_left_image(self, msg: Image) -> None:
        self._left_msg = msg
        self._left_recv_time = time.monotonic()
        self._last_frame_time = time.monotonic()

    def _on_right_image(self, msg: Image) -> None:
        self._right_msg = msg
        self._right_recv_time = time.monotonic()
        self._last_frame_time = time.monotonic()

    def _on_left_info(self, msg: CameraInfo) -> None:
        self._left_info = msg

    def _on_right_info(self, msg: CameraInfo) -> None:
        self._right_info = msg

    def _compute_tick(self) -> None:
        if not _HAS_CV2:
            self._set_fault("OpenCV is not available")
            return

        timeout_s = float(self.get_parameter("status_timeout_s").value)
        if timeout_s > 0.0:
            last_seen = max(self._left_recv_time, self._right_recv_time)
            if last_seen > 0.0 and (time.monotonic() - last_seen) > timeout_s:
                self._set_fault("waiting for stereo frames")
                return

        started = time.monotonic()
        try:
            left, right, header = self._get_input_pair()
            if left is None or right is None:
                if self.get_parameter("simulate_input").value:
                    left, right, header = self._make_simulated_pair()
                else:
                    self._set_fault("waiting for stereo image pair")
                    return

            disparity = self._compute_disparity(left, right)
            depth = self._disparity_to_depth(disparity)
            distance = self._distance_from_depth(depth)

            self._frame_seq += 1
            self._last_compute_ms = (time.monotonic() - started) * 1000.0
            self._fault_reason = ""
            self._publish_outputs(header, disparity, depth, distance)
            self._update_latest(distance, depth, "stereo_sgbm")
        except Exception as exc:
            self._set_fault(f"depth compute failed: {exc}")

    def _get_input_pair(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Header]:
        if self._left_msg is None or self._right_msg is None:
            return None, None, self._make_header()

        max_pair_dt = float(self.get_parameter("max_pair_time_diff_ms").value) / 1000.0
        if max_pair_dt > 0.0:
            pair_dt = abs(self._left_recv_time - self._right_recv_time)
            if pair_dt > max_pair_dt:
                return None, None, self._make_header()

        left = self._image_to_gray(self._left_msg)
        right = self._image_to_gray(self._right_msg)
        left, right = self._rectify_pair(left, right)
        width = int(self.get_parameter("compute_width").value)
        height = int(self.get_parameter("compute_height").value)
        left = cv2.resize(left, (width, height), interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, (width, height), interpolation=cv2.INTER_AREA)

        header = Header()
        header.stamp = self._left_msg.header.stamp
        header.frame_id = self.get_parameter("frame_id").value
        return left, right, header

    def _make_simulated_pair(self) -> Tuple[np.ndarray, np.ndarray, Header]:
        width = int(self.get_parameter("compute_width").value)
        height = int(self.get_parameter("compute_height").value)
        left = np.zeros((height, width), dtype=np.uint8)
        cv2.rectangle(left, (width // 3, height // 3), (width * 2 // 3, height * 2 // 3), 180, -1)
        cv2.circle(left, (width // 2, height // 2), min(width, height) // 9, 240, -1)
        shift = max(4, width // 80)
        right = np.roll(left, -shift, axis=1)
        right[:, -shift:] = 0
        return left, right, self._make_header()

    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("frame_id").value
        return header

    @staticmethod
    def _image_to_gray(msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if enc in ("mono8", "8uc1"):
            return raw.reshape((msg.height, msg.step))[:, :msg.width]
        if enc in ("rgb8", "bgr8"):
            arr = raw.reshape((msg.height, msg.step))[:, : msg.width * 3]
            arr = arr.reshape((msg.height, msg.width, 3))
            code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
            return cv2.cvtColor(arr, code)
        if enc in ("mono16", "16uc1"):
            raw16 = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            arr = raw16.reshape((msg.height, msg.step // 2))[:, :msg.width]
            max_val = max(float(arr.max()), 1.0)
            return np.clip(arr.astype(np.float32) * (255.0 / max_val), 0, 255).astype(np.uint8)
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    def _load_calibration(self, path: str):
        if not path:
            return None
        expanded = os.path.expandvars(os.path.expanduser(path))
        if not os.path.isfile(expanded):
            self.get_logger().warn(f"calibration file not found: {expanded}")
            return None

        fs = cv2.FileStorage(expanded, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().warn(f"failed to open calibration file: {expanded}")
            return None

        def read_mat(*names):
            for name in names:
                node = fs.getNode(name)
                if not node.empty():
                    value = node.mat()
                    if value is not None:
                        return value.astype(np.float64)
            return None

        def read_scalar(*names, default=0.0):
            for name in names:
                node = fs.getNode(name)
                if not node.empty():
                    return float(node.real())
            return default

        calib = {
            "K1": read_mat("K_left", "K1", "camera_matrix_left"),
            "D1": read_mat("D_left", "D1", "distortion_left"),
            "K2": read_mat("K_right", "K2", "camera_matrix_right"),
            "D2": read_mat("D_right", "D2", "distortion_right"),
            "R": read_mat("R"),
            "T": read_mat("T"),
            "R1": read_mat("R1"),
            "R2": read_mat("R2"),
            "P1": read_mat("P1"),
            "P2": read_mat("P2"),
            "Q": read_mat("Q"),
            "image_width": int(read_scalar("image_width", "width", default=0.0)),
            "image_height": int(read_scalar("image_height", "height", default=0.0)),
        }
        fs.release()

        required = ("K1", "D1", "K2", "D2")
        if any(calib[name] is None for name in required):
            self.get_logger().warn(
                "calibration file is missing K/D matrices; using CameraInfo/default geometry"
            )
            return None
        self.get_logger().info(f"loaded stereo calibration: {expanded}")
        return calib

    def _rectify_pair(self, left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self._calibration is None or bool(self.get_parameter("assume_rectified").value):
            return left, right

        height, width = left.shape[:2]
        expected_w = int(self._calibration.get("image_width") or 0)
        expected_h = int(self._calibration.get("image_height") or 0)
        if expected_w and expected_h and (width != expected_w or height != expected_h):
            raise ValueError(
                "calibration image size mismatch: "
                f"calib={expected_w}x{expected_h}, frame={width}x{height}"
            )

        key = (width, height)
        maps = self._rectify_maps.get(key)
        if maps is None:
            K1 = self._calibration["K1"]
            D1 = self._calibration["D1"]
            K2 = self._calibration["K2"]
            D2 = self._calibration["D2"]
            R1 = self._calibration.get("R1")
            R2 = self._calibration.get("R2")
            P1 = self._calibration.get("P1")
            P2 = self._calibration.get("P2")
            if R1 is None or R2 is None or P1 is None or P2 is None:
                R = self._calibration.get("R")
                T = self._calibration.get("T")
                if R is None or T is None:
                    raise ValueError("calibration file lacks R/T or rectification matrices")
                R1, R2, P1, P2, _ = cv2.stereoRectify(
                    K1, D1, K2, D2, (width, height), R, T, flags=cv2.CALIB_ZERO_DISPARITY
                )
            left_maps = cv2.initUndistortRectifyMap(
                K1, D1, R1, P1, (width, height), cv2.CV_16SC2
            )
            right_maps = cv2.initUndistortRectifyMap(
                K2, D2, R2, P2, (width, height), cv2.CV_16SC2
            )
            maps = (left_maps, right_maps)
            self._rectify_maps[key] = maps

        (left_map1, left_map2), (right_map1, right_map2) = maps
        return (
            cv2.remap(left, left_map1, left_map2, cv2.INTER_LINEAR),
            cv2.remap(right, right_map1, right_map2, cv2.INTER_LINEAR),
        )

    def _compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        num_disp = int(self.get_parameter("num_disparities").value)
        num_disp = max(16, int(math.ceil(num_disp / 16.0)) * 16)
        block_size = int(self.get_parameter("block_size").value)
        block_size = max(3, block_size | 1)
        matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=int(self.get_parameter("disp12_max_diff").value),
            uniquenessRatio=int(self.get_parameter("uniqueness_ratio").value),
            speckleWindowSize=int(self.get_parameter("speckle_window_size").value),
            speckleRange=int(self.get_parameter("speckle_range").value),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        return matcher.compute(left, right).astype(np.float32) / 16.0

    def _stereo_geometry(self) -> Tuple[float, float]:
        fx = float(self.get_parameter("focal_px").value)
        baseline = float(self.get_parameter("baseline_m").value)
        source_width = 0
        if self._calibration is not None:
            p1 = self._calibration.get("P1")
            k1 = self._calibration.get("K1")
            p2 = self._calibration.get("P2")
            q = self._calibration.get("Q")
            t = self._calibration.get("T")
            if p1 is not None and p1[0, 0] > 0:
                fx = float(p1[0, 0])
            elif k1 is not None and k1[0, 0] > 0:
                fx = float(k1[0, 0])
            if p2 is not None and p2[0, 0] > 0 and p2[0, 3] != 0.0:
                baseline = abs(float(p2[0, 3]) / float(p2[0, 0]))
            elif q is not None and q.shape[0] >= 4 and q.shape[1] >= 3 and q[3, 2] != 0.0:
                baseline = abs(1.0 / float(q[3, 2]))
            elif t is not None:
                baseline = float(np.linalg.norm(t.reshape(-1)))
            source_width = int(self._calibration.get("image_width") or 0)
        if self._left_info is not None and self._left_info.k[0] > 0:
            fx = float(self._left_info.k[0])
            source_width = int(self._left_info.width)
        if self._right_info is not None and self._right_info.p[0] > 0:
            tx = float(self._right_info.p[3])
            if tx != 0.0:
                baseline = abs(tx / float(self._right_info.p[0]))
        if source_width > 0:
            fx *= float(self.get_parameter("compute_width").value) / float(source_width)
        return fx, baseline

    def _disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        fx, baseline = self._stereo_geometry()
        min_depth = float(self.get_parameter("depth_min_m").value)
        max_depth = float(self.get_parameter("depth_max_m").value)
        depth = np.zeros_like(disparity, dtype=np.float32)
        valid = disparity > 0.5
        depth[valid] = (fx * baseline) / disparity[valid]
        depth[(depth < min_depth) | (depth > max_depth)] = 0.0
        return depth

    def _distance_from_depth(self, depth: np.ndarray) -> dict:
        x = int(self.get_parameter("roi_x").value)
        y = int(self.get_parameter("roi_y").value)
        w = int(self.get_parameter("roi_width").value)
        h = int(self.get_parameter("roi_height").value)
        return self._distance_for_roi(depth, x, y, w, h)

    def _distance_for_roi(self, depth: np.ndarray, x: int, y: int, w: int, h: int) -> dict:
        height, width = depth.shape[:2]
        x0 = max(0, min(width, x))
        y0 = max(0, min(height, y))
        x1 = max(x0, min(width, x0 + max(1, w)))
        y1 = max(y0, min(height, y0 + max(1, h)))
        roi = depth[y0:y1, x0:x1]
        valid = roi[roi > 0.0]
        total = int(roi.size)
        valid_count = int(valid.size)
        min_ratio = float(self.get_parameter("min_valid_ratio").value)
        if total == 0 or valid_count / max(total, 1) < min_ratio:
            return {
                "distance_m": 0.0,
                "confidence": 0.0,
                "valid_pixels": valid_count,
                "total_pixels": total,
                "roi": [x0, y0, x1 - x0, y1 - y0],
            }
        confidence = min(1.0, valid_count / max(total, 1))
        return {
            "distance_m": float(np.median(valid)),
            "confidence": float(confidence),
            "valid_pixels": valid_count,
            "total_pixels": total,
            "roi": [x0, y0, x1 - x0, y1 - y0],
        }

    def _publish_outputs(self, header: Header, disparity: np.ndarray, depth: np.ndarray, distance: dict) -> None:
        if self.get_parameter("publish_depth").value:
            msg = self._depth_to_image(header, depth)
            self._depth_pub.publish(msg)
            if self.get_parameter("publish_aligned_depth_compat").value:
                self._aligned_depth_pub.publish(msg)

        if self.get_parameter("publish_disparity").value:
            self._disparity_pub.publish(self._disparity_to_msg(header, disparity))

        if self.get_parameter("publish_pointcloud").value and _HAS_PC2:
            cloud = self._depth_to_pointcloud(header, depth)
            if cloud is not None:
                self._points_pub.publish(cloud)

        dist_msg = StereoDistance()
        dist_msg.header = header
        dist_msg.roi_x, dist_msg.roi_y, dist_msg.roi_width, dist_msg.roi_height = distance["roi"]
        dist_msg.distance_m = float(distance["distance_m"])
        dist_msg.confidence = float(distance["confidence"])
        dist_msg.valid_pixels = int(distance["valid_pixels"])
        dist_msg.total_pixels = int(distance["total_pixels"])
        dist_msg.source = "stereo_sgbm"
        self._distance_pub.publish(dist_msg)

    @staticmethod
    def _depth_to_image(header: Header, depth: np.ndarray) -> Image:
        mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        msg = Image()
        msg.header = header
        msg.height, msg.width = mm.shape
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = mm.tobytes()
        return msg

    def _disparity_to_msg(self, header: Header, disparity: np.ndarray) -> DisparityImage:
        img = Image()
        img.header = header
        img.height, img.width = disparity.shape
        img.encoding = "32FC1"
        img.is_bigendian = 0
        img.step = img.width * 4
        img.data = disparity.astype(np.float32).tobytes()

        fx, baseline = self._stereo_geometry()
        msg = DisparityImage()
        msg.header = header
        msg.image = img
        msg.f = float(fx)
        msg.t = float(baseline)
        msg.min_disparity = 0.0
        msg.max_disparity = float(self.get_parameter("num_disparities").value)
        msg.delta_d = 1.0 / 16.0
        return msg

    def _depth_to_pointcloud(self, header: Header, depth: np.ndarray) -> Optional[PointCloud2]:
        if not _HAS_PC2:
            return None
        fx, _ = self._stereo_geometry()
        fy = fx
        cx = depth.shape[1] / 2.0
        cy = depth.shape[0] / 2.0
        if self._left_info is not None and self._left_info.k[4] > 0:
            fy = float(self._left_info.k[4])
            cx = float(self._left_info.k[2]) * depth.shape[1] / max(self._left_info.width, 1)
            cy = float(self._left_info.k[5]) * depth.shape[0] / max(self._left_info.height, 1)

        stride = max(1, int(self.get_parameter("pointcloud_stride").value))
        max_points = max(1, int(self.get_parameter("pointcloud_max_points").value))
        ys, xs = np.where(depth[::stride, ::stride] > 0.0)
        points = []
        for yy, xx in zip(ys[:max_points], xs[:max_points]):
            u = float(xx * stride)
            v = float(yy * stride)
            z = float(depth[int(v), int(u)])
            points.append(((u - cx) * z / fx, (v - cy) * z / fy, z))
        return pc2.create_cloud_xyz32(header, points)

    def _update_latest(self, distance: dict, depth: np.ndarray, source: str) -> None:
        with self._latest_lock:
            self._latest_depth = depth.copy()
            self._latest = {
                "ok": distance["distance_m"] > 0.0,
                "source": source,
                "timestamp": time.time(),
                "frame_seq": self._frame_seq,
                "distance_m": distance["distance_m"],
                "confidence": distance["confidence"],
                "valid_pixels": distance["valid_pixels"],
                "total_pixels": distance["total_pixels"],
                "roi": distance["roi"],
                "compute_ms": self._last_compute_ms,
            }

    def _set_fault(self, reason: str) -> None:
        if self._fault_reason != reason:
            self.get_logger().warn(reason)
        self._fault_reason = reason
        with self._latest_lock:
            self._latest.update({
                "ok": False,
                "source": "fault",
                "timestamp": time.time(),
                "message": reason,
            })

    def _on_get_distance(self, request, response):
        depth = None
        with self._latest_lock:
            if self._latest_depth is not None:
                depth = self._latest_depth.copy()
        if depth is None:
            response.success = False
            response.message = self._fault_reason or "no depth frame available"
            return response
        distance = self._distance_for_roi(
            depth,
            int(request.roi_x),
            int(request.roi_y),
            int(request.roi_width),
            int(request.roi_height),
        )
        response.success = distance["distance_m"] > 0.0
        response.message = "ok" if response.success else "no valid depth in ROI"
        response.distance_m = float(distance["distance_m"])
        response.confidence = float(distance["confidence"])
        response.valid_pixels = int(distance["valid_pixels"])
        response.total_pixels = int(distance["total_pixels"])
        return response

    def _on_get_status(self, request, response):
        response.node_name = "stereo_depth"
        response.uptime_s = float(time.monotonic() - self._start_time)
        if self._fault_reason:
            response.status = STATUS_DEGRADED
            response.status_text = self._fault_reason
        else:
            response.status = STATUS_NORMAL
            response.status_text = "normal"
        return response

    def _status_tick(self) -> None:
        msg = VisionStatus()
        msg.active_model = "opencv_stereo_sgbm"
        msg.avg_inference_ms = float(self._last_compute_ms)
        msg.fps = float(self.get_parameter("target_fps").value)
        if self._fault_reason:
            msg.status = STATUS_DEGRADED
            msg.status_text = self._fault_reason
        else:
            msg.status = STATUS_NORMAL
            msg.status_text = "normal"
        self._status_pub.publish(msg)

    def _heartbeat_tick(self) -> None:
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = "stereo_depth"
        msg.lifecycle_state = LIFECYCLE_ACTIVE
        msg.error_code = ERROR_RUNTIME if self._fault_reason else ERROR_NONE
        self._heartbeat_pub.publish(msg)

    def _start_http_server(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                if self.path == "/health":
                    self._json({"ok": True, "node": "stereo_depth"})
                elif self.path == "/latest":
                    with node._latest_lock:
                        payload = dict(node._latest)
                    self._json(payload)
                elif self.path == "/depth.pgm":
                    self._depth_pgm()
                elif self.path == "/ws":
                    self._websocket()
                else:
                    self.send_error(404)

            def _json(self, payload):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _depth_pgm(self):
                with node._latest_lock:
                    depth = None if node._latest_depth is None else node._latest_depth.copy()
                if depth is None:
                    self.send_error(503, "no depth frame")
                    return
                max_depth = max(float(node.get_parameter("depth_max_m").value), 0.1)
                img = np.clip((depth / max_depth) * 255.0, 0, 255).astype(np.uint8)
                body = f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii") + img.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/x-portable-graymap")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _websocket(self):
                key = self.headers.get("Sec-WebSocket-Key", "")
                if not key:
                    self.send_error(400, "missing websocket key")
                    return
                accept = base64.b64encode(hashlib.sha1(
                    (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
                ).digest()).decode("ascii")
                self.send_response(101)
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                try:
                    while True:
                        with node._latest_lock:
                            payload = json.dumps(dict(node._latest), ensure_ascii=False).encode("utf-8")
                        self.wfile.write(self._ws_text_frame(payload))
                        self.wfile.flush()
                        time.sleep(1.0)
                except Exception:
                    return

            @staticmethod
            def _ws_text_frame(payload: bytes) -> bytes:
                length = len(payload)
                if length < 126:
                    return struct.pack("!BB", 0x81, length) + payload
                if length < 65536:
                    return struct.pack("!BBH", 0x81, 126, length) + payload
                return struct.pack("!BBQ", 0x81, 127, length) + payload

        host = self.get_parameter("http_host").value
        port = int(self.get_parameter("http_port").value)
        self._http_server = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(f"stereo_depth HTTP debug API listening on {host}:{port}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._http_server is not None:
            node._http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
