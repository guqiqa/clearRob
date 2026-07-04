#!/usr/bin/env python3
"""
yolo_detector.py — ONNX Runtime YOLO inference backend.

Loads a YOLO11 ONNX model and runs object detection on RGB images.
Supports models exported via ultralytics (standard detect format).

Usage:
    detector = YOLODetector("path/to/model.onnx", conf=0.25, iou=0.45)
    detections = detector.detect(rgb_image)  # numpy (H, W, 3) uint8
    # Returns list of dicts: {class_id, class_name, confidence, xmin, ymin, xmax, ymax}
"""

from __future__ import annotations

import logging
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_logger = logging.getLogger("yolo_detector")


# ---------------------------------------------------------------------------
# Pre/Post-processing
# ---------------------------------------------------------------------------

def _letterbox(
    img: np.ndarray,
    new_shape: Tuple[int, int] = (640, 640),
    color: Tuple[int, int, int] = (114, 114, 114),
) -> Tuple[np.ndarray, float, float, float]:
    """Resize with padding, preserving aspect ratio. Returns (image, ratio, dw, dh)."""
    h0, w0 = img.shape[:2]
    w, h = new_shape
    r = min(w / w0, h / h0)
    new_w, new_h = int(w0 * r), int(h0 * r)
    dw = (w - new_w) / 2
    dh = (h - new_h) / 2

    if (w0, h0) != (new_w, new_h):
        import cv2
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, dw, dh


def _nms(
    boxes: np.ndarray, scores: np.ndarray, iou_threshold: float
) -> np.ndarray:
    """Non-maximum suppression. Returns indices to keep."""
    if len(boxes) == 0:
        return np.array([], dtype=np.int64)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)

    order = scores.argsort()[::-1]
    keep: List[int] = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-16)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return np.array(keep, dtype=np.int64)


# ---------------------------------------------------------------------------
# Detection result
# ---------------------------------------------------------------------------

class Detection:
    """Single detection result."""
    __slots__ = ("class_id", "class_name", "confidence", "xmin", "ymin", "xmax", "ymax")

    def __init__(
        self,
        class_id: int,
        class_name: str,
        confidence: float,
        xmin: int,
        ymin: int,
        xmax: int,
        ymax: int,
    ) -> None:
        self.class_id = class_id
        self.class_name = class_name
        self.confidence = confidence
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

    def __repr__(self) -> str:
        return (
            f"Detection({self.class_name}, conf={self.confidence:.2f}, "
            f"box=({self.xmin},{self.ymin},{self.xmax},{self.ymax}))"
        )


# ---------------------------------------------------------------------------
# YOLODetector
# ---------------------------------------------------------------------------

class YOLODetector:
    """ONNX Runtime YOLO detector.

    Args:
        model_path: Path to the ONNX model file.
        class_names: List of class name strings, indexed by class_id.
        conf_threshold: Minimum confidence to keep a detection.
        iou_threshold: NMS IoU threshold.
        input_size: Model input (width, height). Default (640, 640).
    """

    def __init__(
        self,
        model_path: str,
        class_names: Optional[List[str]] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        input_size: Tuple[int, int] = (640, 640),
    ) -> None:
        self._model_path = model_path
        self._conf = conf_threshold
        self._iou = iou_threshold
        self._input_size = input_size
        self._class_names = class_names or [
            "recyclable", "kitchen_waste", "hazardous", "other_waste",
            "green_waste", "road_obstacle", "pedestrian_pet", "stain",
        ]
        self._nc = len(self._class_names)

        self._session = None
        self._input_name = ""
        self._input_shape: Tuple[int, int, int, int] = (1, 3, *input_size)  # NCHW
        self._loaded = False

    # ------------------------------------------------------------------
    # Load / unload
    # ------------------------------------------------------------------

    def load(self) -> bool:
        """Load the ONNX model. Returns True on success."""
        try:
            import onnxruntime as ort
        except ImportError:
            _logger.error("onnxruntime not installed. pip install onnxruntime")
            return False

        providers = ort.get_available_providers()
        _logger.info(f"ONNX Runtime providers: {providers}")

        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        try:
            self._session = ort.InferenceSession(
                self._model_path,
                sess_options=sess_options,
                providers=providers,
            )
        except Exception as exc:
            _logger.error(f"Failed to load ONNX model: {exc}")
            return False

        # Inspect inputs / outputs
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        self._input_name = inputs[0].name
        self._input_shape = tuple(inputs[0].shape)  # type: ignore[arg-type]
        _logger.info(
            f"Model loaded: input={self._input_name} {self._input_shape}, "
            f"output={outputs[0].name} {outputs[0].shape}"
        )
        self._loaded = True
        return True

    def unload(self) -> None:
        """Release the ONNX session."""
        self._session = None
        self._loaded = False

    @property
    def loaded(self) -> bool:
        return self._loaded

    @property
    def model_path(self) -> str:
        return self._model_path

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def detect(self, image: np.ndarray) -> Tuple[List[Detection], float]:
        """Run detection on an RGB image (H, W, 3) uint8.

        Returns:
            (detections, inference_time_ms)
        """
        if not self._loaded or self._session is None:
            _logger.warning("Detector not loaded, returning empty result")
            return [], 0.0

        t0 = time.perf_counter()

        # --- Preprocess ---
        input_tensor, ratio, dw, dh = self._preprocess(image)

        # --- Inference ---
        outputs = self._session.run(None, {self._input_name: input_tensor})
        raw = outputs[0]  # shape: [1, 4+nc, N] or [1, N, 4+nc]

        # --- Postprocess ---
        detections = self._postprocess(raw, ratio, dw, dh, image.shape[1], image.shape[0])

        t1 = time.perf_counter()
        return detections, (t1 - t0) * 1000.0

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, float, float, float]:
        """Letterbox + normalize → NCHW tensor."""
        img, ratio, dw, dh = _letterbox(image, self._input_size)
        # HWC → CHW, uint8 → float32, 0..255 → 0..1
        tensor = img.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32) / 255.0
        return tensor, ratio, dw, dh

    def _postprocess(
        self,
        raw: np.ndarray,
        ratio: float,
        dw: float,
        dh: float,
        orig_w: int,
        orig_h: int,
    ) -> List[Detection]:
        """Parse YOLO output → Detection list."""
        # Handle both [1, 4+nc, N] and [1, N, 4+nc] formats
        if raw.ndim == 3 and raw.shape[1] == 4 + self._nc:
            # [1, 4+nc, N] — standard YOLO ONNX export format
            raw = raw[0].T  # → [N, 4+nc]
        elif raw.ndim == 3 and raw.shape[2] == 4 + self._nc:
            raw = raw[0]  # → [N, 4+nc]
        elif raw.ndim == 2:
            pass  # already [N, 4+nc]
        else:
            _logger.warning(f"Unexpected output shape: {raw.shape}, nc={self._nc}")
            return []

        boxes_raw = raw[:, :4]      # cx, cy, w, h (normalized 0..1)
        scores_raw = raw[:, 4:]     # class scores

        # Decode boxes: cx,cy,w,h → x1,y1,x2,y2
        cx, cy, bw, bh = boxes_raw[:, 0], boxes_raw[:, 1], boxes_raw[:, 2], boxes_raw[:, 3]
        x1 = (cx - bw / 2 - dw) / ratio
        y1 = (cy - bh / 2 - dh) / ratio
        x2 = (cx + bw / 2 - dw) / ratio
        y2 = (cy + bh / 2 - dh) / ratio

        # Class assignment
        if scores_raw.shape[1] > 1:
            class_ids = scores_raw.argmax(axis=1)
            confidences = scores_raw.max(axis=1)
        else:
            # Single class (unlikely for our 8-class model, but handle gracefully)
            class_ids = np.zeros(len(scores_raw), dtype=np.int64)
            confidences = scores_raw[:, 0]

        # Filter by confidence
        mask = confidences >= self._conf
        if not mask.any():
            return []

        x1, y1, x2, y2 = x1[mask], y1[mask], x2[mask], y2[mask]
        class_ids = class_ids[mask]
        confidences = confidences[mask]

        # Clamp to image bounds
        x1 = np.maximum(0, x1)
        y1 = np.maximum(0, y1)
        x2 = np.minimum(orig_w, x2)
        y2 = np.minimum(orig_h, y2)

        # Filter invalid boxes
        valid = (x2 > x1) & (y2 > y1)
        if not valid.any():
            return []
        x1, y1, x2, y2 = x1[valid], y1[valid], x2[valid], y2[valid]
        class_ids = class_ids[valid]
        confidences = confidences[valid]

        # NMS
        nms_boxes = np.stack([x1, y1, x2, y2], axis=1)
        keep = _nms(nms_boxes, confidences, self._iou)

        # Build results
        detections: List[Detection] = []
        for idx in keep:
            cid = int(class_ids[idx])
            detections.append(Detection(
                class_id=cid,
                class_name=self._class_names[cid] if cid < len(self._class_names) else f"class_{cid}",
                confidence=float(confidences[idx]),
                xmin=int(round(x1[idx])),
                ymin=int(round(y1[idx])),
                xmax=int(round(x2[idx])),
                ymax=int(round(y2[idx])),
            ))
        return detections
