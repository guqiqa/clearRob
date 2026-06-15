"""
YOLO detector wrapper — loads a YOLOv8 model and runs inference on RGB images.

Supports: YOLOv8 / YOLOv8s / YOLOv8m / YOLOv8l, with FP16 and device selection.
Provides a clean interface decoupled from ROS2 so it can be tested independently.
"""

from __future__ import annotations

import logging
import time
from typing import List, Optional, Tuple

import numpy as np
from ultralytics import YOLO

from .detection_utils import Detection, filter_by_confidence

logger = logging.getLogger("vision_detector.yolo")


class YOLODetector:
    """Wraps an Ultralytics YOLO model for single-frame inference.

    Call ``load()`` once to initialise the model, then ``detect(image)`` on
    each incoming BGR/RGB numpy frame.
    """

    # ------------------------------------------------------------------
    def __init__(self) -> None:
        self._model: Optional[YOLO] = None
        self._model_path: str = ""
        self._device: str = "cuda:0"
        self._conf_threshold: float = 0.45
        self._iou_threshold: float = 0.45
        self._input_size: Tuple[int, int] = (640, 640)
        self._half: bool = True
        self._class_names: dict[int, str] = {}

    # ---- public API --------------------------------------------------

    def load(
        self,
        model_path: str,
        *,
        device: str = "cuda:0",
        conf_threshold: float = 0.45,
        iou_threshold: float = 0.45,
        input_width: int = 640,
        input_height: int = 640,
        half_precision: bool = True,
    ) -> bool:
        """Load a YOLO model from *model_path*.

        Returns True on success, False on failure.
        """
        self._model_path = model_path
        self._device = device
        self._conf_threshold = conf_threshold
        self._iou_threshold = iou_threshold
        self._input_size = (input_width, input_height)
        self._half = half_precision and (device != "cpu")

        try:
            logger.info("Loading YOLO model from: %s (device=%s)", model_path, device)
            self._model = YOLO(model_path)
            if self._half:
                self._model.model.half()
            self._model.to(device)

            # warm-up inference
            dummy = np.random.randint(
                0, 255, (input_height, input_width, 3), dtype=np.uint8
            )
            _ = self._model.predict(
                dummy,
                conf=conf_threshold,
                iou=iou_threshold,
                imgsz=self._input_size,
                verbose=False,
            )

            self._class_names = self._model.names or {}
            logger.info(
                "Model loaded successfully. Classes: %d, device: %s",
                len(self._class_names),
                device,
            )
            return True

        except Exception:
            logger.exception("Failed to load YOLO model from %s", model_path)
            self._model = None
            return False

    def detect(self, image: np.ndarray) -> Tuple[List[Detection], float]:
        """Run detection on a single BGR/RGB image.

        Parameters
        ----------
        image : np.ndarray
            Input image (H, W, 3), uint8, BGR or RGB.

        Returns
        -------
        (detections, inference_ms) : Tuple[List[Detection], float]
            List of detections and the inference wall-time in milliseconds.
            Returns empty list if model is not loaded.
        """
        if self._model is None:
            return [], 0.0

        t0 = time.perf_counter()
        results = self._model.predict(
            image,
            conf=self._conf_threshold,
            iou=self._iou_threshold,
            imgsz=self._input_size,
            verbose=False,
        )
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        detections: List[Detection] = []

        for result in results:
            if result.boxes is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()      # (N, 4), pixel coords
            confs = result.boxes.conf.cpu().numpy()       # (N,)
            clss  = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

            for box, conf, cls_id in zip(boxes, confs, clss):
                det = Detection(
                    class_name=self._class_names.get(cls_id, f"class_{cls_id}"),
                    confidence=float(conf),
                    class_id=int(cls_id),
                    x1=float(box[0]),
                    y1=float(box[1]),
                    x2=float(box[2]),
                    y2=float(box[3]),
                    pixel_area=float((box[2] - box[0]) * (box[3] - box[1])),
                )
                detections.append(det)

        return detections, elapsed_ms

    def unload(self) -> None:
        """Release the model and free GPU memory."""
        if self._model is not None:
            del self._model
            self._model = None
            logger.info("Model unloaded.")
            # Give GC/torch a chance to reclaim VRAM
            import gc
            gc.collect()

    # ---- properties --------------------------------------------------

    @property
    def is_loaded(self) -> bool:
        return self._model is not None

    @property
    def model_path(self) -> str:
        return self._model_path

    @property
    def device(self) -> str:
        return self._device

    @property
    def conf_threshold(self) -> float:
        return self._conf_threshold

    @property
    def input_size(self) -> Tuple[int, int]:
        return self._input_size
