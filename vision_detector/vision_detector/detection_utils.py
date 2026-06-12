"""
Detection data structures and utility functions.

Defines the canonical Detection dataclass used throughout the vision_detector
package, plus helper functions for filtering and format conversion.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass(slots=True)
class Detection:
    """Single detection result — platform-agnostic, not ROS-dependent."""

    class_name: str
    confidence: float
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    pixel_area: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    def to_normalised(self, img_w: int, img_h: int) -> dict:
        """Return a dict with bounding-box values normalised to [0, 1]."""
        return {
            "x1": self.x1 / img_w,
            "y1": self.y1 / img_h,
            "x2": self.x2 / img_w,
            "y2": self.y2 / img_h,
        }


def filter_by_confidence(
    detections: List[Detection], threshold: float
) -> List[Detection]:
    """Return detections whose confidence >= *threshold*."""
    return [d for d in detections if d.confidence >= threshold]


def filter_by_area(
    detections: List[Detection], min_area: float = 0.0, max_area: float = float("inf")
) -> List[Detection]:
    """Return detections whose pixel_area is in [*min_area*, *max_area*]."""
    return [d for d in detections if min_area <= d.pixel_area <= max_area]


def compute_area_ratio(
    detections: List[Detection], img_w: int, img_h: int
) -> float:
    """Fraction of total image area covered by all detection boxes (0.0-1.0)."""
    if not detections:
        return 0.0
    total_area = sum(d.pixel_area for d in detections)
    return min(total_area / (img_w * img_h), 1.0)
