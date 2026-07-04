"""
粗粒度合并工具 — 将 20 细类预测结果合并为 7 粗类

用法:
  from coarse_merge import CoarseMerger
  merger = CoarseMerger("configs/fine_labels.yaml")
  merged_detections = merger.merge(fine_detections)
  # fine_detections: [(class_id, x1, y1, x2, y2, conf), ...]
  # merged: grouped by coarse class with max confidence
"""

import yaml
from pathlib import Path
from collections import defaultdict


class CoarseMerger:
    """将 20 细类合并为 7 粗类"""

    def __init__(self, config_path=None):
        path = Path(config_path) if config_path else Path(__file__).parent.parent / "configs" / "fine_labels.yaml"
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        self.fine_to_coarse = {int(k): int(v) for k, v in cfg["fine_to_coarse"].items()}
        self.coarse_names = {int(k): v for k, v in cfg["coarse_names"].items()}
        self.fine_names = {int(k): v for k, v in cfg["fine_classes"].items()}
        self.num_coarse = len(self.coarse_names)
        self.num_fine = len(self.fine_names)

    def merge_single(self, class_id, confidence, bbox):
        """合并单个检测结果: (fine_class_id, conf, [x1,y1,x2,y2]) → (coarse_class_id, conf, bbox)"""
        cid = self.fine_to_coarse.get(int(class_id), -1)
        return (cid, confidence, bbox) if cid >= 0 else None

    def merge(self, detections):
        """合并检测列表，同粗类的保留最高置信度

        detections: [(fine_class_id, x1, y1, x2, y2, confidence), ...]
        returns:    [(coarse_class_id, x1, y1, x2, y2, confidence), ...]
        """
        merged = []
        for det in detections:
            result = self.merge_single(det[0], det[5], det[1:5])
            if result:
                merged.append((result[0],) + result[2] + (result[1],))
        return self._nms_per_class(merged)

    def _nms_per_class(self, detections, iou_threshold=0.5):
        """类内 NMS 去重"""
        if not detections:
            return []
        by_class = defaultdict(list)
        for det in detections:
            by_class[det[0]].append(det)

        result = []
        for cls_id, dets in by_class.items():
            dets = sorted(dets, key=lambda d: d[5], reverse=True)
            kept = []
            for d in dets:
                if not any(self._iou(d[1:5], k[1:5]) > iou_threshold for k in kept):
                    kept.append(d)
            result.extend(kept)
        return sorted(result, key=lambda d: d[5], reverse=True)

    @staticmethod
    def _iou(boxA, boxB):
        xa = max(boxA[0], boxB[0]); ya = max(boxA[1], boxB[1])
        xb = min(boxA[2], boxB[2]); yb = min(boxA[3], boxB[3])
        inter = max(0, xb-xa) * max(0, yb-ya)
        areaA = (boxA[2]-boxA[0]) * (boxA[3]-boxA[1])
        areaB = (boxB[2]-boxB[0]) * (boxB[3]-boxB[1])
        return inter / (areaA + areaB - inter + 1e-8)

    def get_coarse_name(self, cid):
        return self.coarse_names.get(cid, "unknown")
