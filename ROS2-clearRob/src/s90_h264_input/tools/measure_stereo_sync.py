#!/usr/bin/env python3
"""Measure ROS2 stereo image rates and timestamp pairing quality.

This intentionally does not decode video or alter the camera pipeline. It is a
read-only acceptance probe for the software H264 bridge and can run on the
target board after sourcing the ROS2 workspace.
"""

import argparse
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image


def stamp_ns(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def percentile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * p / 100.0
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return float(ordered[low])
    return float(ordered[low] + (ordered[high] - ordered[low]) * (index - low))


class StereoSyncProbe(Node):
    def __init__(self, left_topic: str, right_topic: str, max_delta_ms: float):
        super().__init__("stereo_sync_probe")
        qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.left_queue = deque(maxlen=30)
        self.right_queue = deque(maxlen=30)
        self.max_delta_ns = int(max_delta_ms * 1_000_000.0)
        self.started = time.monotonic()
        self.count = {"left": 0, "right": 0, "pairs": 0, "pairs_within_limit": 0}
        self.last_stamp = {"left": None, "right": None}
        self.regressions = {"left": 0, "right": 0}
        self.deltas_ms = []
        self.dimensions = {"left": None, "right": None}
        self.encodings = {"left": None, "right": None}
        self.create_subscription(Image, left_topic, lambda msg: self.on_image("left", msg), qos)
        self.create_subscription(Image, right_topic, lambda msg: self.on_image("right", msg), qos)

    def on_image(self, eye: str, msg: Image):
        self.count[eye] += 1
        self.dimensions[eye] = [int(msg.width), int(msg.height)]
        self.encodings[eye] = msg.encoding
        stamp = stamp_ns(msg)
        previous = self.last_stamp[eye]
        if previous is not None and stamp <= previous:
            self.regressions[eye] += 1
        self.last_stamp[eye] = stamp
        queue = self.left_queue if eye == "left" else self.right_queue
        queue.append((stamp, msg))
        self.try_pair()

    def try_pair(self):
        # Pair each frame with the nearest still-unpaired opposite-eye frame.
        while self.left_queue and self.right_queue:
            left_stamp = self.left_queue[0][0]
            right_stamp = self.right_queue[0][0]
            if left_stamp <= right_stamp:
                source = self.left_queue.popleft()
                candidates = list(self.right_queue)
                index = min(range(len(candidates)), key=lambda i: abs(candidates[i][0] - source[0]))
                target = candidates[index]
                del self.right_queue[index]
            else:
                source = self.right_queue.popleft()
                candidates = list(self.left_queue)
                index = min(range(len(candidates)), key=lambda i: abs(candidates[i][0] - source[0]))
                target = candidates[index]
                del self.left_queue[index]
            self.count["pairs"] += 1
            delta_ms = abs(source[0] - target[0]) / 1_000_000.0
            self.deltas_ms.append(delta_ms)
            if delta_ms <= self.max_delta_ns / 1_000_000.0:
                self.count["pairs_within_limit"] += 1

    def report(self):
        elapsed = max(time.monotonic() - self.started, 1e-6)
        left_fps = self.count["left"] / elapsed
        right_fps = self.count["right"] / elapsed
        pair_rate = self.count["pairs_within_limit"] / max(min(self.count["left"], self.count["right"]), 1)
        return {
            "duration_s": round(elapsed, 3),
            "left": {"frames": self.count["left"], "fps": round(left_fps, 3), "size": self.dimensions["left"], "encoding": self.encodings["left"], "timestamp_regressions": self.regressions["left"]},
            "right": {"frames": self.count["right"], "fps": round(right_fps, 3), "size": self.dimensions["right"], "encoding": self.encodings["right"], "timestamp_regressions": self.regressions["right"]},
            "pairs": self.count["pairs"],
            "pairs_within_limit": self.count["pairs_within_limit"],
            "pair_rate": round(pair_rate, 5),
            "delta_ms": {
                "count": len(self.deltas_ms),
                "median": percentile(self.deltas_ms, 50),
                "p95": percentile(self.deltas_ms, 95),
                "p99": percentile(self.deltas_ms, 99),
                "max": max(self.deltas_ms) if self.deltas_ms else None,
                "within_limit": sum(d <= self.max_delta_ns / 1_000_000.0 for d in self.deltas_ms),
                "limit_ms": self.max_delta_ns / 1_000_000.0,
            },
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--left-topic", default="camera_native/left/image_raw")
    parser.add_argument("--right-topic", default="camera_native/right/image_raw")
    parser.add_argument("--max-delta-ms", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    probe = StereoSyncProbe(args.left_topic, args.right_topic, args.max_delta_ms)
    deadline = time.monotonic() + max(args.duration_s, 0.1)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(json.dumps(probe.report(), ensure_ascii=False, indent=2))
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
