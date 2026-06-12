"""
vision_detector — YOLOv8-based garbage detection ROS2 package (M1: basic detection).

Subscribes to sensor/camera/image_raw, runs YOLOv8 inference,
and publishes detection results to vision/detect/list.
"""

__version__ = "0.1.0"
