# One frame of detection results.
# Published by vision_detector on topic "vision/detect/list".
# Consumed by: fusion_engine, master_controller, cleaning_actuator.
#
# CRITICAL CONTRACT — timestamp alignment:
#   Header.stamp MUST be set to the original camera image timestamp
#   (sensor_msgs/Image.header.stamp), NOT the ROS node's current system time.
#   This is mandatory for fusion_engine's 2D→3D spatial association —
#   using the wrong timestamp will cause coordinate drift between the vision
#   detection frame and the LiDAR point cloud frame, breaking the fusion pipeline.

std_msgs/Header  header      # stamp = camera image timestamp; frame_id = camera optical frame
Detection[]      detections  # All detected objects in this frame (may be empty)
uint32           frame_seq   # Monotonically increasing frame sequence number (wraps at overflow)
float32          inference_ms # Inference latency for this frame (milliseconds)
