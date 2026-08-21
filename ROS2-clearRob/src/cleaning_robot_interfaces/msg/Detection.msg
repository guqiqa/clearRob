# Single detection target (2D bounding box in pixel coordinates)
#
# Contract: published by vision_detector as part of DetectionArray.
# Consumed primarily by fusion_engine for 2D→3D spatial association.
#
# Coordinate system: pixel coordinates relative to the original camera image
# resolution, with origin at the top-left corner of the image.

string   class_name          # Class name (must match the 8-class agreement: recyclable/kitchen_waste/hazardous/other_waste/green_waste/road_obstacle/pedestrian_pet/stain)
float32  confidence          # Confidence score 0.0 ~ 1.0
uint16   class_id            # Class ID 0 ~ 7

# Bounding box — pixel coordinates (int32, whole-pixel precision is sufficient for LiDAR fusion;
# sub-pixel YOLO outputs should be rounded to nearest integer).
int32    xmin                # BBox top-left x (pixels, 0-indexed)
int32    ymin                # BBox top-left y (pixels, 0-indexed)
int32    xmax                # BBox bottom-right x (pixels, exclusive)
int32    ymax                # BBox bottom-right y (pixels, exclusive)

float32  depth_m             # Optional true depth at bbox center/ROI (m), 0.0 = unavailable
