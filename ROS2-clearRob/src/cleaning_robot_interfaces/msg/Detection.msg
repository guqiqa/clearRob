# Single detection target
# Coordinate system: x1,y1,x2,y2 are pixel coordinates (relative to original image resolution, top-left origin)
string   class_name          # Class name
float32  confidence          # Confidence 0.0~1.0
uint16   class_id            # Class ID 0~7
float32  x1                  # BBox top-left x (pixels)
float32  y1                  # BBox top-left y (pixels)
float32  x2                  # BBox bottom-right x (pixels)
float32  y2                  # BBox bottom-right y (pixels)
float32  pixel_area          # Pixel area = (x2-x1)*(y2-y1)
