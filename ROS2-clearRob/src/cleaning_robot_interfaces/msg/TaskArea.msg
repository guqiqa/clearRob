# Cleaning area task
std_msgs/Header  header
string           area_id             # Unique area identifier (UUID)
geometry_msgs/Polygon  boundary      # Area polygon boundary
uint32           cleaning_passes     # Number of coverage passes (default 1)
float32          overlap_ratio       # Path overlap ratio 0.0~0.3 (default 0.1)
