# Fused 3D target alert from fusion_engine
std_msgs/Header  header
string           target_class
float32          confidence
geometry_msgs/Point  position_3d     # Map frame
geometry_msgs/Vector3  dimensions    # Target dimensions (m)
uint8            alert_level         # 0=INFO 1=WARN 2=CRITICAL
string           alert_reason
