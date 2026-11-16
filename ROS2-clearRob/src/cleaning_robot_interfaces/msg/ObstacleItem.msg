# Single obstacle from lidar_perception
std_msgs/Header  header
uint32           obstacle_id
geometry_msgs/Point  position        # 3D position (LiDAR frame)
geometry_msgs/Vector3  dimensions    # x=length y=width z=height (m)
float32          distance            # Distance from robot (m)
float32          velocity            # Relative velocity (m/s)
uint8            classification      # 0=unknown 1=small(<0.1m) 2=medium(0.1~0.5m) 3=large(>0.5m)
