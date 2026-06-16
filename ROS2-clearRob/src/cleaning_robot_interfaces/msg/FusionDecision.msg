# Obstacle avoidance decision
std_msgs/Header  header
uint8            action              # 0=PASS 1=DETOUR 2=WAIT 3=SLOW
uint32           target_id           # Associated obstacle ID
float32          detour_offset       # Detour offset (m)
float32          wait_duration       # Wait duration (s)
float32          slow_ratio          # Slow ratio 0.0~1.0
