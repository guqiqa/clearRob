# Cleaning strategy command to cleaning_actuator
std_msgs/Header  header
uint8            strategy            # 0=COVER 1=SPOT 2=TRACK 3=IDLE
float32          suction_power       # Suction power 0.0~1.0
float32          water_amount        # Water amount 0.0~1.0
float32          brush_speed         # Brush speed 0.0~1.0
