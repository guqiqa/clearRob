# Raw RC signal from device abstraction layer
std_msgs/Header  header
uint8[16]        buttons             # Pressed button numbers
float32[8]       axes                # Axis values -1.0~1.0
uint8            signal_strength     # 0~100%
bool             connected
