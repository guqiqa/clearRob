# Normalized control command from control_gateway
std_msgs/Header  header
uint8            command             # Command code 0~9
string           command_name        # "START"/"PAUSE"/"RESUME"/"STOP"/"RETURN"/"MANUAL"/"SET_AREA"/"SET_MODE"/"ESTOP"
uint8            source              # 0=Remote 1=App 2=Voice
float32[10]      params              # Command parameters
