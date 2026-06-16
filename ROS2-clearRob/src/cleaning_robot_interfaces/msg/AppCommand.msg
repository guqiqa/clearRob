# Structured App command from device abstraction layer
std_msgs/Header  header
string           command             # "START"/"PAUSE"/"RESUME"/"STOP"/"RETURN"/"MANUAL"/"SET_AREA"/"SET_MODE"
string           params_json         # JSON params
string           request_id          # UUID request ID
