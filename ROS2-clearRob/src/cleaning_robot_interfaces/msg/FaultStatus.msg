# Fault status from motion_controller
std_msgs/Header  header
uint8            fault_code          # 0=OK 1=overcurrent 2=overheat 3=encoder 4=timeout 5=driver 6=estop 7=slope
string           fault_text
bool             requires_reset
