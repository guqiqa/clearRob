# Mode switch command from master_controller to all modules
std_msgs/Header  header
uint8            mode        # 0=CRUISE 1=CLEAN 2=RETURN
string           mode_name   # "CRUISE" / "CLEAN" / "RETURN"
