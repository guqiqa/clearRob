# Node heartbeat (1Hz, all modules)
std_msgs/Header  header
string           node_name
uint8            lifecycle_state     # 1=Unconfigured 2=Inactive 3=Active 4=Finalized
uint8            error_code          # 0=normal 1=init_failed 2=runtime_error
