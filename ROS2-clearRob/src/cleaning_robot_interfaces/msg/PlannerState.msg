# Path planner state
std_msgs/Header  header
uint8            state               # 0=IDLE 1=PLANNING 2=READY 3=EXECUTING 4=REPLANNING 5=BLOCKED 6=COMPLETED
string           state_text
