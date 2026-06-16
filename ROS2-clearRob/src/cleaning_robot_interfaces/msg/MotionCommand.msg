# Arbitrated motion command to motion_controller
std_msgs/Header  header
float32          linear_velocity     # m/s (positive=forward)
float32          angular_velocity    # rad/s (positive=CCW)
bool             estop               # Emergency stop flag
