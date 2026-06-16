# Target waypoint sequence from path_planner
std_msgs/Header  header
geometry_msgs/PoseStamped[]  waypoints
float32          total_length        # Total path length (m)
uint32           sequence            # Path sequence number
