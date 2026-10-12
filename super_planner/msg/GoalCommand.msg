uint8 SET_GOAL=1
uint8 CANCEL_GOAL=2

std_msgs/Header header
uint8 command
uint64 goal_id
geometry_msgs/Pose goal
string reason
