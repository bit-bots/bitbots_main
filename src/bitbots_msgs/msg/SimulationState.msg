# Snapshot published after a completed physics step. Positions are in meters.
std_msgs/Header header
uint64 step_number
bool ball_present
geometry_msgs/Point ball_position
SimulationRobotState[] robots
# Discontinuities applied before this step, for suppressing artificial ball touches.
uint32[] teleported_robots
bool ball_teleported
