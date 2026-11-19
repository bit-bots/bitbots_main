# Snapshot published after a completed physics step. Positions are in meters.
# Header time and state refer to the contact solve before integration, after teleports.
std_msgs/Header header
# Identifies the completed step whose contacts and forces are included.
uint64 step_number
bool ball_present
geometry_msgs/Point ball_position
SimulationRobotState[] robots
# Discontinuities applied before this step, for suppressing artificial ball touches.
uint32[] teleported_robots
bool ball_teleported
SimulationRobotContact[] robot_contacts
