std_msgs/Header header

# Mark this message so it has prio
bool from_hcm

string[] joint_names
float64[] positions

# set to -1.0 for maximum
float64[] velocities
float64[] accelerations
float64[] max_currents
