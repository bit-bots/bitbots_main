# Aggregated active collision contacts between a pair of simulator robot indices.
uint32 robot_a
uint32 robot_b
# Sum of contact force magnitudes in newtons.
float64 force
# Greatest contact-point approach speed toward the other robot, in m/s.
float64 approach_a
float64 approach_b
# World-frame location of the strongest contact in this pair during the step.
bool position_valid
geometry_msgs/Point position
