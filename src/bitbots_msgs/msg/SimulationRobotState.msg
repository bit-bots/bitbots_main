# Robot root position in the simulation world frame, in meters.
uint32 robot_index
geometry_msgs/Point position
# Whether any collision geometry of this robot touches the ball during this step.
bool touching_ball
# Ground-truth motion, with linear speed in m/s and angular/joint speeds in rad/s.
bool motion_valid
float64 linear_speed
float64 angular_speed
float64 body_joint_speed
float64 head_joint_speed
# Torso local up projected onto world up, and height relative to the model's initial height.
float64 upright
float64 relative_height
