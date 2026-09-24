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
# Conservative world-axis-aligned bounds of the robot collision geometry, in metres.
bool bounds_valid
float64 min_x
float64 max_x
float64 min_y
float64 max_y
# Root heading in the world frame, in radians.
bool heading_valid
float64 yaw
# Fraction of sampled horizontal escape directions blocked nearby by this robot.
float64 ball_blockage
# Sum of solved robot-ball contact force magnitudes in newtons for this physics step.
float64 ball_contact_force
