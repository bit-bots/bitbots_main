#
# This message is only for debugging / visualizing the walking
#

std_msgs/Header header

# times of the walk engine
float64 phase_time
float64 traj_time

# int instead of bool for easy plotting
uint8 is_left_support
uint8 is_double_support
std_msgs/String engine_state
int32 engine_state_number # same as engine_state only as number to use it in plottjuggler


# footstep
geometry_msgs/Vector3 footstep_last
geometry_msgs/Vector3 footstep_next


# output of the walk engine, in frame of the support foot
geometry_msgs/Pose engine_trunk_goal
geometry_msgs/Pose engine_trunk_goal_abs
geometry_msgs/Vector3 engine_trunk_axis

geometry_msgs/Pose engine_fly_goal
geometry_msgs/Vector3 engine_fly_axis
