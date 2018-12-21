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

# output of the walk engine, in frame of the support foot
geometry_msgs/Pose engine_trunk_goal
geometry_msgs/Pose engine_fly_goal


# the following is computed to trunk frame
# position of the goals, the resulting positions computed by running FK on the IK results and the actual position from joint states
geometry_msgs/Pose left_foot_goal
geometry_msgs/Pose left_foot_ik_result
geometry_msgs/Pose left_foot_position
geometry_msgs/Pose right_foot_goal
geometry_msgs/Pose right_foot_ik_result
geometry_msgs/Pose right_foot_position

# same values as above but grouped for fly and support foot
geometry_msgs/Pose fly_foot_goal
geometry_msgs/Pose fly_foot_ik_result
geometry_msgs/Pose fly_foot_position
geometry_msgs/Pose support_foot_goal
geometry_msgs/Pose support_foot_ik_result
geometry_msgs/Pose support_foot_position

# offset between goal and computed position
geometry_msgs/Vector3 left_foot_ik_offset
geometry_msgs/Vector3 right_foot_ik_offset
geometry_msgs/Vector3 fly_foot_ik_offset
geometry_msgs/Vector3 support_foot_ik_offset

# offset between goal and actual position taken from joint states
geometry_msgs/Vector3 left_foot_actual_offset
geometry_msgs/Vector3 right_foot_actual_offset
geometry_msgs/Vector3 fly_foot_actual_offset
geometry_msgs/Vector3 support_foot_actual_offset