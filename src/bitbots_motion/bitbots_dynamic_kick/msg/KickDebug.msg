std_msgs/Header header

uint8 INITIAL=0
uint8 MOVE_TRUNK=1
uint8 RAISE_FOOT=2
uint8 WINDUP=3
uint8 KICK=4
uint8 MOVE_BACK=5
uint8 LOWER_FOOT=6
uint8 MOVE_TRUNK_BACK=7
uint8 DONE=8

float64 engine_time
uint8 engine_phase

# some poses useful for debugging, for example in plotjuggler
# everything is relative to the support foot
# the poses are the following:
# - poses of the engine goals, derived from the splines
# - poses after stabilizing the engine goals, still in cartesian space
# - poses computed by running FK on the IK results
# - the translational difference between the IK input and IK output (i.e. stabilized_goal and ik_result)

geometry_msgs/Pose trunk_pose_goal
geometry_msgs/Pose trunk_pose_stabilized_goal
geometry_msgs/Pose trunk_pose_ik_result
geometry_msgs/Point trunk_position_ik_offset

geometry_msgs/Pose flying_foot_pose_goal
geometry_msgs/Pose flying_foot_pose_stabilized_goal
geometry_msgs/Pose flying_foot_pose_ik_result
geometry_msgs/Point flying_foot_position_ik_offset
