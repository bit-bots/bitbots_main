std_msgs/Header header

float64 engine_time

# some poses useful for debugging, for example in plotjuggler
# everything is relative to the support foot
# the poses are the following:
# - poses of the engine goals, derived from the splines
# - poses after stabilizing the engine goals, still in cartesian space
# - poses computed by running FK on the IK results
# - the translational difference between the IK input and IK output (i.e. stabilized_goal and ik_result)

geometry_msgs/Transform trunk_pose_goal
geometry_msgs/Transform trunk_pose_stabilized_goal
geometry_msgs/Transform trunk_pose_ik_result
geometry_msgs/Vector3 trunk_position_ik_offset

geometry_msgs/Transform flying_foot_pose_goal
geometry_msgs/Transform flying_foot_pose_stabilized_goal
geometry_msgs/Transform flying_foot_pose_ik_result
geometry_msgs/Vector3 flying_foot_position_ik_offset
