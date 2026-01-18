#
# This message is only used for debugging the dynup engine.
#

std_msgs/Header header

float64 time
bool stabilization_active
int32 state_number

geometry_msgs/Pose l_arm_pose
geometry_msgs/Pose r_arm_pose
geometry_msgs/Pose l_arm_pose_from_shoulder
geometry_msgs/Pose r_arm_pose_from_shoulder

geometry_msgs/Pose l_leg_pose
geometry_msgs/Pose r_leg_pose

float64 foot_roll
float64 foot_pitch
float64 foot_yaw