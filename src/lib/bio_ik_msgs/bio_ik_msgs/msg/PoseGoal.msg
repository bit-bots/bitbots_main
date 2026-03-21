# The name of this goal's end effector link
string link_name

# Weight of the current goal
# Goals with a higher weight will have a stronger influence than goals with a smaller weight
float64 weight

# The goal pose into which the end effector link shall be brought
geometry_msgs/Pose pose

# How rotational errors will be weighted in relation to positional errors
float64 rotation_scale
