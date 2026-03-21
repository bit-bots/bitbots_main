# The name of this goal's end effector link
string link_name

# Weight of the current goal
# Goals with a higher weight will have a stronger influence than goals with a smaller weight
float64 weight

# The axis in the link's local coordinate frame which shall point towards the target
geometry_msgs/Vector3 axis

# The target point which the axis shall point at
geometry_msgs/Point target
