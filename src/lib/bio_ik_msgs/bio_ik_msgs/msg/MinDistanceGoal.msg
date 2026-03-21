# The name of this goal's end effector link
string link_name

# Weight of the current goal
# Goals with a higher weight will have a stronger influence than goals with a smaller weight
float64 weight

# The distance will be computed from the origin of the end effector link to the target point
geometry_msgs/Point target

# Minimum allowed distance
# Forces the end effector link to be at least the specified distance away from the target
float64 distance
