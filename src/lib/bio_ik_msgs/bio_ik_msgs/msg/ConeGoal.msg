# The name of this goal's end effector link
string link_name

# Weight of the current goal
# Goals with a higher weight will have a stronger influence than goals with a smaller weight
float64 weight

geometry_msgs/Point position
float64 position_weight
geometry_msgs/Vector3 axis
geometry_msgs/Vector3 direction
float64 angle
