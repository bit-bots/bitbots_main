# The name of the joint variable
string variable_name

# The desired position value of the joint variable
float64 variable_position

# Weight of the current goal
# Goals with a higher weight will have a stronger influence than goals with a smaller weight
float64 weight

# If set to false, the goal will be primary goal (like eg. position goals)
# If set to true, this goal will always have lower priority than the primary goals
bool secondary
