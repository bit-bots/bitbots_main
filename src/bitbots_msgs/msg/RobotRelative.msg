# An another robot relative to the robot

# robot without known color
uint8 ROBOT_UNDEFINED = 1
uint8 ROBOT_RED = 2
uint8 ROBOT_BLUE = 3

uint8 type

# The number of the robot, if it is a robot and if it can be read. Put in 0 if not known
uint8 player_number

# Position (in meters)
PoseWithCertainty pose

# Educated guess of the width and height (in meters)
float32 width
float32 height
