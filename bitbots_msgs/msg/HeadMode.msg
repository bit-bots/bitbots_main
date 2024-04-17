# This message is used for communicating between the body behavior and the head behavior.
# The body tells the head by this message what it shall do.

# Search for Ball and track it if found
uint8 BALL_MODE=0 
# Look generally for all features on the field (ball, goals, corners, center point)
uint8 FIELD_FEATURES=1
# Simply look directly forward
uint8 LOOK_FORWARD=2
#Don't move the head
uint8 DONT_MOVE=3
# Ball Mode adapted for Penalty Kick
uint8 BALL_MODE_PENALTY = 4
# Do a pattern which only looks in front of the robot
uint8 LOOK_FRONT = 5


uint8 head_mode
