# This message is used for communicating between the body behavior and the head behavior.
# The body tells the head by this message what it shall do.

# Track the last seen ball position
uint8 TRACK_BALL=0
# Look generally for all features on the field (ball, goals, corners, center point)
uint8 SEARCH_FIELD_FEATURES=1
# Simply look directly forward
uint8 LOOK_FORWARD=2
#Don't move the head
uint8 DONT_MOVE=3
# Ball Mode adapted for Penalty Kick
uint8 SEARCH_BALL_PENALTY = 4
# Do a pattern which only looks in front of the robot
uint8 SEARCH_FRONT = 5


uint8 head_mode
