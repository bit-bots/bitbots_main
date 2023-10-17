# This message is used for communicating between the body behavior and the head behavior.
# The body tells the head by this message what it shall do.

# Search for Ball and track it if found
uint8 BALL_MODE=0
# Search for goal posts, mainly to locate the robot on the field
uint8 POST_MODE=1
# Track ball and goal by constantly switching between both
uint8 BALL_GOAL_TRACKING=2
# Look generally for all features on the field (ball, goals, corners, center point)
uint8 FIELD_FEATURES=3
# Look for features outside of the field (perimeter advertising, walls, etc).
# Can be used for localization using features on the ceiling.
uint8 NON_FIELD_FEATURES=4
# Simply look down to its feet.
uint8 LOOK_DOWN=5
# Simply look directly forward
uint8 LOOK_FORWARD=7
#Don't move the head
uint8 DONT_MOVE=8
# Look to the ceiling, for example for visual compass
uint8 LOOK_UP=9
# Record ground truth for the visual compass
uint8 RECORD_VISUAL_COMPASS=10
# Ball Mode adapted for Penalty Kick
uint8 BALL_MODE_PENALTY = 11
# Look for visual compass features
uint8 VISUAL_COMPASS_FEATURES = 12
# Do a pattern which only looks in front of the robot
uint8 LOOK_FRONT = 13


uint8 head_mode
