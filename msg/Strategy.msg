# This message provides information about the current strategy of the robot to the team communication so that it can be
# shared with other team robots

# Which role the robot has currently
uint8 ROLE_UNDEFINED=0
uint8 ROLE_IDLING=1
uint8 ROLE_OTHER=2
uint8 ROLE_STRIKER=3
uint8 ROLE_SUPPORTER=4
uint8 ROLE_DEFENDER=5
uint8 ROLE_GOALIE=6
uint8 role

# The current action of the robot
uint8 ACTION_UNDEFINED=0
uint8 ACTION_POSITIONING=1
uint8 ACTION_GOING_TO_BALL=2
uint8 ACTION_TRYING_TO_SCORE=3
uint8 ACTION_WAITING=4
uint8 ACTION_KICKING=5
uint8 ACTION_SEARCHING=6
uint8 ACTION_LOCALIZING=7
uint8 action

# Offensive strategy
uint8 SIDE_UNDEFINED=0
uint8 SIDE_LEFT=1
uint8 SIDE_MIDDLE=2
uint8 SIDE_RIGHT=3
uint8 offensive_side
