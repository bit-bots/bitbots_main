uint8 NUMBER_UNKNOWN=0

uint8 TEAM_UNKNOWN=0
uint8 TEAM_OWN=1
uint8 TEAM_OPPONENT=2

uint8 STATE_UNKNOWN=0
uint8 STATE_STANDING=1
uint8 STATE_FALLEN=2
uint8 STATE_KICKING=3
uint8 STATE_INACTIVE=4

uint8 FACING_UNKNOWN=0
uint8 FACING_THIS_WAY=1
uint8 FACING_AWAY=2
uint8 FACING_LEFT=3
uint8 FACING_RIGHT=4

# The number of the robot, if it can be read from the image.
# Values can range from NUMBER_UNKNOWN(0) up to 255.
uint8 player_number

# Which team's robot it is. 
# Value can be TEAM_UNKNOWN, TEAM_OWN or TEAM_OPPONENT
uint8 team

# State of the robot.
# Value can be one of the STATE_* constants defined in this msg
uint8 state

# An approximate direction the robot is facing in the image.
# Value can be one of the FACING_* constants defined in this msg
uint8 facing
