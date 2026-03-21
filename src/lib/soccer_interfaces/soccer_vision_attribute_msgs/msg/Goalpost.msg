uint8 SIDE_UNKNOWN=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

uint8 TEAM_UNKNOWN=0
uint8 TEAM_OWN=1
uint8 TEAM_OPPONENT=2

# Whether the post is a left or right post, when looking INTO the goal.
# Value can be SIDE_UNKNOWN, SIDE_LEFT or SIDE_RIGHT
uint8 side

# Which team's goal, the goalpost is part of. A team's goal is the one
# they have to defend. This can be useful if the two goals have different colors,
# and you know whether you're looking at your own or opponent's goal.
# Value can be TEAM_UNKNOWN, TEAM_OWN or TEAM_OPPONENT
uint8 team
