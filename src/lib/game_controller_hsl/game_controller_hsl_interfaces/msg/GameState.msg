# This message defines a GameState extracted  from the RoboCupGameControlData packet received by the game controller.
# While the game controller provides info about both teams and all players this message represents the
# state for the currently running team/player combination.
#
# For the packet defition for RoboCupGameControlData see:
# https://github.com/RoboCup-HumanoidSoccerLeague/GameController/blob/master/game_controller_msgs/headers/RoboCupGameControlData.h
#
# For additional documentation of the underlying meaning of fields have a look at:
# https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules

std_msgs/Header header

##### Game State
uint8 players_per_team

uint8 COMPETITION_TYPE_SMALL = 0
uint8 COMPETITION_TYPE_MIDDLE = 1
uint8 COMPETITION_TYPE_LARGE = 2
uint8 competition_type

uint8 GAME_PHASE_NORMAL = 0
uint8 GAME_PHASE_PENALTY_SHOOT_OUT = 1
uint8 GAME_PHASE_EXTRA_TIME = 2
uint8 GAME_PHASE_TIMEOUT = 3
uint8 game_phase

uint8 STATE_INITIAL = 0
uint8 STATE_READY = 1
uint8 STATE_SET = 2
uint8 STATE_PLAYING = 3
uint8 STATE_FINISHED = 4
uint8 main_state

uint8 SET_PLAY_NONE = 0
uint8 SET_PLAY_DIRECT_FREE_KICK = 1
uint8 SET_PLAY_INDIRECT_FREE_KICK = 2
uint8 SET_PLAY_PENALTY_KICK = 3
uint8 SET_PLAY_THROW_IN = 4
uint8 SET_PLAY_GOAL_KICK = 5
uint8 SET_PLAY_CORNER_KICK = 6
uint8 set_play

bool first_half
bool stopped
uint8 kicking_team
uint8 own_score
uint8 rival_score

# Seconds remaining for the game half
int16 seconds_remaining
# Seconds remaining for current game action (e.g. kickoff)
int16 secondary_time

##### TeamInfo

uint8 TEAM_BLUE = 0
uint8 TEAM_RED = 1
uint8 TEAM_YELLOW = 2
uint8 TEAM_BLACK = 3
uint8 TEAM_WHITE = 4
uint8 TEAM_GREEN = 5
uint8 TEAM_ORANGE = 6
uint8 TEAM_PURPLE = 7
uint8 TEAM_BROWN = 8
uint8 TEAM_GRAY = 9
uint8 own_player_color
uint8 own_goalie_color
uint8 rival_player_color
uint8 rival_goalie_color

# The number of the current penalty shot during penalty shootout
uint8 penalty_shot
# a binary pattern indicating the successful penalty shots (1 for successful, 0 for unsuccessful)
uint16 single_shots

# Number of remaining messages, that can be sent by the team
uint16 message_budget

# Extra fields to make decisions based on number of active or cautioned players.
# The indexes represent the players in order.
bool[] team_mates_with_penalty
bool[] team_mates_with_yellow_card
bool[] team_mates_with_red_card

##### RobotInfo

uint8 PENALTY_NONE = 0
uint8 PENALTY_ILLEGAL_POSITIONING = 1
uint8 PENALTY_MOTION_IN_SET = 2
uint8 PENALTY_MOTION_IN_STOP = 3
uint8 PENALTY_LOCAL_GAME_STUCK = 4
uint8 PENALTY_INCAPABLE_ROBOT = 5
uint8 PENALTY_PICK_UP = 6
uint8 PENALTY_BALL_HOLDING = 7
uint8 PENALTY_LEAVING_THE_FIELD = 8
uint8 PENALTY_PLAYING_WITH_ARMS_HANDS = 9
uint8 PENALTY_PUSHING = 10
uint8 PENALTY_CAUTIONED = 11
uint8 PENALTY_SENT_OFF = 12
uint8 PENALTY_SUBSTITUTE = 13
uint8 penalty

bool penalized
# Extra fields to differentiate between penalties, which will have the robot
# removed from the game to the sideline and penalties that happen in place
# on the field (e.g. MOTION_IN_SET)
bool penalized_in_place
uint8 seconds_till_unpenalized

# Number of yellow cards the player has received
uint8 cautions
bool has_yellow_card
bool has_red_card
