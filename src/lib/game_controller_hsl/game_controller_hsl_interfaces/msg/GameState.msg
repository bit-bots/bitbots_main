# This message provides all information from the game controller
# for additional information see documentation of the game controller
# https://github.com/bhuman/GameController


std_msgs/Header header

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

uint8 kicking_team
bool first_half
bool stopped
uint8 own_score
uint8 rival_score

# Seconds remaining for the game half
int16 secs_remaining
# Seconds remaining for things like kickoff
int16 secondary_time
bool penalized
uint16 seconds_till_unpenalized
uint8 warnings
uint8 cautions

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

uint8 message_budget
bool[] team_mates_with_penalty
