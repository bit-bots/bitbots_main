#!/usr/bin/env python
# -*- coding:utf-8 -*-

from construct import Byte, Struct, Enum, Bytes, Const, Array, Int16ul, PaddedString, Flag, Int16sl, Float32l

Short = Int16ul

RobotInfoStruct = "robot_info" / Struct(
    # define NONE                        0
    # define PENALTY_HL_KID_BALL_MANIPULATION    1
    # define PENALTY_HL_KID_PHYSICAL_CONTACT     2
    # define PENALTY_HL_KID_ILLEGAL_ATTACK       3
    # define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
    # define PENALTY_HL_KID_REQUEST_FOR_SERVICE  6
    # define PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
    # define MANUAL                      15
    "penalty" / Byte,
    "secs_till_unpenalized" / Byte,
    "warnings" / Byte,
    "cautions" / Byte,
    #"number_of_red_cards" / Byte,
    #"goalkeeper" / Flag
)

TeamInfoStruct = "team" / Struct(
    "team_number" / Byte,
    "field_player_color" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9
                        ),
    "goalkeeper_color" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9
                        ),
    "goalkeeper" / Byte,
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "message_budget" / Short,
    "players" / Array(20, RobotInfoStruct) #always eleven fine?
)

GAME_CONTROLLER_RESPONSE_VERSION = 19

GameStateStruct = "gamedata" / Struct(
    "header" / Const(b'RGme'),
    "version" / Const(GAME_CONTROLLER_RESPONSE_VERSION, Byte),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "competition_type" / Enum(Byte,
                        COMPETITION_TYPE_SMALL=0,
                        COMPETITION_TYPE_MIDDLE=2,
                        COMPETITION_TYPE_LARGE=1
                        ),
    "stopped" / Flag,
    "game_phase" / Enum(Byte,
                        GAME_PHASE_TIMEOUT=0,
                        GAME_PHASE_NORMAL=1,
                        GAME_PHASE_EXTRA_TIME =2,
                        GAME_PHASE_PENALTY_SHOOT_OUT=3
                        ),
    "state" / Enum(Byte,
                        STATE_INITIAL=0,
                        # auf startposition gehen
                        STATE_READY=1,
                        # bereithalten
                        STATE_SET=2,
                        # spielen
                        STATE_PLAYING=3,
                        # spiel zu ende
                        STATE_FINISHED=4
                        ),
    "set_play" / Enum(Byte,
                         SET_PLAY_NONE=0,
                         SET_PLAY_DIRECT_FREE_KICK=1,
                         SET_PLAY_INDIRECT_FREE_KICK=2,
                         SET_PLAY_PENALTY_KICK=3,
                         SET_PLAY_THROW_IN=4,
                         SET_PLAY_GOAL_KICK=5,
                         SET_PLAY_CORNER_KICK=6
                         ),
    "first_half" / Flag,
    "kicking_team" / Byte,
    "secs_remaining" / Int16sl,
    "secondary_time" / Int16sl,
    "teams" / Array(2, "team" / TeamInfoStruct)
)

GAME_CONTROLLER_RESPONSE_VERSION = 4

ResponseStruct = Struct(
    "header" / Const(b"RGrt"),
    "version" / Const(GAME_CONTROLLER_RESPONSE_VERSION, Byte),
    "player_number" / Byte,
    "team_number" / Byte,
    "fallen" / Flag,
    "pose" / Float32l[3],
    "ball_age" / Float32l,
    "ball" / Float32l[2]
)
