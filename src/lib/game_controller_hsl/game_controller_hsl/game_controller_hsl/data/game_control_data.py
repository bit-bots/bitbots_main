#!/usr/bin/env python
# -*- coding:utf-8 -*-

from construct import (
    Array,
    Byte,
    Const,
    Enum,
    Flag,
    Int16sl,
    Int16ul,
    Struct,
)

GAME_CONTROLLER_STRUCT_VERSION = 20
GAME_CONTROLLER_STRUCT_HEADER = b"RGme"

MAX_NUM_PLAYERS = 20

Short = Int16ul

RobotInfoStruct = Struct(
    "penalty" / Enum(
        Byte,
        PENALTY_NONE=0,
        PENALTY_ILLEGAL_POSITIONING=1,
        PENALTY_MOTION_IN_SET=2,
        PENALTY_MOTION_IN_STOP=3,
        PENALTY_LOCAL_GAME_STUCK=4,
        PENALTY_INCAPABLE_ROBOT=5,
        PENALTY_PICK_UP=6,
        PENALTY_BALL_HOLDING=7,
        PENALTY_LEAVING_THE_FIELD=8,
        PENALTY_PLAYING_WITH_ARMS_HANDS=9,
        PENALTY_PUSHING=10,
        PENALTY_CAUTIONED=11,
        PENALTY_SENT_OFF=12,
        PENALTY_SUBSTITUTE=13,
    ),
    "secs_till_unpenalized" / Byte,
    "cautions" / Byte,
)

TeamInfoStruct = Struct(
    "team_number" / Byte,
    "field_player_color"
    / Enum(
        Byte,
        BLUE=0,
        RED=1,
        YELLOW=2,
        BLACK=3,
        WHITE=4,
        GREEN=5,
        ORANGE=6,
        PURPLE=7,
        BROWN=8,
        GRAY=9,
    ),
    "goalkeeper_color"
    / Enum(
        Byte,
        BLUE=0,
        RED=1,
        YELLOW=2,
        BLACK=3,
        WHITE=4,
        GREEN=5,
        ORANGE=6,
        PURPLE=7,
        BROWN=8,
        GRAY=9,
    ),
    "goalkeeper" / Byte,
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "message_budget" / Short,  # number of team messages the team is allowed to send for the remainder of the game
    "players" / Array(MAX_NUM_PLAYERS, RobotInfoStruct),
)

GameControlDataStruct = Struct(
    "header" / Const(GAME_CONTROLLER_STRUCT_HEADER),
    "version" / Const(GAME_CONTROLLER_STRUCT_VERSION, Byte),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "competition_type"
    / Enum(
        Byte,
        COMPETITION_TYPE_SMALL=0,
        COMPETITION_TYPE_MIDDLE=1,
        COMPETITION_TYPE_LARGE=2,
    ),
    "stopped" / Flag,
    "game_phase"
    / Enum(
        Byte,
        GAME_PHASE_NORMAL=0,
        GAME_PHASE_PENALTY_SHOOT_OUT=1,
        GAME_PHASE_EXTRA_TIME=2,
        GAME_PHASE_TIMEOUT=3,
    ),
    "state"
    / Enum(
        Byte,
        STATE_INITIAL=0,
        STATE_READY=1,  # Go to (starting) position
        STATE_SET=2,  # Hold position (no motion allowed)
        STATE_PLAYING=3,  # Normal play
        STATE_FINISHED=4,  # Game over
    ),
    "set_play"
    / Enum(
        Byte,
        SET_PLAY_NONE=0,
        SET_PLAY_DIRECT_FREE_KICK=1,
        SET_PLAY_INDIRECT_FREE_KICK=2,
        SET_PLAY_PENALTY_KICK=3,
        SET_PLAY_THROW_IN=4,
        SET_PLAY_GOAL_KICK=5,
        SET_PLAY_CORNER_KICK=6,
    ),
    "first_half" / Flag,
    "kicking_team" / Byte,
    "secs_remaining" / Int16sl,
    "secondary_time" / Int16sl,
    "teams" / Array(2, TeamInfoStruct),
)
