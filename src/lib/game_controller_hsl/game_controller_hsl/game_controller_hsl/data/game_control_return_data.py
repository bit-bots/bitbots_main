#!/usr/bin/env python
from construct import (
    Byte,
    Const,
    Flag,
    Float32l,
    Struct,
)

GAME_CONTROLLER_RETURN_STRUCT_VERSION = 4
GAME_CONTROLLER_RETURN_STRUCT_HEADER = b"RGrt"

GameControlReturnDataStruct = Struct(
    "header" / Const(GAME_CONTROLLER_RETURN_STRUCT_HEADER),
    "version" / Const(GAME_CONTROLLER_RETURN_STRUCT_VERSION, Byte),
    "player_number" / Byte,
    "team_number" / Byte,
    "fallen" / Flag,
    "pose" / Float32l[3],
    "ball_age" / Float32l,
    "ball" / Float32l[2],
)
