# -*- coding:utf-8 -*-
"""
Network
^^^^^^^

Enthält alle Keys für Netzwerkdaten.

History:
''''''''

* 05.08.14: Created (Marc Bestmann)
"""

DATA_GAME_STATUS = "GameStatus"
""" The Key for the current Game State """

DATA_KEY_TEAM_MATES = "TeamMates"
""" Die Daten der anderen Roboter welche über Mitecom empfangen wurden """

DATA_KEY_MINIMAL_BALL_TIME = "minimalballtime"
""" Die kürzeste Zeit die ein Teammitgliet zum Ball braucht """

DATA_KEY_BALL_TIME = "balltime"
""" Die Zeit die benötigt wird um den ball zu erreichen und sich
auszurichten """

DATA_KEY_ROLE = "teamrole"
""" Die Rolle welche der Roboter zurzeit ausfüllt. Mögliche Rollen sind
in :mod:´mitecom.mitecom´ als ROLE_* definiert """

DATA_KEY_GOALIE_BALL_RELATIVE_POSITION = 'teamgoalieballposition'
""" Die Position des Balles Relativ zum Goalie als tupel (x, y) in mm.

x = vorne, y links """

DATA_KEY_FIELDIE_BALL_TIME_LIST = 'teamfieldieballtimelist'
""" Eine Liste bestehend aus Tupeln von Roboterids und der Vermuteten Zeit die sie zum Ball brauchen. """

DATA_KEY_KICKOFF_OFFENSE_SIDE = 'KickoffOffenseSide'
"""
Key für die offence richtung nach dem kickoff

0 = unset

-1 = left

1 = right
"""
DATA_KEY_KICKOFF_OFFENSE_SIDE_RECEIVED = 'KickoffOffenseSideReceived'
""" The received value """

DATA_KEY_KICK_OFF_TIME = "KickOffNow"
""" There is a kick off"""

DATA_KEY_DROP_BALL_TIME = "DropBallNow"
""" There is a drop ball"""

DATA_KEY_OWN_KICK_OF = "OwnKickOff"
""" The key for boolean flag wheter we have or have not kick off """

DATA_KEY_OWN_GOALS = "EnemyGoals"
""" Count of Goals for the Enemy Team from Gamecontroller """

DATA_KEY_ENEMY_GOALS = "OwnGoals"
""" Count of Goals for our Team from Gamecontroller """

DATA_KEY_SECONDS_REMAINING = "RemainingSeconds"
""" Sekunden die im Aktuellen (Gamestate) """

DATA_KEY_SECONDAR_SECONDS_REMAINING = "SecondaryRemainingSeconds"
""" Sekunden im Sekundären Status (Gamestate) """

DATA_KEY_DROP_IN_TIME = "Gamecontrollerdropintimr"
""" Sekunden seit dem letzten Drop Ball """

###########################################
DATA_KEY_GAME_STATUS = "GameStatus"
""" The Key in the data dictionary for the GameStatus """

DATA_VALUE_STATE_SET = 2
""" The GameState SET """

DATA_VALUE_STATE_PLAYING = 3
""" The GameState PLAYING """

DATA_VALUE_STATE_READY = 1
""" The GameState READY """

DATA_VALUE_STATE_FINISHED = 4
""" The GameState FINISHED """

DATA_VALUE_STATE_INITIAL = 1
""" The GameStatus INITIAL """
###########################################

DATA_KEY_CENTROIDS = "Centroids"
""" TODO DOKU"""
