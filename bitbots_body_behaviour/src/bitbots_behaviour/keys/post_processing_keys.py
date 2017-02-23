# -*- coding:utf-8 -*-
"""
PostProcessing
^^^^^^^^^^^^^^

Enthält alle Keys für das PostProcessing der Sensordaten.

History:
''''''''

* 05.08.14: Created (Marc Bestmann)
"""

DATA_KEY_BALL_SPEED = "ballspeed"
""" Die Geschwindigkeit des Balles in m/s. Wenn es nicht bestimmt werden kann
-1 """

DATA_KEY_BALL_VECTOR = "ballvector"
""" Der bewegungsvector des Balles im Lokalen Koordinatensystem.
Geschwindigkeit in m/s """

DATA_KEY_BASELINE_INTERSECTION_DISTANCE = "baselineIntersectionDistance"
""" Die Distanz relativ zum Roboter, in der der Ball die Grundlinie des Roboters schneiden wird """

DATA_KEY_BASELINE_INTERSECTION_TIME = "baselineIntersectionTime"
""" Die Zeit bis der Ball die Grundlinie des Roboters schneiden wird """

DATA_KEY_BALL_INFO_FILTERED = "DataBallInfoFiltered"
""" The data from the BallInfoFilterModule """

DATA_KEY_RELATIVE_TO_GOAL_POSITION = "RelativeToGoalPosition"
""" Data Key for the Position relativ to two goal posts """

DATA_KEY_RELATIVE_TO_GOAL_POSITION_AVERAGED = "RelativeToGoalPositionAveraged"
""" Data Key for the Averaged relative Position """
