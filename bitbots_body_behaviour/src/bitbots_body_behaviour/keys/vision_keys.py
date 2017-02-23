# -*- coding:utf-8 -*-
"""
Vision
^^^^^^

Enthält alle Keys für die Vision.

History:
''''''''

* 05.08.14: Created (Marc Bestmann)
"""

DATA_KEY_OBSTACLE_FOUND = "ObstacleFound"
""" Ob ein Obstracel gefunden wurde """

DATA_KEY_OBSTACLE_INFO = "ObstacleInfo"
""" Liste der Obstracel die es gibt """

DATA_KEY_BALL_INFO = 'BallInfo'
""" Informationen zum Ball """

DATA_KEY_BALL_FOUND = 'BallFound'
""" Ball wurde gefunden """

DATA_KEY_BALL_HEAD_POSE = "BallHeadPose"
""" Kopfposition als tupel (headpan,headtilt) zum letzten Zeitpunkt als der ball gesehen wurde """

DATA_KEY_CAMERA_CAPTURE_TIMESTAMP = "CameraCaptureTimestamp"
""" Timestamp of the last image captured """

DATA_KEY_RAW_IMAGE = "RawImage"
""" The Image from the Camera """

DATA_KEY_CAMERA_FRAME_VERSION = "CameraFrameVersion"
""" An identifier for the number of the camera image frame """

DATA_KEY_BALL_LAST_SEEN = "BallLastSeen"
""" Wann der Ball zum letzten mal gesehen wurde """

DATA_KEY_BALL_INFO_SIMPLE_FILTERED = "BallInfoSimpleFiltered"
""" Simpel gefilterte Ballinformationen (ausgehend von den RAW-Daten) """

DATA_KEY_CURRENT_HORIZON_UV = "CurrentHorizonUV"
""" The UV Value - of the current Horizon """

DATA_KEY_CURRENT_HORIZON_ORIENTATION = "DATA_KEY_CURRENT_HORIZON_ORIENTATION"
""" The current orientation of the horizon: -1 / 0 / 1 (Tipping of the line) """

DATA_KEY_RAW_BALL = "Rawballlist"
""" A list off Ball named tupels, unferarbeitet"""

DATA_KEY_TRANSFORMER = "Transformer"
""" Der Transformer für vision basierte Dinge. Rechnet Bildpixel in u,v (in mm) um"""

DATA_KEY_GOAL_INFO_FILTERED = "GoalInfoFiltered"
""" Data Key for Filtered Goal Information """

DATA_KEY_GOAL_FOUND = "GoalFound"
""" Data Key for the Goal found boolean flag """

DATA_KEY_ANY_GOALPOST_LAST_SEEN = "AnyGoalpostLastSeen"
""" Data key for the last time a goalpost was recognised """

DATA_KEY_IS_NEW_FRAME = "IsNewFrame"
""" Data Key for New Frame boolean flag """

DATA_KEY_GOAL_INFO = "GoalInfo"
""" Data Key for Goal Info """

DATA_KEY_CAMERA_RESOLUTION = "CameraResulution"
""" Auflösung des Camerabilds """

DATA_KEY_IMAGE_FORMAT = "ImageFormat"
""" String der four letter code für die farbcodierung """

DATA_KEY_CAMERA_EXPOSURE_CALLBACK = "CameraExposureCalback"
""" Funktion mit der man anfragen kann, dass die Camera das Exposure anpasst"""

DATA_KEY_LINE_POINTS = "LinePoints"
""" Erkannte linen punkte"""

DATA_KEY_RAW_GOAL_DATA = "RawGoalData"
""" Like raw ball with goals"""

BALL_INFO_FILTERED = DATA_KEY_BALL_INFO_SIMPLE_FILTERED  # todo genauer evaluieren
""" Current best filtered ball info"""

DATA_KEY_IMAGE_PATH = "ImagePaths"
""" For datacamera """
