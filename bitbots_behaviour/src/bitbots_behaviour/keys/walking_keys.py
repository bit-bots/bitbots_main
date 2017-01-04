# -*- coding:utf-8 -*-
"""
Walking Keys
^^^^^^^^^^^^

Enthält alle Keys für das Walking.

History:
''''''''

* 07.08.14: Created (Marc Bestmann)
"""

DATA_KEY_WALKING = "Walking"
""" Walking """

DATA_KEY_CAMERA_FOOT_PHASE = "CameraFootPhase"
""" Auf welchem Fuß er steht, zum Zeitpunkt des Kamerabilds, aus dem Walking"""

DATA_KEY_SUPPORT_LEG = "SupportLeg"
""" Bein auf dem der Roboter steht, berechnet durch den Transformer, zum Zeitpunkt des Kamerabilds"""

DATA_KEY_ZMP_WALKING = "ZMPWalking"
""" Das ZMP Walking, dass nicht in die Motion runtergezogen wurde """

DATA_KEY_MOTION_WALKING = "MotionWalking"
""" Stellt das zmp walking, dass in der motion implementiert ist bereit """

DATA_KEY_OLD_WALKING = "OldWalking"
""" Stellt das alte zmp-lose walking bereit"""

DATA_KEY_WALKING_FORWARD = "Walking.Forward"
""" **erwünschte** Vorwärtsgeschwindigkeit des Walkings"""

DATA_KEY_WALKING_SIDEWARD = "Walking.Sideward"
""" **erwünschte** Seitwärtsgeschwindigkeit des Walkings """

DATA_KEY_WALKING_ANGULAR = "Walking.Angular"
""" **erwünschte** Drehgeschwindigkeit des Walkings """

DATA_KEY_WALKING_ACTIVE = "Walking.Active"
""" Ob das Walking als aktiv gewünscht ist (nicht wirklich ist, siehe DATA_KEY_WALKING_RUNNING) """

DATA_KEY_WALKING_HIP_PITCH_IN = "Walking.HipPitch.In"
""" TODO: Sinnvoller kommentar"""

DATA_KEY_WALKING_HIP_PITCH_OUT = "Walking.HipPitch.Out"
""" TODO: Sinnvoller kommentar"""

DATA_KEY_WALKING_ARMS = "Walking.Arme"
""" TODO Sinnvoller kommentar, vermutlich wird hier festgelegt ob die arme beim laufen bewegt werden sollen oder nich"""

DATA_KEY_WALKING_FORWARD_REAL = "Walking.Forward.Real"
""" Der wirkliche wert mit dem das Walking gerade geradeaus läuft """

DATA_KEY_WALKING_SIDEWARD_REAL = "Walking.Sideward.Real"
""" Der wirkliche wert mit dem das Walking gerade seitwärts läuft """

DATA_KEY_WALKING_ANGULAR_REAL = "Walking.Angular.Real"
""" Der wirkliche wert mit dem das Walking gerade dreht """

DATA_KEY_WALKING_RUNNING = "Walking.Running"
""" Ob das Running selbst sagt dass es läuft,
dies ist etwas anderes als DATA_KEY_WALKING_ACTIVE,
weil das Walking einen Moment braucht, bis es wirklich läuft"""
