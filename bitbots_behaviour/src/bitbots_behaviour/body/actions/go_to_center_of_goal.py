# -*- coding:utf-8 -*-
"""
FocusBall
^^^^^^^^^^

.. moduleauthor:: Fabian Fiedler <0fiedler@informatik.uni-hamburg.de>

History:

* 2.04.14 erstellt

"""

import time
import math

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.util import get_config
from bitbots.util.speaker import say


class GoToCenterOfGoal(AbstractActionModule):
    def __init__(self, _):
        super(GoToCenterOfGoal, self).__init__()
        config = get_config()
        self.goalWidth = config["Field"]["goalWidth"]
        self.drehungsFaktor = 1.05
        self.laufeSeit = None
        self.time = time.time()

    def perform(self, connector, reevaluate=False):
        """
        Diese Methode richtet den Goalie aus, erstmal alles stueck fuer stueck
        This Method repositionates the goalie, for now step by step.
        """
        if time.time() - self.time > 15:
            self.debug("Mehr als 15 Sekunden das Tor gesucht, ich gebe es auf")
            connector.blackboard_capsule().set_priorities(own_goal_priority=0, ball_priority=1000, align_priority=0)
            connector.blackboard_capsule().set_thrown(False)
            return self.interrupt()

        if connector.raw_vision_capsule().own_goal_seen():
            # todo testen ob own_goal_seen geht, sonst schauen ob tor gesehen und näherdran als hälfte des spielfeldes
            # aktivate the tracking
            connector.blackboard_capsule().schedule_own_goal_tracking()

            goal_u = connector.filtered_vision_info_capsule().get_center_of_seen_goal()[0]
            goal_v = connector.filtered_vision_info_capsule().get_center_of_seen_goal()[1]
            distance = (goal_u ** 2 + goal_v ** 2)

            if goal_u != 0:
                winkel = math.degrees((math.atan(goal_u / goal_v) / 3.1415) * 180)
            else:
                winkel = 0

            capsule_obj = connector.walking_capsule()
            capsule_fwd = capsule_obj.ZERO
            capsule_ang = capsule_obj.ZERO
            capsule_swd = capsule_obj.ZERO

            if distance < 300:
                # wir stehen nahe genug
                say("Im in the center of my goal")
                self.pop()
            elif distance < 5000:
                # es ist nicht das entfernte (vermutlich gegnerische)tor
                capsule_fwd = capsule_obj.FAST_FORWARD
                if abs(winkel) > 10:
                    capsule_fwd = capsule_obj.MEDIUM_FORWARD
                    if winkel > 0:
                        if winkel > 45:
                            capsule_ang = capsule_obj.MEDIUM_ANGULAR_RIGHT
                        else:
                            capsule_ang = capsule_obj.SLOW_ANGULAR_RIGHT
                    elif winkel:
                        if winkel < -45:
                            capsule_ang = capsule_obj.MEDIUM_ANGULAR_LEFT
                        else:
                            capsule_ang = capsule_obj.SLOW_ANGULAR_LEFT
                else:
                    capsule_ang = capsule_obj.ZERO
            else:
                say("i dont want to go to this goal. its to far away")
                self.interrupt()
                # todo bessere fehlerbehandlung, zB genau so laufen, dass man weit weg davon ist oder dadran ausrichten

            capsule_obj.start_walking(capsule_fwd, capsule_ang, capsule_swd)

            """
            goalInfo = connector.use_behaviour_blackboard_capsule().getGoalieGoal()
            if goalInfo[0].u > 0 and goalInfo[1].u > 0: # Stehen hinter der Torlinie
                self.debug("Stehe hinter der Torlinie")
                connector.start_walking(3, 0)
                self.laufeSeit = time.time()

            elif goalInfo[1].u > goalInfo[0].u * self.drehungsFaktor:
            # Das Rechte Tor sieht auf der u Achse weiter Weg aus, d.h. der Winkel ist Falsch
                self.debug("Linksdrehung")
                connector.start_walking(0, -3)                            # Linksdrehung um den Winkel anzupassen
                self.laufeSeit = time.time()

            elif goalInfo[0].u > goalInfo[1].u * self.drehungsFaktor: # Siehe Oben. Rechtsrehung
                self.debug("Rechtsdrehung")
                connector.start_walking(0, 3)
                self.laufeSeit = time.time()

            elif goalInfo[0].v < - (self.goalWidth/2):
                self.debug("Linkes Tor Weiter weg als es sollte")
                connector.start_walking(0,0,3)
                self.laufeSeit = time.time()

            elif goalInfo[1].v > self.goalWidth/2:
                self.debug("REchtes Tor weiter weg als es sollte")
                connector.start_walking(0,0,-3)
                self.laufeSeit = time.time()

            else:
                self.debug("Ich stehe gut")
                connector.use_behaviour_blackboard_capsule().setThrown(False)
                return self.pop()

            if time.time() - self.laufeSeit >= 2:
                connector.use_walking_capsule().stop_walking()
                self.debug("Höre auf zu laufen ,da etwas gedreht")
        """
