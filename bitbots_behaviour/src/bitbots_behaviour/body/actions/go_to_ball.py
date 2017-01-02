# -*- coding:utf-8 -*-
"""
GoToBall
^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 10.12.13: Created from old behaviour (Martin Poppinga)

* 12.03.14: Intelligenterer Weg zum Ball (Judith Hartfill)

* 19.08.14: Balltracking nur noch wenn nahe am ball, mathefehler behoben

Goes to the ball, looking at distance and degree of the ball.
"""
import math

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.util import get_config


class GoToBall(AbstractActionModule):
    """
    Goes to the ball
    """

    def __init__(self, args="Normal"):
        super(GoToBall, self).__init__()
        config = get_config()
        self.camera_angle = config["Behaviour"]["Common"]["Camera"]["cameraAngle"]
        self.searching_turn_angular_absolute = config["Behaviour"]["Fieldie"]["searchingTurnAngularAbsolute"]
        self.max_kick_distance = config["Behaviour"]["Fieldie"]["kickDistance"]
        self.min_kick_distance = config["Behaviour"]["Fieldie"]["minKickDistance"]
        self.state = args

    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().unset_dont_need_ball()
        ballinfo_u = connector.raw_vision_capsule().get_ball_info("u")
        ballinfo_v = connector.raw_vision_capsule().get_ball_info("v")

        assert ballinfo_u is not None
        assert ballinfo_v is not None

        if ballinfo_u != 0:
            winkel = math.degrees(math.atan(ballinfo_v / ballinfo_u))
        else:
            winkel = 0

        distancefactor = int(ballinfo_u / 50.0) + 4
        capsule_obj = connector.walking_capsule()
        capsule_fwd = capsule_obj.ZERO
        capsule_ang = capsule_obj.ZERO
        capsule_swd = capsule_obj.ZERO

        if ballinfo_u < self.min_kick_distance:
            # close Ball (ztoo close)
            if abs(ballinfo_v) > 250:
                # we are able to reach the ball while goinf backwards, without loosing it
                capsule_fwd = capsule_obj.MEDIUM_BACKWARD
            else:
                capsule_fwd = capsule_obj.SLOW_BACKWARD

                if self.sign(ballinfo_v) < 0:
                    capsule_swd = capsule_obj.SLOW_SIDEWARDS_LEFT
                else:
                    capsule_swd = capsule_obj.SLOW_SIDEWARDS_RIGHT

        elif connector.raw_vision_capsule().get_ball_info("distance") <= 800:  # TODO: entfernung prüfen
            # if the distance is big enough, dont turn

            if abs(ballinfo_v) < 100:
                # we are gooing staight to the ball
                capsule_swd = capsule_obj.ZERO
            else:
                if self.sign(ballinfo_v) < 0:
                    capsule_swd = capsule_obj.SLOW_SIDEWARDS_LEFT
                else:
                    capsule_swd = capsule_obj.SLOW_SIDEWARDS_RIGHT

            if ballinfo_u > 350:
                capsule_fwd = capsule_obj.MEDIUM_FORWARD
            else:
                capsule_fwd = capsule_obj.SLOW_FORWARD

        else:
            # balll is too far, we are running curves
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

        capsule_obj.start_walking(capsule_fwd, capsule_ang, capsule_swd)

        if connector.raw_vision_capsule().get_ball_info("distance") < 10000:
            connector.blackboard_capsule().schedule_ball_tracking()


class GoToBallPenaltykick(AbstractActionModule):  # todo not yet refactored 6.12.14.

    def perform(self, connector, reevaluate=False):

        connector.blackboard_capsule().unset_dont_need_ball()
        ballinfo_u = connector.raw_vision_capsule().get_ball_info("u")
        ballinfo_v = connector.raw_vision_capsule().get_ball_info("v")

        assert ballinfo_u is not None
        assert ballinfo_v is not None

        if connector.raw_vision_capsule().get_ball_info("u") != 0:
            winkel = (math.atan(ballinfo_v / ballinfo_u) / 3.1415) * 180
        else:
            winkel = 0

        distancefactor = int(ballinfo_u / 100.0) + 4

        if winkel != 0:
            anglefactor = (2 / (ballinfo_u / 1000.0)) * (winkel / 5)
            distancefactor = (distancefactor / (abs(anglefactor))) + 3  # %TODO Prüfen [martin] 10.5.2014
        else:
            anglefactor = 0
        connector.walking_capsule().start_walking_plain(distancefactor, anglefactor)

        connector.blackboard_capsule().schedule_ball_tracking()
