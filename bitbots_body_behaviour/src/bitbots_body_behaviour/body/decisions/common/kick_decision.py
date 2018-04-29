# -*- coding:utf-8 -*-
"""
Kick Decision
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.actions.kick_ball import KickBall


class AbstractKickDecision(AbstractDecisionModule):
    """
    Decides wich leg he has to use to kick the ball and if he has te repositionate before kicking
    """

    def __init__(self, connector, _):
        super(AbstractKickDecision, self).__init__(connector)
        self.max_goal_hard_distance = connector.config["Body"]["Fieldie"]["maxGoalHardKickDistance"]
        self.toggle_use_side_kick = connector.config["Body"]["Toggles"]["Fieldie"]["useSideKickInGame"]
        self.use_dynamic_kick_toggle = connector.config["Body"]["Toggles"]["Fieldie"]["useDynamicKick"]

    def perform(self, connector, reevaluate=False):

        self.action(connector, reevaluate)

    def action(self, connector, reevaluate):
        self.do_not_reevaluate()
        connector.blackboard.unset_finished_align()

        # TODO improve
        if not self.toggle_use_side_kick \
                or not (connector.world_model.get_opp_goal_distance() < 1
                        and connector.world_model.get_opp_goal_angle() > 30):

            if False and connector.world_model.get_opp_goal_distance() > self.max_goal_hard_distance:
                return self.hard_kick(connector)
            else:
                return self.kick_normal(connector)

        else:
            return self.kick_side_goal(connector)

    def kick_normal(self, connector):
        """
        Pushes a normal kick, depending on side of the Ball
        """
        if connector.vision.get_ball_relative()[1] <= 0:
            return self.push(KickBall, init_data="R")
        else:
            return self.push(KickBall, init_data="L")

    def kick_side_goal(self, connector):
        """
        Pushes a sidekick, depending on the side of the enemy goal
        """
        if connector.world_model.get_opp_goal_center_uv()[1] > 0:
            return self.push(KickBall, init_data="SRK")
        else:
            return self.push(KickBall, init_data="SLK")

    def hard_kick(self, connector):
        """
        Pushes a hard kick, depending on side of the ball
        :param connector:
        :return:
        """

        if connector.world_model.get_ball_position_uv()[1] <= 0:
            return self.push(KickBall, init_data="RP")
        else:
            return self.push(KickBall, init_data="LP")

    def get_reevaluate(self):
        return True


class KickDecisionCommon(AbstractKickDecision):
    pass


class KickDecisionPenaltyKick(AbstractKickDecision):
    def action(self, connector, reevaluate):
        if connector.raw_vision_capsule().get_ball_info("v") <= 0:
            return self.push(KickBall, init_data="R")
        else:
            return self.push(KickBall, init_data="L")
