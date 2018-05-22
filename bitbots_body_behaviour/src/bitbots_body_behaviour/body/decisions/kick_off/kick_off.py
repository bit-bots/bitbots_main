# -*- coding:utf-8 -*-
"""
KickOff
^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_body_behaviour.body.decisions.common.role_decider import RoleDecider
from bitbots_body_behaviour.body.decisions.kick_off.kick_off_role_decider import KickOffRoleDecider
from bitbots_body_behaviour.body.decisions.kick_off.enemy_kick_off import EnemyKickOff

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class KickOff(AbstractDecisionModule):
    """
    Decides if there is a kick off
    """

    def __init__(self, connector, args):
        super(KickOff, self).__init__(connector, args)
        self.toggle_care_about_kickoff = connector.config["Body"]["Toggles"]["Fieldie"]["careAboutKickoff"]

    def perform(self, connector, reevaluate=False):
        if 0 < connector.gamestate.get_seconds_since_last_drop_ball() < 60:
            # there is a drop ball, normal behaviour
            return self.push(RoleDecider)
        else:
            if (self.toggle_care_about_kickoff and connector.gamestate.get_seconds_since_last_drop_ball() < 60 and
                    not connector.blackboard.has_performed_kickoff()):  # todo unset performed kickoff later
                # there is a kickoff and this is the first run in it

                connector.blackboard.set_performed_kickoff()
                # only run this once

                if connector.gamestate.has_kickoff():
                    # we have the kickoff
                    return self.push(KickOffRoleDecider)
                else:
                    # the other team has kickoff
                    return self.push(EnemyKickOff)
            else:
                return self.push(RoleDecider)

