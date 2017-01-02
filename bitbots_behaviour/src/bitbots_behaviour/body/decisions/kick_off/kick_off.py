# -*- coding:utf-8 -*-
"""
KickOff
^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 29.11.13: Created (Martin Poppinga)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.decisions.common.role_decider import RoleDecider
from bitbots.modules.behaviour.body.decisions.kick_off.kick_off_role_decider import KickOffRoleDecider
from bitbots.modules.behaviour.body.decisions.kick_off.wait_for_enemy_kick_off import WaitForEnemyKickOff
from bitbots.util import get_config


class KickOff(AbstractDecisionModule):
    """
    Decided if there is a kick off

    """

    def __init__(self, _):
        super(KickOff, self).__init__()
        config = get_config()["Behaviour"]
        self.toggle_care_about_kickoff = config["Toggles"]["Fieldie"]["careAboutKickoff"]

    def perform(self, connector, reevaluate=False):

        if 0 < connector.gamestatus_capsule().get_seconds_since_last_drop_ball() < 60:
            # there is a drop ball, normal behaviour
            return self.push(RoleDecider)
        else:
            if self.toggle_care_about_kickoff and \
                    connector.gamestatus_capsule().get_seconds_since_last_drop_ball() < 60 and \
                    not connector.blackboard_capsule().has_performed_kickoff():  # todo testme
                # there is a kickoff and this is the first run in it

                connector.blackboard_capsule().set_performed_kickoff()
                # only run this onece

                if connector.gamestatus_capsule().has_kickoff():
                    # we have the kickoff
                    return self.push(KickOffRoleDecider)
                else:
                    # the other team has kickoff
                    return self.push(WaitForEnemyKickOff)
            else:
                return self.push(RoleDecider)

