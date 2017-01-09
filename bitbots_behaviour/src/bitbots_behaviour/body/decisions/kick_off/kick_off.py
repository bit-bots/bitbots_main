# -*- coding:utf-8 -*-
"""
KickOff
^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rosparam
from bitbots_common.stackmachine.abstract_decision_module import AbstractDecisionModule
from body.decisions.common.role_decider import RoleDecider
from body.decisions.kick_off.kick_off_role_decider import KickOffRoleDecider
from body.decisions.kick_off.wait_for_enemy_kick_off import WaitForEnemyKickOff


class KickOff(AbstractDecisionModule):
    """
    Decided if there is a kick off
    """

    def __init__(self, _):
        super(KickOff, self).__init__()
        self.toggle_care_about_kickoff = rosparam.get_param("/Behaviour/Toggles/Fieldie/careAboutKickoff")

    def perform(self, connector, reevaluate=False):
        return self.push(RoleDecider)
        if 0 < connector.gamestate.get_seconds_since_last_drop_ball() < 60:
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

