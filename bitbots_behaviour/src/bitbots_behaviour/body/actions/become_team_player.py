"""
BecomeRunningGoalie
^^^^^^^^^^^^^^^^^^^

Changes its role to a fieldie. This is mostly a semantic module for the behaviour graph.

"""
from bitbots_common.stackmachine import AbstractDecisionModule


class BecomeTeamPlayer(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().set_goalie_out_of_goal(True)
        connector.set_duty("TeamPlayer")
        return self.interrupt()
