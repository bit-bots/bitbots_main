from bitbots_msgs.msg import Strategy
from dynamic_stack_decider import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class ChangeAction(AbstractActionElement):
    """
    Changes the action communicated to the other robots.
    This action determines the behavior of the robot on a very high level
    and should not be confused with the DSD actions.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard = blackboard

        self.action = parameters.get("action", None)
        self.actions = {
            "undefined": Strategy.ACTION_UNDEFINED,
            "positioning": Strategy.ACTION_POSITIONING,
            "going_to_ball": Strategy.ACTION_GOING_TO_BALL,
            "kicking": Strategy.ACTION_KICKING,
            "searching": Strategy.ACTION_SEARCHING,
            "localizing": Strategy.ACTION_LOCALIZING,
            "trying_to_score": Strategy.ACTION_TRYING_TO_SCORE,
            "waiting": Strategy.ACTION_WAITING,
        }

    def perform(self, reevaluate=False):
        self.blackboard.team_data.set_action(self.actions[self.action])
        self.pop()
