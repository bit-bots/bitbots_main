import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class AllowedToMove(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AllowedToMove, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether the robot is penalized or was just unpenalized.
        :param reevaluate:
        :return:
        """
        if self.blackboard.gamestate.is_allowed_to_move():
            self.publish_debug_data("Seconds since unpenalized", self.blackboard.gamestate.get_seconds_since_unpenalized())
            rospy.logwarn(self.blackboard.gamestate.get_seconds_since_unpenalized())
            if self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
                return 'JUST_UNPENALIZED'
            else:
                return 'YES'
        else:
            return 'NO'

    def get_reevaluate(self):
        # Do not reevaluate, should only be reevaluated when the position was reached
        return True
