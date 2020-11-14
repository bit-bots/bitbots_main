import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class TimerRunning(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(TimerRunning, self).__init__(blackboard, dsd, parameters)
        if 'name' not in parameters:
            raise Exception('TimerRunning: Name parameter is missing!')

    def perform(self, reevaluate=False):
        """
        Determines whether the timer is currently running.
        :param reevaluate:
        :return:
        """
        if self.blackboard.blackboard.timer_running():
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True


class TimerEnded(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(TimerEnded, self).__init__(blackboard, dsd, parameters)
        if 'name' not in parameters:
            raise Exception('TimerEnded: Name parameter is missing!')

    def perform(self, reevaluate=False):
        """
        Determines whether the timer has ended.
        :param reevaluate:
        :return:
        """
        if self.blackboard.blackboard.timer_ended():
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True