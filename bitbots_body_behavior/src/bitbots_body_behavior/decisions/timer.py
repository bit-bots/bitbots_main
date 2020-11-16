import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class TimerRunning(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(TimerRunning, self).__init__(blackboard, dsd, parameters)
        if 'name' not in parameters:
            raise Exception('TimerRunning: Name parameter is missing!')
        self.name = parameters['name']
        self.last_result = ''

    def perform(self, reevaluate=False):
        """
        Determines whether the timer is currently running.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data(f"timer {self.name}", self.blackboard.blackboard.timer_remaining(self.name))
        if self.blackboard.blackboard.timer_running(self.name):
            self.last_result = 'YES'
            return 'YES'
        self.last_result = 'NO'
        return 'NO'

    def get_reevaluate(self):
        return self.last_result == 'NO'


class TimerEnded(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(TimerEnded, self).__init__(blackboard, dsd, parameters)

        if 'name' not in parameters:
            raise Exception('TimerEnded: Name parameter is missing!')
        self.name = parameters['name']

    def perform(self, reevaluate=False):
        """
        Determines whether the timer has ended.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data(f"timer {self.name}", self.blackboard.blackboard.timer_remaining(self.name))

        if self.blackboard.blackboard.timer_ended(self.name):
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True