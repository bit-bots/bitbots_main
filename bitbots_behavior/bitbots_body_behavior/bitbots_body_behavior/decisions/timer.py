from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class TimerRunning(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        if "name" not in parameters:
            raise KeyError("TimerRunning: Name parameter is missing!")
        self.name = parameters["name"]
        self.last_result = ""

    def perform(self, reevaluate=False):
        """
        Determines whether the timer is currently running. Is only reevaluated if the last result was 'NO'.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data(f"timer {self.name}", self.blackboard.misc.timer_remaining(self.name))
        if self.blackboard.misc.timer_running(self.name):
            self.last_result = "YES"
        else:
            self.last_result = "NO"
        return self.last_result

    def get_reevaluate(self):
        """
        This decision is only evaluated if the last result was 'NO'.
        This allows to define a situation in which the timer has ended after it was started before.
        :return: Whether the decision is reevaluated (When the last result was 'NO')
        """
        return self.last_result == "NO"


class TimerEnded(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        if "name" not in parameters:
            raise KeyError("TimerEnded: Name parameter is missing!")
        self.name = parameters["name"]

    def perform(self, reevaluate=False):
        """
        Determines whether the timer has ended.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data(f"timer {self.name}", self.blackboard.misc.timer_remaining(self.name))

        if self.blackboard.misc.timer_ended(self.name):
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
