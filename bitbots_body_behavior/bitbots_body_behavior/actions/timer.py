from bitbots_blackboard.blackboard import BodyBlackboard

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class StartTimer(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(StartTimer, self).__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard

        if 'name' not in parameters:
            raise KeyError('StartTimer: Name parameter is missing!')
        self.timer_name = parameters['name']
        if 'duration' not in parameters:
            raise KeyError(f'StartTimer ({self.timer_name}): Duration parameter is missing!')
        self.duration = parameters['duration']

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.start_timer(self.timer_name, self.duration)
        return self.pop()


class EndTimer(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(EndTimer, self).__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard

        if 'name' not in parameters:
            raise KeyError('EndTimer: Name parameter is missing!')
        self.timer_name = parameters['name']

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.end_timer(self.timer_name)
        return self.pop()
